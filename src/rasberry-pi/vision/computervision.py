#!/usr/bin/env python3
#in order to use this code properly, copy, paste, and run "pip install pyserial" in the Command Prompt (windows) / Terminal (Mac)
#This will install the neccesary library to use serial communication with Pyhton
"""
WRO 2026 FUTURE ENGINEERS - Tamagotchi-Triplets
Complete autonomous driving pipeline: Vision + State Machine + Serial (Pi ↔ Arduino)

HARDWARE ARCHITECTURE:
- Raspberry Pi 3B+:  Computer Vision (OpenCV), State Machine
- Arduino/ESP32:     Motor control, Ultrasonic Sensors (USS)

SERIAL PROTOCOL (Pi → Arduino):
    Format: "steering,speed,state,colour1,colour2,phase\n"
    Example: "0.350,0.500,OBSTACLE,green,None,ALIGN\n"

    Fields:
    - steering:  float -1.0 (full left) to +1.0 (full right)
    - speed:     float 0.0 (stop) to 1.0 (full speed) - THIS IS DRIVING SPEED, not wheel RPM
    - state:     string (STRAIGHT, CORNER, OBSTACLE, PARKING, EMERGENCY_STOP)
    - colour1:   string (blue, orange, red, green, magenta, None)
    - colour2:   string (secondary colour, for debugging)
    - phase:     string (APPROACH, ALIGN, PASSING, RECOVER) - for traffic lights

SERIAL PROTOCOL (Arduino → Pi):
    Format: "front_cm,left_cm,right_cm\n"
    Example: "25.3,12.1,8.7\n"
    - All distances in centimetres
"""

import cv2
import numpy as np
from collections import namedtuple
from picamera import PiCamera, PiRGBArray
import time
import logging
import serial
import sys
import signal
import glob

# Global Constants (Official Rulebook Baselines)

# Official Rulebook Specs (converted to OpenCV HSV)
OFFICIAL_COLOUR_BASELINES = {
    'blue':   (np.array([100, 150, 80]),  np.array([130, 255, 255])),
    'orange': (np.array([5, 120, 120]),   np.array([15, 255, 255])),
    'red1':   (np.array([0, 120, 70]),    np.array([10, 255, 255])),
    'red2':   (np.array([170, 120, 70]),  np.array([180, 255, 255])),
    'green':  (np.array([40, 100, 70]),   np.array([80, 255, 255])),
    'magenta':(np.array([140, 100, 70]),  np.array([170, 255, 255])),
}

MIN_LINE_AREA = 20
MIN_PILLAR_AREA = 150
MIN_PARKING_AREA = 80
CORNER_PIXEL_THRESHOLD = 120

# Serial configuration (Match this with Arduino/ESP32)
ARDUINO_PORT = '/dev/ttyACM0'  # Change as needed
BAUD_RATE = 115200
CONTROL_LOOP_DT = 0.05  # 20 Hz

# 1. OUTPUT DATA STRUCTURE

VisionResult = namedtuple('VisionResult', [
    'lane_offset_px', 'lane_offset_normalized', 'track_width_px', 'lane_confidence',
    'corner_detected', 'corner_colour',
    'traffic_sign_colour', 'traffic_sign_x', 'traffic_sign_area', 'obstacle_side_required',
    'parking_markers', 'magenta_detected'
])


# 2. VISION PROCESSOR CLASS

class BossModeVision:
    """Adaptive vision pipeline with dynamic thresholding."""
    def __init__(self, resolution=(320, 240), framerate=15, driving_direction=1):
        """
        Args:
            resolution: (width, height) for camera capture
            framerate: Target FPS
            driving_direction: 1 = clockwise (normal), -1 = counter-clockwise (reversed)
                               This flips corner detection so blue/orange is interpreted correctly
                               regardless of which way the car is facing.
        """
        self.width, self.height = resolution
        self.framerate = framerate
        self.driving_direction = driving_direction  # 1 or -1
        self.slack_s = 0
        self.slack_v = 0
        self.frame_count = 0

        # Camera setup
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.exposure_mode = 'off'
        self.camera.shutter_speed = 10000
        self.camera.iso = 400
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = (1.0, 1.0)
        time.sleep(0.5)
        self.raw_capture = PiRGBArray(self.camera, size=resolution)
        logging.info(f"Vision initialised. Direction: {'CW' if driving_direction == 1 else 'CCW'}")

    def __enter__(self):
        return self
    def __exit__(self, *args):
        self.camera.close()

    def _get_hsv(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    def _get_mask(self, hsv, colour_key):
        def _apply_slack(low, high):
            low = low.copy(); high = high.copy()
            low[1] = max(0, int(low[1]) - self.slack_s)
            low[2] = max(0, int(low[2]) - self.slack_v)
            high[1] = min(255, int(high[1]) + self.slack_s)
            high[2] = min(255, int(high[2]) + self.slack_v)
            return low, high

        if colour_key == 'red':
            l1, h1 = _apply_slack(*OFFICIAL_COLOUR_BASELINES['red1'])
            l2, h2 = _apply_slack(*OFFICIAL_COLOUR_BASELINES['red2'])
            return cv2.bitwise_or(cv2.inRange(hsv, l1, h1), cv2.inRange(hsv, l2, h2))
        low, high = _apply_slack(*OFFICIAL_COLOUR_BASELINES[colour_key])
        return cv2.inRange(hsv, low, high)

    def _filter_contours(self, mask, min_area):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [c for c in cnts if cv2.contourArea(c) >= min_area]

    # Module A: Lane keeping
    def _find_dynamic_roi(self, hsv):
        mask = cv2.bitwise_or(self._get_mask(hsv, 'blue'), self._get_mask(hsv, 'orange'))
        row_sums = np.sum(mask, axis=1)
        non_zero_rows = np.where(row_sums > 20)[0]
        if len(non_zero_rows) > 10:
            horizon = np.min(non_zero_rows)
        else:
            horizon = int(self.height * 0.4)
        return max(0, horizon - 10)

    def detect_lane(self, hsv):
        horizon = self._find_dynamic_roi(hsv)
        roi = hsv[horizon:, :]
        h_roi, w_roi = roi.shape[:2]
        mask_lane = cv2.bitwise_or(self._get_mask(roi, 'blue'), self._get_mask(roi, 'orange'))
        bottom_roi = mask_lane[int(h_roi*0.33):, :]
        hist = np.sum(bottom_roi, axis=0, dtype=np.int32)
        if np.max(hist) < 15:
            self.slack_s = min(50, self.slack_s + 5)
            self.slack_v = min(50, self.slack_v + 5)
            return 0.0, 0.0, 0.0, 0.0
        self.slack_s = max(0, self.slack_s - 2)
        self.slack_v = max(0, self.slack_v - 2)
        kernel = np.ones(5, dtype=np.float32) / 5
        hist_smooth = np.convolve(hist, kernel, mode='same')
        peaks = []
        for i in range(1, len(hist_smooth)-1):
            if hist_smooth[i] > 15 and hist_smooth[i] > hist_smooth[i-1] and hist_smooth[i] >= hist_smooth[i+1]:
                peaks.append(i)
        if len(peaks) < 2:
            return 0.0, 0.0, 0.0, 0.2
        left_peak, right_peak = peaks[0], peaks[-1]
        if left_peak > right_peak:
            left_peak, right_peak = right_peak, left_peak
        track_width_px = right_peak - left_peak
        if track_width_px < 10:
            return 0.0, 0.0, 0.0, 0.0
        lane_centre = (left_peak + right_peak) // 2
        offset_px = lane_centre - (w_roi // 2)
        normalized_offset = offset_px / (track_width_px / 2.0)
        normalized_offset = max(-1.0, min(1.0, normalized_offset))
        confidence = min(1.0, np.max(hist) / 200.0)
        return offset_px, normalized_offset, track_width_px, confidence

    # Module B: Corner 
    def detect_corner(self, hsv):
        """
        Detect blue/orange corner lines.
        The driving_direction flag flips the interpretation:
        - If driving_direction ==> 1 (CW): Blue = left turn, Orange = right turn
        - If driving_direction ==> -1 (CCW): Blue = right turn, Orange = left turn
        """
        roi = hsv[int(self.height*0.2):int(self.height*0.5), :]
        blue_area = np.count_nonzero(self._get_mask(roi, 'blue'))
        orange_area = np.count_nonzero(self._get_mask(roi, 'orange'))
        
        # Detect which colour is present
        if blue_area > CORNER_PIXEL_THRESHOLD:
            # Blue detected: Determine turn direction based on driving direction
            if self.driving_direction == 1:
                return True, 'blue'     # CW: Blue = left turn
            else:
                return True, 'orange'   # CCW: Blue = right turn (flipped)
        elif orange_area > CORNER_PIXEL_THRESHOLD:
            if self.driving_direction == 1:
                return True, 'orange'   # CW: Orange = right turn
            else:
                return True, 'blue'     # CCW: Orange = left turn (flipped)
        return False, None

    # Module C: Traffic Signs 
    def detect_traffic_sign(self, hsv):
        """Detect red or green traffic pillars."""
        best_sign = None
        best_x, best_area = -1, 0
        for colour, mask in [('red', self._get_mask(hsv, 'red')), ('green', self._get_mask(hsv, 'green'))]:
            for cnt in self._filter_contours(mask, MIN_PILLAR_AREA):
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / float(h) if h > 0 else 0
                if 0.7 <= aspect <= 2.2 and area > best_area:
                    best_area, best_x, best_sign = area, x + w//2, colour
        if best_sign:
            required_side = 'left' if best_sign == 'green' else 'right'
            return best_sign, best_x, best_area, required_side
        return None, -1, 0, None

    # Module D: Parking 
    def detect_parking(self, hsv):
        mask = self._get_mask(hsv, 'magenta')
        candidates = []
        for cnt in self._filter_contours(mask, MIN_PARKING_AREA):
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = max(w, h) / (min(w, h) + 1e-6)
            if aspect > 2.5:
                candidates.append((x + w//2, y + h//2, cv2.contourArea(cnt)))
        if len(candidates) >= 2:
            candidates.sort(key=lambda t: t[2], reverse=True)
            return [(c[0], c[1]) for c in candidates[:2]]
        return None

    # Main processing 
    def process_frame(self, bgr_frame):
        self.frame_count += 1
        hsv = self._get_hsv(bgr_frame)
        offset_px, norm_offset, track_width, conf = self.detect_lane(hsv)
        corner_detected, corner_colour = self.detect_corner(hsv)
        sign_colour, sign_x, sign_area, required_side = self.detect_traffic_sign(hsv)
        markers = self.detect_parking(hsv)
        return VisionResult(
            lane_offset_px=offset_px,
            lane_offset_normalized=norm_offset,
            track_width_px=track_width,
            lane_confidence=conf,
            corner_detected=corner_detected,
            corner_colour=corner_colour,
            traffic_sign_colour=sign_colour,
            traffic_sign_x=sign_x,
            traffic_sign_area=sign_area,
            obstacle_side_required=required_side,
            parking_markers=markers,
            magenta_detected=markers is not None
        )

    def capture_frame(self):
        self.raw_capture.truncate(0)
        self.camera.capture(self.raw_capture, format='bgr', use_video_port=True)
        return self.raw_capture.array

    def get_frame_and_process(self):
        return self.process_frame(self.capture_frame())


# 3. Serial
def setup_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        print(f"[INFO] Serial connected to {port} at {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {port}: {e}")
        print("Available ports:", glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        sys.exit(1)

def read_uss_data(ser):
    """
    Read ultrasonic sensor data from Arduino/ESP32.
    Expects format: "front_cm,left_cm,right_cm\n"
    Returns: (front_dist, left_dist, right_dist) as floats, or (999, 999, 999) if timeout.
    """
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode().strip()
            parts = line.split(',')
            if len(parts) == 3:
                return float(parts[0]), float(parts[1]), float(parts[2])
        except (ValueError, UnicodeDecodeError):
            pass
    return 999.0, 999.0, 999.0

def send_command(ser, steering, speed, state, colour1=None, colour2=None, phase=None):
    """
    Send a rich command to the Arduino/ESP32.
    Format: "steering,speed,state,colour1,colour2,phase\n"
    - speed = DRIVING SPEED (linear velocity), NOT wheel rotation speed
    - The Pi sends a continuous stream of commands for traffic light passing
    - The Arduino executes; no need to implement traffic light logic on the Arduino
    
    Args:
        steering: float -1.0 to 1.0
        speed: float 0.0 to 1.0 (DRIVING SPEED, not wheel RPM)
        state: string (STRAIGHT, CORNER, OBSTACLE, PARKING, EMERGENCY_STOP)
        colour1: string (blue, orange, red, green, magenta, None)
        colour2: string (secondary colour, for debugging)
        phase: string (APPROACH, ALIGN, PASSING, RECOVER) - for traffic lights
    """
    steering = max(-1.0, min(1.0, steering))
    speed = max(0.0, min(1.0, speed))
    
    colour1_str = colour1 if colour1 else 'None'
    colour2_str = colour2 if colour2 else 'None'
    phase_str = phase if phase else 'None'
    
    command = f"{steering:.3f},{speed:.3f},{state},{colour1_str},{colour2_str},{phase_str}\n"
    ser.write(command.encode())
    
    # Optionally read feedback from Arduino (non-blocking)
    if ser.in_waiting > 0:
        feedback = ser.readline().decode().strip()
        if feedback:
            print(f"[Arduino] {feedback}")


# 4. STATE MACHINE (WITH TRAFFIC LIGHT ALIGNMENT ALGORITHM)
class RobotState:
    """
    Full state machine implementing the Bot States PDF.
    Includes a 3-phase traffic light algorithm that handles wrong-side alignment.
    """
    def __init__(self):
        self.state = "STRAIGHT"
        self.corner_count = 0
        self.laps = 0
        
        # Traffic light state
        self.pillar_phase = "APPROACH"  # APPROACH, ALIGN, PASSING, RECOVER
        self.pillar_side = None         # 'left' or 'right'
        self.pillar_x = 160             # Last known x-position
        self.pillar_counter = 0         # Frames since pillar was seen
        self.pillar_seen_recently = False

    def update(self, vision_result, front_dist, left_dist, right_dist):
        """
        Update state based on vision + ultrasonic data.
        Returns (steering, speed, state_name, colour1, colour2, phase).
        """
        # 1. LEAVING PARKING ZONE
        if (front_dist <= 5.0 and (left_dist <= 5.0 or right_dist <= 5.0) 
            and vision_result.magenta_detected):
            self.state = "LEAVING_PARKING"
            return 0.9, 0.2, "LEAVING_PARKING", "magenta", None, None

    
        # 2. TURNING AROUND CORNERS
        # Tightness of worst-case turns:
        # Full lock (-0.9 to +0.9) at 25% speed.
        # The minimum turning radius depends on the car's mechanical limits.
        if (vision_result.corner_detected and 
            15.0 <= front_dist <= 30.0 and 
            (left_dist >= 30.0 or right_dist >= 30.0)):
            self.state = "CORNER"
            steering = -0.9 if vision_result.corner_colour == 'blue' else 0.9
            self.corner_count += 1
            return steering, 0.25, "CORNER", vision_result.corner_colour, None, None

        
        # 3. TRAFFIC LIGHT PASSING (3-PHASE ALGORITHM)
        # Align on the correct side when approaching from the wrong side
        # Phase 1 (APPROACH) biases toward the correct side; Phase 2 (ALIGN) corrects position.
        # Keep the Arduino simple: receive commands and move motors.
        # The Pi handles all the decision-making.

        # Check if a traffic sign is visible
        if vision_result.traffic_sign_colour is not None:
            # Update pillar tracking
            self.pillar_x = vision_result.traffic_sign_x
            self.pillar_side = 'left' if vision_result.traffic_sign_colour == 'green' else 'right'
            self.pillar_counter = 0
            self.pillar_seen_recently = True
            self.state = "OBSTACLE"
            
            # Phase 1: APPROACH (when it is far from pillar, > 50cm) 
            # Lane-following with bias toward the correct side.
            # This ensures the car starts moving to the right side early.
            if front_dist > 50.0 or front_dist == 999.0:  # 999 means no data
                self.pillar_phase = "APPROACH"
                # Lane-following with slight bias toward correct side
                lane_steer = -vision_result.lane_offset_normalized * 0.6
                
                # Add gentle bias toward the required side
                if self.pillar_side == 'left':
                    bias = -0.15  # Steer left
                else:
                    bias = 0.15   # Steer right
                
                steering = max(-0.8, min(0.8, lane_steer + bias))
                return steering, 0.5, "OBSTACLE", vision_result.traffic_sign_colour, None, "APPROACH"
            
            # Phase 2: ALIGN (when close to pillar, 20-50cm) 
            # If the car is on the wrong side, this phase will steer it to the correct side.
            elif 20.0 < front_dist <= 50.0:
                self.pillar_phase = "ALIGN"
                # Target: position the car so the pillar is on the correct side of the car
                # If pillar_side == 'left': car should be to the RIGHT of the pillar
                #   → pillar should appear at x ≈ 100 (left of centre)
                # If pillar_side == 'right': car should be to the LEFT of the pillar
                #   → pillar should appear at x ≈ 220 (right of centre)
                image_centre = 160
                if self.pillar_side == 'left':
                    target_x = 100  # Pillar should appear left of centre
                else:
                    target_x = 220  # Pillar should appear right of centre
                
                # Calculate error and convert to steering
                error = (self.pillar_x - target_x) / 160.0  # Normalize to -1..1
                steering = max(-0.6, min(0.6, error))
                return steering, 0.3, "OBSTACLE", vision_result.traffic_sign_colour, None, "ALIGN"
            
            # Phase 3: PASSING (when very close, < 20cm)
            # Full commitment to the turn.
            # This is the tightest the turn gets: full lock at 25% speed.
            elif front_dist <= 20.0:
                self.pillar_phase = "PASSING"
                # Full lock toward the correct side
                if self.pillar_side == 'left':
                    steering = -0.8  # Hard left
                else:
                    steering = 0.8   # Hard right
                return steering, 0.25, "OBSTACLE", vision_result.traffic_sign_colour, None, "PASSING"
                
        # 4. RECOVERY (After passing the pillar)
        # If we were in obstacle state but the pillar disappeared, recover.
        if self.state == "OBSTACLE" and self.pillar_seen_recently:
            self.pillar_counter += 1
            if self.pillar_counter > 10:  # Pillar lost for 10+ frames
                self.state = "STRAIGHT"
                self.pillar_phase = "RECOVER"
                self.pillar_seen_recently = False
                # Gradually return to lane-following
                steering = -vision_result.lane_offset_normalized * 0.6
                return steering, 0.5, "STRAIGHT", None, None, "RECOVER"

      
        # 5. STRAIGHT DRIVING 

        if (front_dist >= 20.0 or front_dist == 999.0 and 
            not vision_result.corner_detected and
            vision_result.traffic_sign_colour is None):
            self.state = "STRAIGHT"
            steering = -vision_result.lane_offset_normalized * 0.7
            steering = max(-0.6, min(0.6, steering))
            return steering, 0.5, "STRAIGHT", None, None, None

       
        # 6. PARKING
        
        if (self.laps >= 3 and vision_result.magenta_detected and 
            vision_result.parking_markers is not None):
            self.state = "PARKING"
            avg_x = (vision_result.parking_markers[0][0] + 
                    vision_result.parking_markers[1][0]) / 2
            steering = (avg_x - 160) / 160.0
            steering = max(-0.5, min(0.5, steering))
            return steering, 0.15, "PARKING", "magenta", None, None

     
        # 7. EMERGENCY STOP
       
        if front_dist < 10.0:
            self.state = "EMERGENCY_STOP"
            return 0.0, 0.0, "EMERGENCY_STOP", None, None, None

    
        # 8. FALLBACK
      
        self.state = "SAFE_MODE"
        return 0.0, 0.2, "SAFE_MODE", None, None, None


# 5. MAIN LOOP

def signal_handler(sig, frame):
    print("\n[INFO] Shutting down...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    # Connect to Arduino/ESP32 
    arduino = setup_serial(ARDUINO_PORT, BAUD_RATE)
    
    #  Initialise Vision 
    # driving_direction: 1 = clockwise, -1 = counter-clockwise
    # This is determined by the round setup 
    driving_direction = 1  # Change this based on the round
    vision = BossModeVision(driving_direction=driving_direction)
    
    # Initialise State Machine
    robot = RobotState()
    
    print("[INFO] Control loop started. Press Ctrl+C to stop.")
    print("[INFO] Serial format: steering,speed,state,colour1,colour2,phase")
    print("[INFO] USS data expected from Arduino: front_cm,left_cm,right_cm")
    
    try:
        with vision:
            while True:
                # 1. Read USS data from Arduino
                front_dist, left_dist, right_dist = read_uss_data(arduino)
                
                # 2. Get vision data
                result = vision.get_frame_and_process()
                
                # 3. Update state machine
                steering, speed, state, colour1, colour2, phase = robot.update(
                    result, front_dist, left_dist, right_dist
                )
                
                # 4. Send command to Arduino
                send_command(arduino, steering, speed, state, colour1, colour2, phase)
                
                # 5. Debug output
                print(f"State: {state:15} | Steer: {steering:+5.2f} | "
                      f"F:{front_dist:5.1f}cm L:{left_dist:5.1f}cm R:{right_dist:5.1f}cm | "
                      f"Colour: {colour1 or 'None':6} | Phase: {phase or 'None':8} | "
                      f"Offset: {result.lane_offset_normalized:+6.2f}")
                
                time.sleep(CONTROL_LOOP_DT)
                
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        # Send stop command
        send_command(arduino, 0.0, 0.0, "STOP", None, None, None)
        arduino.close()
        print("[INFO] Serial closed.")


if __name__ == "__main__":
    main()
  
    # Module C: Traffic sign / Obstacle detection
    def detect_traffic_sign(self, hsv):
        """
        Detects Red (Stop/Right) or Green (Go/Left) pillars.
        Returns the required side for the state machine.
        Rulebook: Red must be passed on the RIGHT; Green on the LEFT.
        """
        mask_red = self._get_mask(hsv, 'red')
        mask_green = self._get_mask(hsv, 'green')
        
        best_sign = None
        best_x = -1
        best_area = 0
        
        for colour, mask in [('red', mask_red), ('green', mask_green)]:
            cnts = self._filter_contours(mask, MIN_PILLAR_AREA)
            for cnt in cnts:
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                # 50x50mm pillars appear roughly square (aspect ratio ~1)
                aspect = w / float(h) if h > 0 else 0
                if 0.7 <= aspect <= 2.2:  # Allow perspective distortion
                    if area > best_area:
                        best_area = area
                        best_x = x + w//2
                        best_sign = colour
        
        if best_sign is not None:
            # Rulebook Logic: Green = pass on Left, Red = pass on Right
            required_side = 'left' if best_sign == 'green' else 'right'
            return best_sign, best_x, best_area, required_side
        return None, -1, 0, None

    
    # Module D: Parking marker detection
    def detect_parking(self, hsv):
        """
        Detects the two Magenta bars (200x20mm) defining the parking slot.
        """
        mask_magenta = self._get_mask(hsv, 'magenta')
        cnts = self._filter_contours(mask_magenta, MIN_PARKING_AREA)
        
        candidates = []
        for cnt in cnts:
            x, y, w, h = cv2.boundingRect(cnt)
            # 200x20mm ==> long bar
            aspect = max(w, h) / (min(w, h) + 1e-6)
            if aspect > 2.5:
                cx, cy = x + w//2, y + h//2
                candidates.append((cx, cy, cv2.contourArea(cnt)))
        
        if len(candidates) >= 2:
            # Sort by area and take the two largest bars
            candidates.sort(key=lambda t: t[2], reverse=True)
            return [(c[0], c[1]) for c in candidates[:2]]
        return None

    # Main Processing 

    def process_frame(self, bgr_frame):
        """
        Processes a single BGR frame and returns a fully populated VisionResult.
        """
        self.frame_count += 1
        hsv = self._get_hsv(bgr_frame)

        # 1. Lane Straight Driving
        offset_px, norm_offset, track_width, conf = self.detect_lane(hsv)

        #2.Turning when corner detected 
        corner_detected, corner_colour = self.detect_corner(hsv)

        # 3. Obstacles - Traffic Lights
        sign_colour, sign_x, sign_area, required_side = self.detect_traffic_sign(hsv)

        #4. Parking 
        markers = self.detect_parking(hsv)

        return VisionResult(
            lane_offset_px=offset_px,
            lane_offset_normalized=norm_offset,
            track_width_px=track_width,
            lane_confidence=conf,
            corner_detected=corner_detected,
            corner_colour=corner_colour,
            traffic_sign_colour=sign_colour,
            traffic_sign_x=sign_x,
            traffic_sign_area=sign_area,
            obstacle_side_required=required_side,
            parking_markers=markers,
            magenta_detected=markers is not None
        )

    def capture_frame(self):
        self.raw_capture.truncate(0)
        self.camera.capture(self.raw_capture, format='bgr', use_video_port=True)
        return self.raw_capture.array

    def get_frame_and_process(self):
        return self.process_frame(self.capture_frame())
        
# 3. Serial Helpers 
def etup_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        print(f"[INFO] Serial connected to {port} at {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {port}: {e}")
        print("Available ports:", glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        sys.exit(1)

def send_command(ser, steering, speed):
    steering = max(-1.0, min(1.0, steering))
    speed = max(0.0, min(1.0, speed))
    ser.write(f"{steering:.3f},{speed:.3f}\n".encode())
    if ser.in_waiting > 0:
        fb = ser.readline().decode().strip()
        if fb:
            print(f"[Arduino] {fb}")

# 4. State Machine - based on the robot states in the 'Bot States' document
class RobotState:
    def __init__(self):
        self.state = "STRAIGHT"
        self.corner_count = 0
        self.laps = 0

    def update(self, result):
        #Corner 
        if result.corner_detected:
            self.state = "CORNER"
            turn = -0.8 if result.corner_colour == 'blue' else 0.8
            return turn, 0.3

        #Traffic obstacle 
        if result.traffic_sign_colour is not None:
            self.state = "OBSTACLE"
            steer = -0.5 if result.traffic_sign_colour == 'green' else 0.5
            return steer, 0.4

        #Parking 
        if result.magenta_detected and result.parking_markers:
            self.state = "PARKING"
            avg_x = (result.parking_markers[0][0] + result.parking_markers[1][0]) / 2
            steer = (avg_x - 160) / 160.0
            return max(-0.5, min(0.5, steer)), 0.2

        # Straight (default) 
        self.state = "STRAIGHT"
        steer = -result.lane_offset_normalized * 0.7
        steer = max(-0.6, min(0.6, steer))
        return steer, 0.6


# 5. Main Loop - captures frames, processes vision, updates state, sends serial commands at 20Hz

def signal_handler(sig, frame):
    print("\n[INFO] Shutting down...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    arduino = setup_serial(ARDUINO_PORT, BAUD_RATE)
    vision = BossModeVision()
    robot = RobotState()
    print("[INFO] Control loop started. Press Ctrl+C to stop.")

    try:
        with vision:
            while True:
                result = vision.get_frame_and_process()
                steering, speed = robot.update(result)
                send_command(arduino, steering, speed)

                # Optional debug print
                print(f"State: {robot.state:10} | Steering: {steering:+6.2f} | "
                      f"Offset: {result.lane_offset_normalized:+6.2f} | "
                      f"Sign: {result.traffic_sign_colour or 'None':5}")
                time.sleep(CONTROL_LOOP_DT)

    except KeyboardInterrupt:
        print("[INFO] Stopped by user")
    finally:
        send_command(arduino, 0.0, 0.0)  #Physical stop - sends 0 speed 
        arduino.close() # closes the serial port
        print("[INFO] Serial closed.")

if __name__ == "__main__":
    main()
