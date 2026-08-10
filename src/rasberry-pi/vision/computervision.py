
"""
WRO 2026 FUTURE ENGINEERS - Tamagotchi-Triplets
Hardware: 
- Raspberry Pi 3B+ 
- Camera Module 3 
- 320x240 at 15-20 FPS

Based on Official WRO 2026 Rulebook Specifications:
- Track width: 600mm or 1000mm (dynamically handled)
- Line colours: Orange (CMYK 0,60,100,0), Blue (CMYK 100,80,0,0)
- Traffic signs: Red (RGB 238,39,55), Green (RGB 68,214,44)
- Parking markers: Magenta (RGB 255,0,255)
"""

import cv2
import numpy as np
from collections import namedtuple
from picamera import PiCamera, PiRGBArray
import time
import logging


#  GLOBAL CONSTANTS 

# Official Rulebook Specs (converted to OpenCV HSV ~ H:0-180, S:0-255, V:0-255)
# Note: These are perfect baselines. Tournament lighting may vary (probably will).
# The code includes a dynamic 'S' and 'V' slack adjustment if detections fail.
OFFICIAL_COLOUR_BASELINES = {
    # Blue Line (CMYK 100,80,0,0 -> approx RGB 0,51,204)
    'blue':   (np.array([100, 150, 80]),  np.array([130, 255, 255])),
    # Orange Line (CMYK 0,60,100,0 -> approx RGB 255,153,0)
    'orange': (np.array([5, 120, 120]),   np.array([15, 255, 255])),
    # Red Pillar (RGB 238,39,55) - wraps around 0
    'red1':   (np.array([0, 120, 70]),    np.array([10, 255, 255])),
    'red2':   (np.array([170, 120, 70]),  np.array([180, 255, 255])),
    # Green Pillar (RGB 68,214,44)
    'green':  (np.array([40, 100, 70]),   np.array([80, 255, 255])),
    # Magenta Parking (RGB 255,0,255)
    'magenta':(np.array([140, 100, 70]),  np.array([170, 255, 255])),
}

# Minimum contour areas (50x50mm pillars, 20mm thick lines)
# At 320x240, a pillar at 1m distance is roughly 150-250 pixels.
MIN_LINE_AREA = 20          # Thin 20mm lines
MIN_PILLAR_AREA = 150       # 50x50mm signs
MIN_PARKING_AREA = 80       # 200x20mm bars (long but thin)

# Corner trigger: If blue/orange occupies more than this many pixels in the
# upper half, we are definitely at a corner. (Based on 20mm thick lines).
CORNER_PIXEL_THRESHOLD = 120

#Configuration 
"""
Serial port configuration. 
From my research, on linux/raspberry pi: usually '/dev/ttyACM0' or '/dev/ttyUSB0'
You can find it by running: ls /dev/ttyUSB* /dev/ttyACM*
"""
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200  # Must match the baud rate in our Arduino sketch

# Control loop timing (seconds)
CONTROL_LOOP_DT = 0.05  # 20 Hz control loop

# Serial Communication Setup 

def setup_serial(port, baudrate):
    """ Initialise and return a serial connection to the Arduino."""
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"[INFO] Serial connected to {port} at {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {port}: {e}")
        print("[INFO] Available ports:")
        import glob
        print(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        sys.exit(1)

def send_command(ser, steering_value, speed_value=0.5):
    """
    Send a steering command to the Arduino.
    Format: "steering,speed\n"
    - steering: float between -1.0 (full left) and 1.0 (full right)
    - speed: float between 0.0 (stop) and 1.0 (full speed)
    """
    # Clamp values to safe range
    steering = max(-1.0, min(1.0, steering_value))
    speed = max(0.0, min(1.0, speed_value))
    
    # Build command string
    command = f"{steering:.3f},{speed:.3f}\n"
    ser.write(command.encode())
    
    # Optionally read any feedback from Arduino (non-blocking)
    if ser.in_waiting > 0:
        feedback = ser.readline().decode().strip()
        if feedback:
            print(f"[Arduino] {feedback}")

# 1. Output data structure

# This gives the control loop everything it needs.
# -1.0 = hard left, +1.0 = hard right.
VisionResult = namedtuple('VisionResult', [
    # Module A: Lane Keeping 
    'lane_offset_px',           # Raw pixel error from centre
    'lane_offset_normalized',   # -1.0 to 1.0 (scaled by dynamic track width)
    'track_width_px',           # Estimated pixel width of the lane (for debugging)
    'lane_confidence',          # 0.0 to 1.0 (how sure we are about the lane)
    
    # Module B: Corner Module
    'corner_detected',          # Boolean: True if Blue/Orange corner ahead
    'corner_colour',            # 'blue' or 'orange' (determines turn direction)
    
    # Module C: Traffic Obstacles 
    'traffic_sign_colour',      # 'red' or 'green' or None
    'traffic_sign_x',           # X-coordinate (pixels) of the sign's centre
    'traffic_sign_area',        # Area (larger = closer)
    'obstacle_side_required',   # 'left' or 'right' (Green=Left, Red=Right)
    
    #Parking Module
    'parking_markers',          # List of two (x,y) centroids, or None
    'magenta_detected'          # Bool: True if parking slot is visible
])

# Vision Processor Class
class BossModeVision: 
    """
   Attempted to make this adapt to lighting changes - we'll see if it works when we test it out
    """
    def __init__(self, resolution=(320, 240), framerate=15):
        self.width, self.height = resolution
        self.framerate = framerate
        
        # State variables for dynamic threshold adjustment
        self.slack_s = 0   # Saturation slack
        self.slack_v = 0   # Value slack
        self.frame_count = 0

        # Camera Setup 
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.exposure_mode = 'off'
        self.camera.shutter_speed = 10000  # 1/100 sec to avoid motion blur
        self.camera.iso = 400
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = (1.0, 1.0) # Tune this to your hall lights
        time.sleep(0.5)  # Sensor warm-up
        
        self.raw_capture = PiRGBArray(self.camera, size=resolution)
        logging.info("BossModeVision initialised. Let's win this.")

    def __enter__(self): return self
    def __exit__(self, *args): self.camera.close()

    def _get_hsv(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    def _get_mask(self, hsv, colour_key):
        """Uses the official baselines + dynamic slack for robustness."""

        def _apply_slack(low, high):
            low = low.copy()
            high = high.copy()
            low[1] = max(0, int(low[1]) - self.slack_s)
            low[2] = max(0, int(low[2]) - self.slack_v)
            high[1] = min(255, int(high[1]) + self.slack_s)
            high[2] = min(255, int(high[2]) + self.slack_v)
            return low, high

        if colour_key == 'red':
            l1, h1 = _apply_slack(*OFFICIAL_COLOUR_BASELINES['red1'])
            l2, h2 = _apply_slack(*OFFICIAL_COLOUR_BASELINES['red2'])
            m1 = cv2.inRange(hsv, l1, h1)
            m2 = cv2.inRange(hsv, l2, h2)
            return cv2.bitwise_or(m1, m2)

        low, high = _apply_slack(*OFFICIAL_COLOUR_BASELINES[colour_key])
        return cv2.inRange(hsv, low, high)

    def _filter_contours(self, mask, min_area):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [c for c in cnts if cv2.contourArea(c) >= min_area]

    
    # Module A: Lane keeping
    def _find_dynamic_roi(self, hsv):
        """
        Cool feature (Inspired by my poor eyesight): Finds the vanishing point by detecting where
        the lane lines converge. This crops out the useless background.
        """
        # Simple heuristic: find the highest y-coordinate where blue/orange exists.
        mask = cv2.bitwise_or(self._get_mask(hsv, 'blue'), self._get_mask(hsv, 'orange'))
        # Project horizontally to find rows with lines
        row_sums = np.sum(mask, axis=1)
        non_zero_rows = np.where(row_sums > 20)[0]
        if len(non_zero_rows) > 10:
            horizon = np.min(non_zero_rows)
        else:
            horizon = int(self.height * 0.4)  # Fallback to 40% from top
        return max(0, horizon - 10)  # A bit of margin

    def detect_lane(self, hsv):
        """Note: Because I was unsure about the track width, I've attempted to normalise this 
        so that PID can be universal 
        If the track is 600mm, the pixel distance between lines is small.
        If the track is 1000mm, it's large. 
        """
        # Dynamically crop the image to ignore the far away points in background
        horizon = self._find_dynamic_roi(hsv)
        roi = hsv[horizon:, :]
        h_roi, w_roi = roi.shape[:2]

        mask_blue = self._get_mask(roi, 'blue')
        mask_orange = self._get_mask(roi, 'orange')
        mask_lane = cv2.bitwise_or(mask_blue, mask_orange)

        # Histogram of the bottom 2/3rds of the ROI  (Region Of Interest - or you can call it area of interest) (closest to the car)
        bottom_roi = mask_lane[int(h_roi*0.33):, :]
        hist = np.sum(bottom_roi, axis=0, dtype=np.int32)
        
        if np.max(hist) < 15:
            # No lines visible  ==> increment slack for next frame 
            self.slack_s = min(50, self.slack_s + 5)
            self.slack_v = min(50, self.slack_v + 5)
            return 0.0, 0.0, 0.0, 0.0

        # Reset slack if we see lines
        self.slack_s = max(0, self.slack_s - 2)
        self.slack_v = max(0, self.slack_v - 2)

        # Smooth histogram to find peaks
        kernel = np.ones(5, dtype=np.float32) / 5
        hist_smooth = np.convolve(hist, kernel, mode='same')
        
        # Find peaks
        peaks = []
        for i in range(1, len(hist_smooth)-1):
            if hist_smooth[i] > 15 and hist_smooth[i] > hist_smooth[i-1] and hist_smooth[i] >= hist_smooth[i+1]:
                peaks.append(i)
        
        if len(peaks) < 2:
            return 0.0, 0.0, 0.0, 0.2  # Low confidence

        # Get the leftmost and rightmost significant peaks
        left_peak = peaks[0]
        right_peak = peaks[-1]
        
        # Ensure left is actually left
        if left_peak > right_peak:
            left_peak, right_peak = right_peak, left_peak
            
        # Calculation: Track Width and Normalised Offset
        track_width_px = right_peak - left_peak
        if track_width_px < 10:
            return 0.0, 0.0, 0.0, 0.0
            
        lane_centre = (left_peak + right_peak) // 2
        image_centre = w_roi // 2
        offset_px = lane_centre - image_centre
        
        # Normalise: -1.0 = touching left line , 0.0 = centre, +1.0 = touching right line)
        normalized_offset = offset_px / (track_width_px / 2.0)
        # Clamp to -1 - 1
        normalized_offset = max(-1.0, min(1.0, normalized_offset))
        
        confidence = min(1.0, np.max(hist) / 200.0)  # Heuristic confidence
        
        return offset_px, normalized_offset, track_width_px, confidence

    # Module B: Corner detection (Triggers the state machine)
   
    def detect_corner(self, hsv):
        """
        Looks for the thick blue/orange lines that signal a 90-degree turn.
        We check the upper-middle region where the corner line spans the track.
        """
        # Corner lines are usually visible in the middle-to-upper area.
        roi = hsv[int(self.height*0.2):int(self.height*0.5), :]
        mask_blue = self._get_mask(roi, 'blue')
        mask_orange = self._get_mask(roi, 'orange')
        
        # For a corner, the line is thick and spans horizontally.
        # We check the total area of the mask.
        blue_area = np.count_nonzero(mask_blue)
        orange_area = np.count_nonzero(mask_orange)
        
        if blue_area > CORNER_PIXEL_THRESHOLD:
            return True, 'blue'
        elif orange_area > CORNER_PIXEL_THRESHOLD:
            return True, 'orange'
        return False, None

  
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
