#!/usr/bin/env python3
"""
WRO 2026 FUTURE ENGINEERS - Tamagotchi-Triplets
Complete autonomous driving pipeline: Vision + State Machine + Serial (Pi ↔ Arduino)

HARDWARE ARCHITECTURE:
- Raspberry Pi 3B+:  Computer Vision (OpenCV), State Machine, PID Control
- Arduino/ESP32:     Motor control, Ultrasonic Sensors (USS), Emergency Stop

SERIAL PROTOCOL (Pi → Arduino):
    Format: "steering,speed,state,colour1,colour2,phase\n"
    Example: "0.350,0.500,STRAIGHT,None,None,None\n"

    Fields:
    - steering:  float -1.0 (full left) to +1.0 (full right)
    - speed:     float 0.0 (stop) to 1.0 (full speed)
    - state:     string (STRAIGHT, CORNER, OBSTACLE, PARKING, EMERGENCY_STOP)
    - colour1:   string (blue, orange, red, green, magenta, None)
    - colour2:   string (secondary colour, for debugging)
    - phase:     string (APPROACH, ALIGN, PASSING, RECOVER)

SERIAL PROTOCOL (Arduino → Pi):
    Format: "front_cm,left_cm,right_cm\n"
    Example: "25.3,12.1,8.7\n"
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
import threading
import queue
from collections import deque


# Global Constants


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

# Serial configuration
ARDUINO_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
CONTROL_LOOP_DT = 0.05  # 20 Hz

# PID tuning parameters 
PID_KP = 1.0      # Proportional gain – start here
PID_KI = 0.03     # Integral gain – fixes drift
PID_KD = 0.15     # Derivative gain – smooths out oscillations
PID_RATE_LIMIT = 2.5  # Max steering change per second
PID_INTEGRAL_LIMIT = 1.0  # Anti-windup clamp

# USS timeout value
USS_TIMEOUT = 999.0


# 1. Output Data Structure

VisionResult = namedtuple('VisionResult', [
    'lane_offset_px', 'lane_offset_normalized', 'track_width_px', 'lane_confidence',
    'corner_detected', 'corner_colour',
    'traffic_sign_colour', 'traffic_sign_x', 'traffic_sign_area', 'obstacle_side_required',
    'parking_markers', 'magenta_detected'
])

# 2. PID Controller

class PIDController:
    """
    PID controller with anti-windup,
    derivative-on-measurement, and output rate limiting..
    
    Features:
    - Derivative on measurement (no derivative kick)
    - Integral anti-windup with dynamic clamping
    - Low-pass filter on derivative (kills camera noise)
    - Output rate limiting ( human-like steering)
    - Conditional integration (only corrects when the error is small)
    """
    def __init__(self, kp, ki, kd, dt=0.05,
                 output_min=-1.0, output_max=1.0,
                 rate_limit=2.0,
                 derivative_filter_alpha=0.5,
                 integral_limit=1.0):
        """
        Args:
            kp: Proportional gain (steering response strength)
            ki: Integral gain (steady-state error correction)
            kd: Derivative gain (damping/anti-oscillation)
            dt: Update rate (seconds)
            output_min: Minimum output value (usually -1.0) - we can change upon testingI
            output_max: Maximum output value (usually 1.0) - we can change upon testing
            rate_limit: Max output change per second (prevents snapping)
            derivative_filter_alpha: Low-pass filter for derivative (0-1, higher = more smoothing)
            integral_limit: Max absolute integral sum (anti-windup)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max
        self.rate_limit = rate_limit
        self.derivative_filter_alpha = derivative_filter_alpha
        self.integral_limit = integral_limit

        # State variables
        self.integral = 0.0
        self.prev_measurement = 0.0
        self.prev_output = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True

        # Feedforward (optional)
        self.feedforward_gain = 0.0

        # Performance tracking
        self.last_output = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

    def update(self, setpoint, measurement, feedforward=0.0):
        """
        Calculate PID output.
        
        Args:
            setpoint: Desired value (e.g., 0.0 for lane centre)
            measurement: Actual value (e.g., lane_offset_normalized)
            feedforward: Additional steering bias (for corners, etc.)
        
        Returns:
            PID output clamped to [-1.0, 1.0]
        """
        dt = max(self.dt, 0.001)  # Prevent divide-by-zero

        # 1. Calculate error 
        error = setpoint - measurement

        # 2. Proportional term (immediate response) 
        self.p_term = self.kp * error

        # 3. Integral term (steady-state correction) 
        # Only integrate if error is small to prevent windup during large turns
        # and if the output isn't saturated
        if abs(error) < 0.5:  # Only correct within 50% of lane
            self.integral += error * dt
            # Clamp integral to prevent windup
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        else:
            # Large error -> don't accumulate integral (avoid overshoot)
            self.integral = 0.0

        self.i_term = self.ki * self.integral

        # 4. Derivative term 
        # Use derivative-on-measurement to prevent derivative kick on setpoint changes
        if self.first_run:
            self.prev_measurement = measurement
            self.first_run = False
            derivative_raw = 0.0
        else:
            derivative_raw = -(measurement - self.prev_measurement) / dt

        self.prev_measurement = measurement

        # Low-pass filter to reduce noise
        self.filtered_derivative = (
            (1.0 - self.derivative_filter_alpha) * derivative_raw +
            self.derivative_filter_alpha * self.filtered_derivative
        )
        self.d_term = self.kd * self.filtered_derivative

        # 5. Combine terms 
        raw_output = self.p_term + self.i_term + self.d_term + (self.feedforward_gain * feedforward)

        # 6. Clamp output 
        clamped_output = max(self.output_min, min(self.output_max, raw_output))

        #7. Anti-windup (back-calculation)
        # If output saturates, reduce integral to prevent windup
        if raw_output != clamped_output:
            if abs(self.ki) > 1e-10:
                self.integral = max(-self.integral_limit, min(self.integral_limit,
                                                             self.integral - (raw_output - clamped_output) / self.ki))
            else:
                self.integral = 0.0

        # 8. Rate limiting (smooth steering)
        max_change = self.rate_limit * dt
        final_output = max(self.prev_output - max_change,
                           min(self.prev_output + max_change, clamped_output))
        self.prev_output = final_output
        self.last_output = final_output

        return final_output

    def reset(self):
        """Reset PID state (useful after emergency stops)."""
        self.integral = 0.0
        self.prev_measurement = 0.0
        self.prev_output = 0.0
        self.filtered_derivative = 0.0
        self.first_run = True

    def get_terms(self):
        """Return individual PID terms for debugging."""
        return self.p_term, self.i_term, self.d_term



# 3. Vision Processor Class

class BossModeVision:
    """Adaptive vision pipeline with dynamic thresholding."""
    def __init__(self, resolution=(320, 240), framerate=15):
        self.width, self.height = resolution
        self.framerate = framerate
        self.slack_s = 0
        self.slack_v = 0
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.frame_drop_count = 0

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
        logging.info("Vision initialised.")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.camera.close()

    def _get_hsv(self, bgr):
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    def _get_mask(self, hsv, colour_key):
        """Uses the official baselines + dynamic slack."""
        def _apply_slack(low, high):
            low = low.copy()
            high = high.copy()
            # Exponential decay instead of linear reduction
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

    def _find_dynamic_roi(self, hsv):
        """Find horizon/vanishing point where lane lines converge."""
        mask = cv2.bitwise_or(self._get_mask(hsv, 'blue'), self._get_mask(hsv, 'orange'))
        row_sums = np.sum(mask, axis=1)
        non_zero_rows = np.where(row_sums > 20)[0]
        if len(non_zero_rows) > 10:
            horizon = np.min(non_zero_rows)
        else:
            horizon = int(self.height * 0.4)
        return max(0, horizon - 10)

    def detect_lane(self, hsv):
        """Dynamic lane detection with normalised offset."""
        horizon = self._find_dynamic_roi(hsv)
        roi = hsv[horizon:, :]
        h_roi, w_roi = roi.shape[:2]

        mask_lane = cv2.bitwise_or(self._get_mask(roi, 'blue'), self._get_mask(roi, 'orange'))
        bottom_roi = mask_lane[int(h_roi * 0.33):, :]
        hist = np.sum(bottom_roi, axis=0, dtype=np.int32)

        if np.max(hist) < 15:
            # Exponential slack decay instead of linear
            self.slack_s = min(50, self.slack_s + 3)
            self.slack_v = min(50, self.slack_v + 3)
            return 0.0, 0.0, 0.0, 0.0

        # Exponential decay when lines are visible
        self.slack_s = max(0, int(self.slack_s * 0.9))
        self.slack_v = max(0, int(self.slack_v * 0.9))

        # Smooth histogram
        kernel = np.ones(5, dtype=np.float32) / 5
        hist_smooth = np.convolve(hist, kernel, mode='same')

        # Find peaks using NumPy
        peaks = []
        for i in range(1, len(hist_smooth) - 1):
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

    def detect_corner(self, hsv, driving_direction=1):
        """
        Detect corner lines with direction-aware interpretation.
        driving_direction: 1 = CW, -1 = CCW
        """
        roi = hsv[int(self.height * 0.2):int(self.height * 0.5), :]
        blue_area = np.count_nonzero(self._get_mask(roi, 'blue'))
        orange_area = np.count_nonzero(self._get_mask(roi, 'orange'))

        if blue_area > CORNER_PIXEL_THRESHOLD:
            if driving_direction == 1:
                return True, 'blue'     # CW: Blue = left turn
            else:
                return True, 'orange'   # CCW: Blue = right turn
        elif orange_area > CORNER_PIXEL_THRESHOLD:
            if driving_direction == 1:
                return True, 'orange'   # CW: Orange = right turn
            else:
                return True, 'blue'     # CCW: Orange = left turn
        return False, None

    def detect_traffic_sign(self, hsv):
        """Detect red or green traffic pillars."""
        best_sign = None
        best_x, best_area = -1, 0

        for colour, mask in [('red', self._get_mask(hsv, 'red')),
                             ('green', self._get_mask(hsv, 'green'))]:
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

    def detect_parking(self, hsv):
        """Detect magenta parking bars."""
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

    def process_frame(self, bgr_frame):
        """Full frame processing pipeline."""
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
        """Capture a single frame from the camera."""
        self.raw_capture.truncate(0)
        self.camera.capture(self.raw_capture, format='bgr', use_video_port=True)
        return self.raw_capture.array

    def get_frame_and_process(self):
        """Convenience: capture and process in one call."""
        return self.process_frame(self.capture_frame())



# 4. Serial Helpers

class SerialDataHandler:
    """Thread-safe serial I/O with ring buffer for USS data."""
    def __init__(self, port, baudrate, max_queue_size=10):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.uss_queue = queue.Queue(maxsize=max_queue_size)
        self.lock = threading.Lock()
        self.connected = False

    def connect(self):
        #Establishes serial connection.
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)
            self.connected = True
            print(f"[INFO] Serial connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {self.port}: {e}")
            return False

    def start_reader(self):
        """Start background thread for reading USS data."""
        if not self.connected:
            return False
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        return True

    def _reader_loop(self):
        #Background thread: read USS data non-blocking.
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode().strip()
                    parts = line.split(',')
                    if len(parts) == 3:
                        try:
                            data = (float(parts[0]), float(parts[1]), float(parts[2]))
                            # Non-blocking queue 
                            if not self.uss_queue.full():
                                self.uss_queue.put(data, block=False)
                            else:
                                # Queue full → discard oldest and add new
                                try:
                                    self.uss_queue.get(block=False)
                                    self.uss_queue.put(data, block=False)
                                except queue.Empty:
                                    pass
                        except ValueError:
                            pass
            except Exception as e:
                print(f"[WARN] Serial read error: {e}")
            time.sleep(0.01)  # Small delay to prevent CPU hog

    def get_uss_data(self):
        # Get the latest USS data from the queue (non-blocking).
        latest = None
        try:
            while True:
                latest = self.uss_queue.get(block=False)
        except queue.Empty:
            pass
        return latest if latest is not None else (USS_TIMEOUT, USS_TIMEOUT, USS_TIMEOUT)

    def send_command(self, steering, speed, state, colour1=None, colour2=None, phase=None):
        #Send command to Arduino (thread-safe).
        if not self.connected or not self.ser:
            return False

        steering = max(-1.0, min(1.0, steering))
        speed = max(0.0, min(1.0, speed))

        colour1_str = colour1 if colour1 else 'None'
        colour2_str = colour2 if colour2 else 'None'
        phase_str = phase if phase else 'None'

        command = f"{steering:.3f},{speed:.3f},{state},{colour1_str},{colour2_str},{phase_str}\n"

        with self.lock:
            try:
                self.ser.write(command.encode())
                return True
            except Exception as e:
                print(f"[ERROR] Serial write failed: {e}")
                return False

    def stop(self):
        """Stop the background thread and close serial."""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()
        self.connected = False


def setup_serial(port, baudrate):
    """Convenience: set up serial with background reader."""
    handler = SerialDataHandler(port, baudrate)
    if handler.connect():
        if handler.start_reader():
            return handler
    return None



# 5. State machine with integrated PID 


class RobotState:
    """
    Full state machine with traffic light alignment and PID control.
    The PID controller is fully integrated into the STRAIGHT state.
    """
    def __init__(self):
        self.state = "STARTUP"
        self.corner_count = 0
        self.laps = 0
        self.max_laps = 3
        self.driving_direction = 1  # 1 = CW, -1 = CCW
        self.direction_determined = False
        self.start_section_corners = 0
        self.sections_passed = 0
        self.finish_detected = False

        # Traffic light state
        self.pillar_phase = "APPROACH"
        self.pillar_side = None
        self.pillar_x = 160
        self.pillar_timestamp = 0
        self.pillar_seen_recently = False
        self.last_pillar_time = time.time()

        # Challenge detection
        self.started_in_parking = False
        self.challenge_type = "UNKNOWN"

      
        # PID controller
        self.steering_pid = PIDController(
            kp=PID_KP,
            ki=PID_KI,
            kd=PID_KD,
            dt=CONTROL_LOOP_DT,
            output_min=-1.0,
            output_max=1.0,
            rate_limit=PID_RATE_LIMIT,
            derivative_filter_alpha=0.6,
            integral_limit=PID_INTEGRAL_LIMIT
        )

        # Steering feedforward for corners
        self.feedforward = 0.0

    def determine_challenge_type(self, vision_result):
        """
        Determine if we're in Open or Obstacle challenge based on magenta detection.
        Called during startup.
        """
        if vision_result.magenta_detected:
            self.challenge_type = "OBSTACLE"
            self.started_in_parking = True
            print("[INFO] Obstacle Challenge detected - starting from parking lot")
            return "OBSTACLE"
        else:
            self.challenge_type = "OPEN"
            self.started_in_parking = False
            print("[INFO] Open Challenge detected - starting from zone")
            return "OPEN"

    def detect_driving_direction(self, vision_result):
        """
        Determine driving direction based on first corner line detected.
        If blue appears first → CW, orange first → CCW.
        """
        if vision_result.corner_detected:
            if vision_result.corner_colour == 'blue':
                self.driving_direction = 1  # CW
            else:
                self.driving_direction = -1  # CCW
            self.direction_determined = True
            print(f"[INFO] Direction determined: {'CW' if self.driving_direction == 1 else 'CCW'}")
            return True
        return False

    def update(self, vision_result, front_dist, left_dist, right_dist):
        """
        Update state based on vision + USS.
        Returns (steering, speed, state_name, colour1, colour2, phase).
        """
        current_time = time.time()

        
        # STATE 0: Startup
        if self.state == "STARTUP":
            self.determine_challenge_type(vision_result)

            if self.detect_driving_direction(vision_result):
                self.state = "STRAIGHT"
                return 0.0, 0.0, "STARTUP", None, None, None

            return 0.0, 0.05, "STARTUP", None, None, None

        
        # STATE 1: Leaving parking zone 
        if (self.challenge_type == "OBSTACLE" and
            self.started_in_parking and
            front_dist <= 5.0 and
            (left_dist <= 5.0 or right_dist <= 5.0) and
            vision_result.magenta_detected):
            self.state = "LEAVING_PARKING"
            # Reset PID when leaving parking
            self.steering_pid.reset()
            return 0.0, 0.0, "LEAVING_PARKING", "magenta", None, "EXIT"

        
        # STATE 2: Turning around corners
        if (vision_result.corner_detected and
            15.0 <= front_dist <= 30.0 and
            (left_dist >= 30.0 or right_dist >= 30.0)):
            self.state = "CORNER"
            steering = -0.9 if vision_result.corner_colour == 'blue' else 0.9

            if self.driving_direction == -1:
                steering = -steering

            self.corner_count += 1
            self.sections_passed += 1

            if self.corner_count % 4 == 0:
                self.laps += 1
                print(f"[INFO] Lap {self.laps} completed!")

            if self.laps >= self.max_laps:
                self.state = "FINISHING"
                # Reset PID for finishing manoeuvre
                self.steering_pid.reset()
                return 0.0, 0.1, "FINISHING", vision_result.corner_colour, None, None

            # Reset PID after corner (prevents windup)
            self.steering_pid.reset()
            return steering, 0.25, "CORNER", vision_result.corner_colour, None, None

        
        # STATE 3: Traffic light passing 
        
        if vision_result.traffic_sign_colour is not None:
            self.pillar_x = vision_result.traffic_sign_x
            self.pillar_side = 'left' if vision_result.traffic_sign_colour == 'green' else 'right'
            self.pillar_timestamp = current_time
            self.pillar_seen_recently = True
            self.last_pillar_time = current_time
            self.state = "OBSTACLE"

            # Phase 1: Approach (> 50cm)
            if front_dist > 50.0 or front_dist == USS_TIMEOUT:
                self.pillar_phase = "APPROACH"
                lane_steer = -vision_result.lane_offset_normalized * 0.6

                if self.pillar_side == 'left':
                    bias = -0.15
                else:
                    bias = 0.15

                steering = max(-0.8, min(0.8, lane_steer + bias))
                return steering, 0.5, "OBSTACLE", vision_result.traffic_sign_colour, None, "APPROACH"

            # Phase 2: Align (20-50cm)
            elif 20.0 < front_dist <= 50.0:
                self.pillar_phase = "ALIGN"
                if self.pillar_side == 'left':
                    target_x = 100
                else:
                    target_x = 220

                error = (self.pillar_x - target_x) / 160.0
                steering = max(-0.6, min(0.6, error))
                return steering, 0.3, "OBSTACLE", vision_result.traffic_sign_colour, None, "ALIGN"

            # Phase 3: Passing (< 20cm)
            elif front_dist <= 20.0:
                self.pillar_phase = "PASSING"
                if self.pillar_side == 'left':
                    steering = -0.8
                else:
                    steering = 0.8
                return steering, 0.25, "OBSTACLE", vision_result.traffic_sign_colour, None, "PASSING"

       
        # STATE 4: Recovery (After passing pillar)
      
        if self.state == "OBSTACLE" and self.pillar_seen_recently:
            if current_time - self.last_pillar_time > 1.0:
                self.state = "STRAIGHT"
                self.pillar_phase = "RECOVER"
                self.pillar_seen_recently = False
                # Reset PID for smooth recovery
                self.steering_pid.reset()
                steering = -vision_result.lane_offset_normalized * 0.6
                return steering, 0.5, "STRAIGHT", None, None, "RECOVER"

        
        # STATE 5: Straight (PID-Controlled)
       
        if (front_dist >= 20.0 and
            not vision_result.corner_detected and
            vision_result.traffic_sign_colour is None):
            self.state = "STRAIGHT"

            # PID target: lane centre
            setpoint = 0.0
            measurement = vision_result.lane_offset_normalized

            # Calculate PID steering output
            pid_steering = self.steering_pid.update(setpoint, measurement, self.feedforward)

            # Apply direction-aware steering
            steering = pid_steering * 0.7
            steering = max(-0.6, min(0.6, steering))

            # Flip if driving counter-clockwise
            if self.driving_direction == -1:
                steering = -steering

            # Get PID terms for debugging
            p_term, i_term, d_term = self.steering_pid.get_terms()

            return steering, 0.5, "STRAIGHT", None, None, None

        
        # STATE 6: Parking
        
        if (self.laps >= self.max_laps and
            vision_result.magenta_detected and
            vision_result.parking_markers is not None):
            self.state = "PARKING"
            avg_x = (vision_result.parking_markers[0][0] +
                     vision_result.parking_markers[1][0]) / 2
            steering = (avg_x - 160) / 160.0
            steering = max(-0.5, min(0.5, steering))
            # Reset PID for parking
            self.steering_pid.reset()
            return steering, 0.15, "PARKING", "magenta", None, "ALIGN"

        
        # STATE 7: Finishing
        if self.state == "FINISHING":
            if self.sections_passed >= 32:
                self.state = "FINISHED"
                self.steering_pid.reset()
                return 0.0, 0.0, "FINISHED", None, None, None
            return 0.0, 0.1, "FINISHING", None, None, None

       
        # STATE 8: Finished
        if self.state == "FINISHED":
            return 0.0, 0.0, "FINISHED", None, None, None

        
        # STATE 9: Emergency stop
        if front_dist < 10.0:
            self.state = "EMERGENCY_STOP"
            self.steering_pid.reset()
            return 0.0, 0.0, "EMERGENCY_STOP", None, None, None

        # STATE 10: Safe Fallback=
        self.state = "SAFE_MODE"
        return 0.0, 0.2, "SAFE_MODE", None, None, None

# 6. MAIN LOOP

def signal_handler(sig, frame):
    print("\n[INFO] Shutting down...")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)

    # Setup serial with background reader
    serial_handler = setup_serial(ARDUINO_PORT, BAUD_RATE)
    if serial_handler is None:
        print("[ERROR] Failed to connect to Arduino. Exiting.")
        sys.exit(1)

    # Initialise vision
    vision = BossModeVision()

    # Initialise state machine (PID is integrated inside)
    robot = RobotState()

    print("[INFO] Control loop started. Press Ctrl+C to stop.")
    print("[INFO] Waiting for first corner to determine direction...")
    print(f"[INFO] PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, RateLimit={PID_RATE_LIMIT}")

    # Timing for frame rate regulation
    frame_times = deque(maxlen=30)

    try:
        with vision:
            while True:
                frame_start = time.time()

                # 1. Get vision data
                result = vision.get_frame_and_process()

                # 2. Get USS data from serial queue
                front_dist, left_dist, right_dist = serial_handler.get_uss_data()

                # 3. Update state machine (PID is used inside STRAIGHT state)
                steering, speed, state, colour1, colour2, phase = robot.update(
                    result, front_dist, left_dist, right_dist
                )

                # 4. Send command to Arduino
                if state != "STARTUP":
                    serial_handler.send_command(steering, speed, state, colour1, colour2, phase)

                # 5. Debug output
                print(f"State: {state:15} | Steer: {steering:+5.2f} | "
                      f"F:{front_dist:5.1f}cm L:{left_dist:5.1f}cm R:{right_dist:5.1f}cm | "
                      f"Colour: {colour1 or 'None':6} | Phase: {phase or 'None':8} | "
                      f"Offset: {result.lane_offset_normalized:+6.2f} | "
                      f"Laps: {robot.laps} | Dir: {'CW' if robot.driving_direction == 1 else 'CCW'}")

                # 6. Frame rate regulation
                frame_time = time.time() - frame_start
                frame_times.append(frame_time)
                avg_fps = 1.0 / (sum(frame_times) / len(frame_times)) if frame_times else 0

                sleep_time = max(0, CONTROL_LOOP_DT - frame_time)
                time.sleep(sleep_time)

                if frame_time > CONTROL_LOOP_DT * 1.5:
                    print(f"[WARN] Frame took {frame_time*1000:.1f}ms (target: {CONTROL_LOOP_DT*1000:.0f}ms)")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")
    finally:
        if serial_handler:
            serial_handler.send_command(0.0, 0.0, "STOP", None, None, None)
            serial_handler.stop()
        print("[INFO] Serial closed.")


if __name__ == "__main__":
    main()
