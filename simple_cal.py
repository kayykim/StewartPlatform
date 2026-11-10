# Simple Auto Calibration System for Ball and Beam Control
# Interactive calibration tool for color detection, geometry, and servo limits
# Generates config.json file for use with ball tracking controller

import cv2
import numpy as np
import json
import math
import serial
import time
from datetime import datetime

class SimpleAutoCalibrator:
    """Interactive calibration system for stewart control setup."""
    
    def __init__(self):
        """Initialize calibration parameters and default values."""

        # ----- unchanged calib script --------
        # Physical system parameters
        self.SIDE_LENGTH_M = math.sqrt(3)/2 * 0.3  # stewart platform plate - 300mm diameter
        
        # Camera configuration
        self.CAM_INDEX = 0  # Default camera index
        self.FRAME_W, self.FRAME_H = 640, 480  # Frame dimensions
        
        # Calibration state tracking
        self.current_frame = None  # Current video frame
        self.phase = "color"  # Current phase: "color", "geometry", "limits", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection

        # Servo hardware configuration
        self.servo = None  # Serial connection to servo
        self.servo_port = "/dev/cu.usbmodem1401"  # Servo communication port


        # ----- changed calib script (for Stewart Platform) --------
        # Geometry calibration data - change to 3 point on plate 
        self.peg_points = []  # endpoint pixel coordinates [(x1,y1), (x2,y2), (x3,y3)]
        self.plate_center = None # centerpoint (cx, cy)
        self.pixel_to_meter_ratio = None # average (np.mean)
        self.pixel_to_meter_ratio_l1 = None  # Conversion ratio from pixels to meters
        self.pixel_to_meter_ratio_l2 = None 
        self.pixel_to_meter_ratio_l3 = None 

        # angles - safety margins and NA 
        self.SERVOMIN = 102
        self.SERVOMAX = 512

        self.neutral_tick = [self.SERVOMAX-110, self.SERVOMAX-80, self.SERVOMAX-80]  # tested on servo_controller.ino
        self.min_tick = [self.SERVOMAX-180, self.SERVOMAX-150, self.SERVOMAX-150]
        self.max_tick = [self.SERVOMAX-40, self.SERVOMAX-10, self.SERVOMAX-10]

        # Position limit results
        #self.position_min = [None, None]  # Minimum ball position in meters for each motor [xmin, ymin]
        #self.position_max = [None, None]  # Maximum ball position in meters for each motor [xmax, ymax]

    def connect_servo(self):
        """Establish serial connection to servo motor for automated limit finding.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.servo = serial.Serial(self.servo_port, 9600)
            time.sleep(2)  # Allow time for connection to stabilize
            print("[SERVO] Connected")
            return True
        except:
            print("[SERVO] Failed to connect - limits will be estimated")
            return False

    def send_servo_angle(self, channel, angle):
        """Send angle command to servo motor with safety clipping.
        
        Args:
            angle (float): Desired servo angle in degrees
        """
        if self.servo:
            tick = int ((angle/180)*(self.SERVOMAX - self.SERVOMIN) + self.SERVOMIN)
            
            # validate tick range
            if(
                0 <= channel < 3 and
                self.min_tick[channel] <= tick <= self.max_tick[channel]
            ):
                # send high bytes, low bytes, channel 
                self.servo.write(tick.to_bytes(2, byteorder="big"))
                self.servo.write(bytes([channel]))
                print(f"Sent Tick: {tick}")

            else:
                print("Invalid tick")
                
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events for interactive calibration.
        
        Args:
            event: OpenCV mouse event type
            x, y: Mouse click coordinates
            flags: Additional event flags
            param: User data (unused)
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "color":
                # Color sampling phase - collect HSV samples at click point
                self.sample_color(x, y)
            elif self.phase == "geometry" and len(self.peg_points) < 3: # changed to adapt to 3 points
                # Geometry phase - collect beam endpoint coordinates
                self.peg_points.append((x, y))
                print(f"[GEO] Peg {len(self.peg_points)} selected")
                if len(self.peg_points) == 3:
                    self.calculate_geometry()

    def sample_color(self, x, y):
        """Sample HSV color values in a 5x5 region around click point.
        
        Args:
            x, y: Center coordinates for color sampling
        """
        if self.current_frame is None:
            return
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        
        # Sample 5x5 region around click point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x + dx, y + dy
                # Check bounds and collect valid samples
                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])
        
        # Update HSV bounds based on collected samples
        if self.hsv_samples:
            samples = np.array(self.hsv_samples)
            
            # Calculate adaptive margins for each HSV channel
            h_margin = max(5, (np.max(samples[:, 0]) - np.min(samples[:, 0])) * 0.1)
            s_margin = max(10, (np.max(samples[:, 1]) - np.min(samples[:, 1])) * 0.15)
            v_margin = max(10, (np.max(samples[:, 2]) - np.min(samples[:, 2])) * 0.15)
            
            # Set lower bounds with margin
            self.lower_hsv = [
                max(0, np.min(samples[:, 0]) - h_margin),
                max(0, np.min(samples[:, 1]) - s_margin),
                max(0, np.min(samples[:, 2]) - v_margin)
            ]
            
            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples[:, 0]) + h_margin),
                min(255, np.max(samples[:, 1]) + s_margin),
                min(255, np.max(samples[:, 2]) + v_margin)
            ]
            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")

    def calculate_geometry(self):
        """Calculate pixel-to-meter conversion ratio from platform endpoint coordinates."""
        p1, p2, p3 = self.peg_points
        
        # Calculate pixel distance between beam endpoints
        #pixel_distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) - from BB
        l3 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        l2 = math.sqrt((p3[0] - p1[0])**2 + (p3[1] - p1[1])**2)
        l1 = math.sqrt((p3[0] - p2[0])**2 + (p3[1] - p2[1])**2)

        cx = (p1[0] + p2[0] + p3[0])/3
        cy = (p1[1] + p2[1] + p3[1])/3

        self.plate_center = (int(cx), int(cy))
        # Convert to meters using known beam length
        self.pixel_to_meter_ratio_l1 = self.SIDE_LENGTH_M / l1
        self.pixel_to_meter_ratio_l2 = self.SIDE_LENGTH_M / l2
        self.pixel_to_meter_ratio_l3 = self.SIDE_LENGTH_M / l3
        
        # average the pixel_to_mean ratio of l1, l2, l3
        self.pixel_to_meter_ratio = np.mean([
            self.pixel_to_meter_ratio_l1,
            self.pixel_to_meter_ratio_l2,
            self.pixel_to_meter_ratio_l3
        ])

        print(f"[GEO] Pixel-to-meter ratios: "
            f"L1={self.pixel_to_meter_ratio_l1:.6f}, "
            f"L2={self.pixel_to_meter_ratio_l2:.6f}, "
            f"L3={self.pixel_to_meter_ratio_l3:.6f}")
        
        # Advance to limits calibration phase
        #self.phase = "limits"
        self.phase = "complete"

    def detect_ball_position(self, frame):
        """Detect ball in frame and return position in meters from center.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            float or None: Ball position in meters from center, None if not detected
        """
        if not self.lower_hsv:
            return None, None
        
        # Convert to HSV and create color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None
        
        # Get largest contour (assumed to be ball)
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest) #x,y = ball's center coords
        
        # Filter out very small detections
        if radius < 5:
            return None, None
        
        # Convert pixel position to meters from center
        cx, cy = self.plate_center
        pixel_offset_x = x - cx
        pixel_offset_y = y - cy

        meters_offset_x = pixel_offset_x * self.pixel_to_meter_ratio
        meters_offset_y = -pixel_offset_y * self.pixel_to_meter_ratio #openCV's axis is inverted
        
        return meters_offset_x, meters_offset_y

    def save_config(self):
            """Save all calibration results to config.json file."""
            config = {
                "timestamp": datetime.now().isoformat(),
                "side_length_m": float(self.SIDE_LENGTH_M),
                "camera": {
                    "index": int(self.CAM_INDEX),
                    "frame_width": int(self.FRAME_W),
                    "frame_height": int(self.FRAME_H)
                },
                "ball_detection": {
                    "lower_hsv": [float(x) for x in self.lower_hsv] if self.lower_hsv else None,
                    "upper_hsv": [float(x) for x in self.upper_hsv] if self.upper_hsv else None
                },
                "calibration": {
                    "pixel_to_meter_ratio": float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None,
                    # "position_min_m_x": float(self.position_min[0]) if self.position_min[0] else None,
                    # "position_min_m_y": float(self.position_min[1]) if self.position_min[1] else None,
                    # "position_max_m_x": float(self.position_max[0]) if self.position_max[0] else None,
                    # "position_max_m_y": float(self.position_max[1]) if self.position_max[1] else None
                },
                "servo": {
                    "port": str(self.servo_port),

                    "neutral_tick_1": int(self.neutral_tick[0]),
                    "neutral_tick_2": int(self.neutral_tick[1]),
                    "neutral_tick_3": int(self.neutral_tick[2]),

                    "min_tick_1": int(self.min_tick[0]),
                    "min_tick_2": int(self.min_tick[1]),
                    "min_tick_3": int(self.min_tick[2]),

                    "max_tick_1": int(self.max_tick[0]),
                    "max_tick_2": int(self.max_tick[1]),
                    "max_tick_3": int(self.max_tick[2])
                }
            }
            
            # Write configuration to JSON file
            with open("config.json", "w") as f:
                json.dump(config, f, indent=2)
            print("[SAVE] Configuration saved to config.json")

    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            numpy.ndarray: Frame with overlay graphics and text
        """
        overlay = frame.copy()
        
        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "geometry": "Click on platform endpoints (3 points)",
            #"limits": "Press 'l' to find limits automatically",
            "complete": "Calibration complete! Press 's' to save"
        }
        
        # Draw current phase and instructions
        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, phase_text[self.phase], (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show geometry calibration points
        for i, peg in enumerate(self.peg_points):
            cv2.circle(overlay, peg, 8, (0, 255, 0), -1)
            cv2.putText(overlay, f"Peg {i+1}", (peg[0]+10, peg[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw line between platform endpoints if both are selected
        if len(self.peg_points) == 3:
            cv2.line(overlay, self.peg_points[0], self.peg_points[1], (255, 0, 0), 2)
            cv2.line(overlay, self.peg_points[1], self.peg_points[2], (255, 0, 0), 2)
            cv2.line(overlay, self.peg_points[2], self.peg_points[0], (255, 0, 0), 2)        

        # draw center point
        if self.plate_center is not None:
            cv2.circle(overlay, self.plate_center, 8, (0, 255, 0), -1)

        
        # Show real-time ball detection if color calibration is complete
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find and draw detected ball
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)
                    
                    # Show position if geometry calibration is complete
                    if self.pixel_to_meter_ratio:
                        pos_x, pos_y = self.detect_ball_position(frame)
                        if pos_x is not None and pos_y is not None:
                            cv2.putText(overlay, f"X: {pos_x:.4f}m, Y: {pos_y:.4f}m",
                                        (int(x)+20, int(y)+20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                            
        # # Show final results if limit calibration is complete
        # if (self.position_min[0] is not None and self.position_max[0] is not None
        #     and self.position_min[1] is not None and self.position_max[1] is not None):

        #     cv2.putText(overlay,
        #                 f"X Limits: {self.position_min[0]:.4f}m to {self.position_max[0]:.4f}m",
        #                 (10, overlay.shape[0] - 40),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        #     cv2.putText(overlay,
        #                 f"Y Limits: {self.position_min[1]:.4f}m to {self.position_max[1]:.4f}m",
        #                 (10, overlay.shape[0] - 20),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        return overlay

    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAM_INDEX)  
        if not self.cap.isOpened():
            print("[CAMERA] Failed to open")
            return
        print("[CAMERA] Connected")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Auto Calibration")
        cv2.setMouseCallback("Auto Calibration", self.mouse_callback)
        
        # Attempt servo connection
        self.connect_servo()
        
        # Display instructions
        print("[INFO] Simple Auto Calibration")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        print("Phase 2: Click on platform endpoints")
        #print("Phase 3: Press 'l' to find limits")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Auto Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('c') and self.phase == "color":
                # Complete color calibration phase
                if self.hsv_samples:
                    self.phase = "geometry"
                    print("[INFO] Color calibration complete. Click on platform endpoints.")
            # elif key == ord('l') and self.phase == "limits":
            #     # Start automatic limit finding
            #     self.find_limits_automatically()
            #     self.phase = "complete"
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
        if self.servo:
            self.servo.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = SimpleAutoCalibrator()
    calibrator.run()