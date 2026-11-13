import cv2
import numpy as np

class Camera:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("[CAMERA ERROR] Could not open camera")
        self.current_frame = None

        # color samples 
        self.hsv_samples = []
        self.lower_hsv = None
        self.upper_hsv = None

        # peg points for calc geometry 
        self.peg_points = []
        self.plate_center = None
        self.pixel_to_meter_ratio = None
        self.platform_radius_m = 0.14

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("[CAMERA ERROR] Could not read camera frame")
        return frame

    def release(self):
        self.cap.release()

    def sample_color(self, x, y):
        if self.current_frame is None:
            return

        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x+dx, y+dy

                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])

        samples = np.array(self.hsv_samples, dtype=float)

        margins = np.maximum(
            [5, 10, 10],
            (samples.max(axis=0) - samples.min(axis=0)) * [0.1, 0.15, 0.15]
        )

        self.lower_hsv = np.maximum(
            samples.min(axis=0)-margins, 0
        ).astype(np.uint8)

        self.upper_hsv = np.minimum(
            samples.max(axis=0) + margins, [179, 255, 255]
        ).astype(np.uint8)

        print("[HSV]", self.lower_hsv, self.upper_hsv)

    def detect_ball(self, frame):
        if self.lower_hsv is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)

        if radius < 5:
            return None #ignore specs
        
        return (int(x), int(y), int(radius))

    def draw_overlay(self, frame, ball_position):
        overlay = frame.copy()

        cv2.putText(
            overlay, "Click ball to calibrate HSV",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2
        )

        if ball_position is not None:
            x, y = ball_position

            cv2.circle(overlay, (x, y), 5, (0, 255, 255), -1)

            cv2.putText(
                overlay, f"Ball: ({x}, {y}) px",
                (x + 15, y + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 255), 1
            )

        return overlay

    def calibrate_geometry(self, center, edge):
        self.platform_center = center
        pixel_radius = np.hypot(
            edge[0] - center[0],
            edge[1] - center[1]
        )
        self.pixel_to_meter_ratio = self.platform_radius_m/pixel_radius

        print("Platform Center:", self.platform_center)
        print("Pixel-to-Meter Ratio:", self.pixel_to_meter_ratio)

    def get_ball_position_m(self, ball_position):
        x, y, radius = ball_position

        center_x, center_y = self.platform_center

        x_m = (x - center_x) * self.pixel_to_meter_ratio
        y_m = -(y - center_y) * self.pixel_to_meter_ratio

        return (x_m, y_m)