"""HSV-based ball detection for the 3-DOF ball-balancing platform.

The public ``detect_ball_x`` name is retained for compatibility, although it
now returns normalized (x, y) coordinates rather than x alone.
"""

import json
import os

import cv2
import numpy as np


class BallDetector:
    """Detect an orange ball and report its normalized image coordinates."""

    def __init__(self, config_file="config.json"):
        self.lower_hsv = np.array([5, 150, 150], dtype=np.uint8)
        self.upper_hsv = np.array([20, 255, 255], dtype=np.uint8)
        self.invert_y = True  # Image y increases down; platform y is positive up.

        if os.path.exists(config_file):
            try:
                with open(config_file, "r", encoding="utf-8") as config_stream:
                    config = json.load(config_stream)
                detection_config = config.get("ball_detection", {})
                lower = detection_config.get("lower_hsv")
                upper = detection_config.get("upper_hsv")
                if lower is not None:
                    self.lower_hsv = np.asarray(lower, dtype=np.uint8)
                if upper is not None:
                    self.upper_hsv = np.asarray(upper, dtype=np.uint8)
                self.invert_y = detection_config.get("invert_y", True)
                if self.lower_hsv.shape != (3,) or self.upper_hsv.shape != (3,):
                    raise ValueError("HSV bounds must each contain three values")
                print(f"[BALL_DETECT] HSV bounds: {self.lower_hsv} to {self.upper_hsv}")
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
                print(f"[BALL_DETECT] Config load error: {exc}; using defaults")
        else:
            print("[BALL_DETECT] No config file found; using default HSV bounds")

    def detect_ball(self, frame):
        """Return ``(found, center_px, radius_px, (x_norm, y_norm))``."""
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            raise ValueError("frame must be a non-empty BGR image")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must have three BGR channels")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, (0.0, 0.0)

        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)
        if radius < 5 or radius > 100:
            return False, None, None, (0.0, 0.0)

        height, width = frame.shape[:2]
        x_norm = (x - width / 2.0) / (width / 2.0)
        y_norm = (y - height / 2.0) / (height / 2.0)
        if self.invert_y:
            y_norm = -y_norm

        return True, (int(round(x)), int(round(y))), float(radius), (
            float(np.clip(x_norm, -1.0, 1.0)),
            float(np.clip(y_norm, -1.0, 1.0)),
        )

    def draw_detection(self, frame, show_info=True):
        found, center, radius, position_norm = self.detect_ball(frame)
        overlay = frame.copy()
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        cv2.line(overlay, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        cv2.line(overlay, (0, center_y), (width, center_y), (100, 100, 100), 1)
        cv2.putText(overlay, "Center", (center_x + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if found:
            cv2.circle(overlay, center, int(round(radius)), (0, 255, 0), 2)
            cv2.circle(overlay, center, 3, (0, 255, 0), -1)
            if show_info:
                cv2.putText(overlay, f"x: {center[0]}, y: {center[1]}",
                            (max(5, center[0] - 45), max(20, center[1] - 40)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(overlay, f"norm: ({position_norm[0]:.3f}, {position_norm[1]:.3f})",
                            (max(5, center[0] - 55), max(40, center[1] - 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        return overlay, found, np.asarray(position_norm, dtype=float)


# One detector instance is shared by the legacy-compatible function.
_detector = BallDetector()


def detect_ball_x(frame):
    """Return ``(found, normalized_xy, visualization_frame)``.
n
    The historical function name is kept so existing imports continue to work.
    """
    vis_frame, found, position_norm = _detector.draw_detection(frame)
    return found, position_norm, vis_frame


def main():
    detector = BallDetector()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera 0")
    print("Ball detection test; press q to quit")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[CAMERA] Failed to grab frame")
                break
            frame = cv2.resize(frame, (320, 240))
            vis_frame, found, position_norm = detector.draw_detection(frame)
            if found:
                print(f"Ball normalized position: x={position_norm[0]:.3f}, y={position_norm[1]:.3f}")
            cv2.imshow("Ball Detection Test", vis_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
