"""Two-axis PID and inverse kinematics for a 3-DOF ball-balancing platform.

The inverse-kinematics equations follow the three actuator-plane model in
StewartPlatform_Slides.pdf (S. Bedi, 2025). The three motor angles are converted
to servo angles and sent to servo_controller.ino over serial as a 4-byte packet:
[0xFF][servo1][servo2][servo3].
"""

import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageTk

try:
    import serial  # pip install pyserial
except ImportError:  # Controller still runs (PID + IK only) without pyserial.
    serial = None

from ball_detection import detect_ball_x
SERVOMIN = 102
SERVOMAX = 512

class BasicPIDController:
    """Track a ball and calculate pitch/roll plus the three actuator joint pairs."""

    PROCESSING_SIZE = (640, 480)
    POINT_ANGLES_DEG = np.array([90.0, 210.0, 330.0])
    # The slides label actuator 1 at 90 deg, actuator 2 at 330 deg,
    # and actuator 3 at 210 deg. This maps [actuator 1, 2, 3] to point indices.
    ACTUATOR_POINT_INDICES = (0, 2, 1)
    SERIAL_HEADER = 0xFF  # Must match PACKET_HEADER in servo_controller.ino.

    def __init__(self, config_file="config.json"):
        with open(config_file, "r", encoding="utf-8") as config_stream:
            self.config = json.load(config_stream)
        cal = self.config.get("calibration", {})
        self.setpoint_x = float(cal.get("setpoint_x_m", 0.0))
        self.setpoint_y = float(cal.get("setpoint_y_m", 0.0))
        self.Kp = float(1.0)
        self.Ki = float(0.0)
        self.Kd = float(0.0)
        self.max_tilt_deg = 5.0 #float(self.config.get("pid", {}).get("max_tilt_deg", 15.0))
        self.error_scale = 100.0 #float(self.config.get("pid", {}).get("error_scale", 100.0))

        calibration = self.config.get("calibration", {})
        if "pixel_to_meter_ratio" not in calibration:
            raise KeyError("config.json needs calibration.pixel_to_meter_ratio (meters/pixel)")
        self.pixel_to_meter_ratio = float(calibration["pixel_to_meter_ratio"])
        if self.pixel_to_meter_ratio <= 0:
            raise ValueError("pixel_to_meter_ratio must be positive")
        proc_w, proc_h = self.PROCESSING_SIZE
        self.position_scale = np.array([
            self.pixel_to_meter_ratio * proc_w / 2.0,
            self.pixel_to_meter_ratio * proc_h / 2.0,
        ])

        #self.kinematics_config = self.config.get("kinematics", {})
        self.platform_radius_m = float(0.14)
        self.center_height_m = float(0.0936)
        self.base_offsets_m = np.array([0.05, 0.05, 0.05])
        self.upper_links_m = np.array([0.094, 0.094, 0.094])
        self.lower_links_m = np.array([0.08004, 0.08004, 0.08004])
        self.ik_branch = -1
        self.motor_angle_limits_deg = [-180,180]
        self.kinematics_ready = True #self._validate_kinematics_config()

        serial_config = self.config.get("servo", {})
        self.serial_port_name = serial_config.get("port")
        self.serial_baud = 9600
        self.serial_enabled = bool(self.serial_port_name is not None)
        self.serial_port = None
        self.serial_lock = threading.Lock()

        self.servo_neutral = np.array([146.0, 161.0, 165.0])
        self.servo_min = np.array([101.0, 114.0, 114.0])
        self.servo_max = np.array([162.0, 176.0, 176.0])
        self.servo_direction = np.array([-1.0, -1.0, -1.0])
        if np.any(self.servo_min < 0) or np.any(self.servo_max > 180):
            raise ValueError("servo min_deg/max_deg must lie within 0..180")
        if np.any(self.servo_min > self.servo_neutral) or np.any(self.servo_neutral > self.servo_max):
            raise ValueError("servo neutral_deg must lie between min_deg and max_deg")
        if not np.all(np.isin(self.servo_direction, (-1.0, 1.0))):
            raise ValueError("servo direction entries must each be 1 or -1")
        self.level_motor_angles = self._compute_level_motor_angles()
        self.last_limit_warning = 0.0

        self.integral = np.zeros(2, dtype=float)
        self.prev_error = np.zeros(2, dtype=float)
        self.last_pid_time = None
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.point_log = []
        self.actuator_log = []
        self.servo_log = []
        self.start_time = None
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()
        self.camera_thread_handle = None
        self.control_thread_handle = None
        self.root = None

    def _validate_kinematics_config(self):
        """Check that physical dimensions required by the lecture model exist."""
        required = (
            "platform_radius_m",
            "center_height_m",
            "base_offsets_m",
            "upper_links_m",
            "lower_links_m",
        )
        missing = [key for key in required if key not in self.kinematics_config]
        if missing:
            print("[KINEMATICS] Not configured; add config.kinematics fields: " + ", ".join(missing))
            print("[KINEMATICS] PID will calculate pitch/roll, but actuator IK will remain disabled.")
            return False

        cfg = self.kinematics_config
        if float(cfg["platform_radius_m"]) <= 0 or float(cfg["center_height_m"]) < 0:
            raise ValueError("platform_radius_m must be positive and center_height_m non-negative")
        for key in ("base_offsets_m", "upper_links_m", "lower_links_m"):
            values = np.asarray(cfg[key], dtype=float)
            if values.shape != (3,) or np.any(values <= 0):
                raise ValueError(f"kinematics.{key} must contain three positive values (meters)")
        if int(cfg.get("ik_branch", 1)) not in (-1, 1):
            raise ValueError("kinematics.ik_branch must be either -1 or 1")
        return True


    def _compute_level_motor_angles(self):
        """Motor angles when the plate is level; these map to the servo neutral angles."""
        if not self.kinematics_ready:
            return None
        try:
            level_points = self.calculate_platform_points([0.0, 0.0])
            return self.calculate_actuator_angles(level_points)[:, 0]
        except (ValueError, RuntimeError) as exc:
            print(f"[SERVO] Cannot compute level pose, servo output disabled: {exc}")
            return None

    def motor_to_servo_angles(self, actuator_angles):
        """Convert IK motor angles to servo angles, clamped to the safe servo range.

        servo = neutral + direction * (motor - motor_at_level_plate)
        Returns (servo_angles, was_clamped).
        """
        if self.level_motor_angles is None:
            raise RuntimeError("Level-pose motor angles unavailable; check kinematics config")
        motor = np.asarray(actuator_angles, dtype=float)[:, 0]
        raw = self.servo_neutral + self.servo_direction * (motor - self.level_motor_angles)
        clamped = np.clip(raw, self.servo_min, self.servo_max)
        return clamped, bool(np.any(clamped != raw))

    def connect_serial(self):
        """Open the Arduino serial port and center the servos. Returns True on success."""
        if not self.serial_enabled:
            print("[SERIAL] Disabled; add config.serial.port to send commands to the Arduino")
            return False
        if serial is None:
            print("[SERIAL] pyserial is not installed (pip install pyserial); hardware output disabled")
            return False
        if not self.serial_port_name:
            print("[SERIAL] config.serial.port is not set; hardware output disabled")
            return False
        try:
            self.serial_port = serial.Serial(
                self.serial_port_name, self.serial_baud, timeout=0.1, write_timeout=0.1
            )
        except (serial.SerialException, OSError) as exc:
            print(f"[SERIAL] Could not open {self.serial_port_name}: {exc}")
            self.serial_port = None
            return False

        time.sleep(2.0)  # Most Arduinos auto-reset when the port opens.
        greeting = self.serial_port.readline().decode("ascii", errors="ignore").strip()
        self.serial_port.reset_input_buffer()
        print(f"[SERIAL] Connected to {self.serial_port_name} @ {self.serial_baud}"
              + (f" ({greeting})" if greeting else ""))
        self.send_servo_angles(self.servo_neutral)
        return True

    def send_servo_angles(self, servo_angles):
        angles = np.clip(np.round(np.asarray(servo_angles, dtype=float)), 0, 180)
        ticks = (angles / 180.0 * (SERVOMAX - SERVOMIN) + SERVOMIN).astype(int)
        packet = b"".join(int(t).to_bytes(2, "big") for t in ticks)
        with self.serial_lock:
            if self.serial_port is None:
                return False
            try:
                self.serial_port.write(packet)
                return True
            except (serial.SerialException, OSError) as exc:
                print(f"[SERIAL] Write failed ({exc}); hardware output disabled")
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
                return False

    def shutdown_serial(self):
        """Return the servos to neutral and close the port."""
        if self.serial_port is None:
            return
        self.send_servo_angles(self.servo_neutral)
        time.sleep(0.1)
        with self.serial_lock:
            if self.serial_port is not None:
                self.serial_port.close()
                self.serial_port = None
        print("[SERIAL] Servos centered; port closed")

    def update_pid(self, position, dt=None):
        """Return [pitch_deg, roll_deg] from measured [x_m, y_m]."""
        position = np.asarray(position, dtype=float)
        if position.shape != (2,) or not np.all(np.isfinite(position)):
            raise ValueError("position must be a finite [x_m, y_m] pair")

        now = time.monotonic()
        if dt is None:
            dt = 0.033 if self.last_pid_time is None else now - self.last_pid_time
        self.last_pid_time = now
        dt = float(np.clip(dt, 0.001, 0.25))

        setpoint = np.array([self.setpoint_x, self.setpoint_y], dtype=float)
        error = (setpoint - position) * self.error_scale
        self.integral += error * dt
        # Basic anti-windup; output is also limited below.
        self.integral = np.clip(self.integral, -100.0, 100.0)
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error.copy()
        return np.clip(output, -self.max_tilt_deg, self.max_tilt_deg)

    def reset_integral(self):
        self.integral[:] = 0.0
        self.prev_error[:] = 0.0
        self.last_pid_time = None
        print("[RESET] PID integral and derivative history reset")

    def calculate_platform_points(self, pitch_roll_deg):
        """Return moving-platform XYZ points in [90, 210, 330] degree order.

        Uses R = Ry(pitch) @ Rx(roll), with the local triangular vertices
        equally spaced at the specified angles and z=0 in the plate frame.
        """
        #if not self.kinematics_ready:
        #    raise RuntimeError("Kinematics dimensions are missing from config.json")
        pitch, roll = np.radians(np.asarray(pitch_roll_deg, dtype=float))
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, sr], [0.0, -sr, cr]])
        rotation = rot_y @ rot_x

        radius = float(self.platform_radius_m)
        theta = np.radians(self.POINT_ANGLES_DEG)
        local_points = np.column_stack((radius * np.cos(theta), radius * np.sin(theta), np.zeros(3)))
        center = np.array([0.0, 0.0, float(self.center_height_m)])
        return (rotation @ local_points.T).T + center

    @staticmethod
    def _solve_actuator_ik(radial_position, z_position, base_offset,
                           upper_link, lower_link, branch=1):
        """Solve the slide's two-link actuator model for one motor/joint pair.

        Slide forward model (in each actuator's radial/vertical plane):
          r = l_base + l_upper*cos(theta_motor) - l_lower*cos(theta_joint)
          z = l_upper*sin(theta_motor) + l_lower*sin(theta_joint)

        Returns angles in degrees. ``branch`` selects one of the two elbow
        configurations; it must be validated against the actual mechanism.
        """
        target_x = float(radial_position) - float(base_offset)
        target_z = float(z_position)
        l1, l2 = float(upper_link), float(lower_link)
        distance = float(np.hypot(target_x, target_z))
        eps = 1e-10
        if distance < eps:
            raise ValueError("IK target is at the motor pivot; direction is undefined")
        if distance > l1 + l2 + eps or distance < abs(l1 - l2) - eps:
            raise ValueError(
                f"IK target unreachable: distance={distance:.5f} m, "
                f"link range=[{abs(l1-l2):.5f}, {l1+l2:.5f}] m"
            )

        cos_alpha = np.clip((distance**2 + l1**2 - l2**2) / (2.0 * distance * l1), -1.0, 1.0)
        alpha = np.arccos(cos_alpha)
        target_direction = np.arctan2(target_z, target_x)
        theta_motor = target_direction + int(branch) * alpha

        # The lower-link vector is (-l2*cos(theta_joint), l2*sin(theta_joint)).
        lower_x = target_x - l1 * np.cos(theta_motor)
        lower_z = target_z - l1 * np.sin(theta_motor)
        theta_lower_vector = np.arctan2(lower_z, lower_x)
        theta_joint = np.pi - theta_lower_vector

        return np.degrees([theta_motor, theta_joint])

    def calculate_actuator_angles(self, platform_points):
        """Return actuator 1..3 [motor_angle, elbow_angle] using slide geometry."""
        #if not self.kinematics_ready:
        #    raise RuntimeError("Kinematics dimensions are missing from config.json")
        points = np.asarray(platform_points, dtype=float)
        if points.shape != (3, 3):
            raise ValueError("platform_points must have shape (3, 3)")

        offsets = np.asarray(self.base_offsets_m, dtype=float)
        upper = np.asarray(self.upper_links_m, dtype=float)
        lower = np.asarray(self.lower_links_m, dtype=float)
        branch = int(self.ik_branch)
        limits = self.motor_angle_limits_deg
        results = []
        for actuator_index, point_index in enumerate(self.ACTUATOR_POINT_INDICES):
            point = points[point_index]
            # The lecture model uses sqrt(x^2+y^2) as the radial coordinate
            # for all three actuator planes (actuator 1 is shown in y-z plane).
            radial = float(np.hypot(point[0], point[1]))
            pair = self._solve_actuator_ik(
                radial, point[2], offsets[actuator_index],
                upper[actuator_index], lower[actuator_index], branch,
            )
            if limits is not None:
                if len(limits) != 2:
                    raise ValueError("motor_angle_limits_deg must be [min_deg, max_deg]")
                if not float(limits[0]) <= pair[0] <= float(limits[1]):
                    raise ValueError(
                        f"Actuator {actuator_index + 1} motor angle {pair[0]:.2f} deg "
                        f"is outside configured limits {limits}"
                    )
            results.append(pair)
        return np.asarray(results)

    def camera_thread(self):
        camera_index = self.config.get("camera", {}).get("index", 0)
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print(f"[CAMERA] Failed to open camera index {camera_index}")
            self.running = False
            return

        print("[CAMERA] Connected")
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue
                frame = cv2.resize(frame, self.PROCESSING_SIZE)
                found, normalized_xy, vis_frame = detect_ball_x(frame)
                with self.latest_frame_lock:
                    self.latest_frame = vis_frame
                if found:
                    position_m = np.asarray(normalized_xy, dtype=float) * self.position_scale
                    try:
                        self.position_queue.put_nowait(position_m)
                    except queue.Full:
                        try:
                            self.position_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self.position_queue.put_nowait(position_m)
                        except queue.Full:
                            pass
        finally:
            cap.release()

    def control_thread(self):
        """Calculate PID, inverse kinematics, and send servo angles to the Arduino."""
        self.start_time = time.time()
        while self.running:
            try:
                position = self.position_queue.get(timeout=0.1)
                pitch_roll = self.update_pid(position)
                points = None
                actuator_angles = None
                servo_angles = None
                if self.kinematics_ready:
                    points = self.calculate_platform_points(pitch_roll)
                    actuator_angles = self.calculate_actuator_angles(points)
                    servo_angles, clamped = self.motor_to_servo_angles(actuator_angles)
                    if clamped and time.monotonic() - self.last_limit_warning > 1.0:
                        print("[SERVO] Command exceeded servo limits and was clamped")
                        self.last_limit_warning = time.monotonic()
                    self.send_servo_angles(servo_angles)

                elapsed = time.time() - self.start_time
                self.time_log.append(elapsed)
                self.position_log.append(position.copy())
                self.setpoint_log.append(np.array([self.setpoint_x, self.setpoint_y]))
                self.control_log.append(pitch_roll.copy())
                if points is not None:
                    self.point_log.append(points.copy())
                    self.actuator_log.append(actuator_angles.copy())
                    self.servo_log.append(servo_angles.copy())

                print(
                    f"[PID] ball=({position[0]:+.4f}, {position[1]:+.4f}) m | "
                    f"pitch={pitch_roll[0]:+.2f} deg | roll={pitch_roll[1]:+.2f} deg"
                )
                if actuator_angles is not None:
                    for i, angles in enumerate(actuator_angles, start=1):
                        point = points[self.ACTUATOR_POINT_INDICES[i - 1]]
                        print(
                            f"[IK] actuator {i}: platform point="
                            f"({point[0]:+.4f}, {point[1]:+.4f}, {point[2]:+.4f}) m; "
                            f"motor={angles[0]:+.2f} deg, elbow={angles[1]:+.2f} deg"
                        )
                    print(
                        f"[SERVO] command: {servo_angles[0]:.1f}, "
                        f"{servo_angles[1]:.1f}, {servo_angles[2]:.1f} deg"
                    )
            except queue.Empty:
                continue
            except (ValueError, RuntimeError) as exc:
                print(f"[KINEMATICS/PID] {exc}")
            except Exception as exc:
                print(f"[CONTROL] Unexpected error: {exc}")
                self.running = False
                break

    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("3-DOF Ball Balancing PID Controller")
        self.root.geometry("560x760")
        self.camera_label = ttk.Label(self.root)
        self.camera_label.pack(pady=8)
        ttk.Label(self.root, text="Two-axis PID gains", font=("Arial", 18, "bold")).pack(pady=8)

        self.kp_var = tk.DoubleVar(value=self.Kp)
        self.ki_var = tk.DoubleVar(value=self.Ki)
        self.kd_var = tk.DoubleVar(value=self.Kd)
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)

        for label, variable, low, high in (
            ("Kp", self.kp_var, 0, 100),
            ("Ki", self.ki_var, 0, 10),
            ("Kd", self.kd_var, 0, 20),
        ):
            ttk.Label(self.root, text=label, font=("Arial", 11)).pack()
            ttk.Scale(self.root, from_=low, to=high, variable=variable,
                      orient=tk.HORIZONTAL, length=500).pack(pady=3)

        calibration = self.config["calibration"]
        x_min = float(calibration.get("position_min_m", -0.05))
        x_max = float(calibration.get("position_max_m", 0.05))
        y_min = float(calibration.get("position_min_y_m", x_min))
        y_max = float(calibration.get("position_max_y_m", x_max))
        ttk.Label(self.root, text="X setpoint (m)").pack()
        ttk.Scale(self.root, from_=x_min, to=x_max, variable=self.setpoint_x_var,
                  orient=tk.HORIZONTAL, length=500).pack(pady=3)
        ttk.Label(self.root, text="Y setpoint (m)").pack()
        ttk.Scale(self.root, from_=y_min, to=y_max, variable=self.setpoint_y_var,
                  orient=tk.HORIZONTAL, length=500).pack(pady=3)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=12)
        ttk.Button(button_frame, text="Reset PID", command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results", command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop", command=self.stop).pack(side=tk.LEFT, padx=5)
        self.root.protocol("WM_DELETE_WINDOW", self.stop)
        self.update_gui()

    def update_gui(self):
        if not self.running or self.root is None:
            return
        with self.latest_frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = ImageTk.PhotoImage(Image.fromarray(rgb))
            self.camera_label.configure(image=image)
            self.camera_label.image = image

        self.Kp = self.kp_var.get()
        self.Ki = self.ki_var.get()
        self.Kd = self.kd_var.get()
        self.setpoint_x = self.setpoint_x_var.get()
        self.setpoint_y = self.setpoint_y_var.get()
        self.root.after(50, self.update_gui)

    def plot_results(self):
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        times = np.asarray(self.time_log)
        positions = np.asarray(self.position_log)
        setpoints = np.asarray(self.setpoint_log)
        controls = np.asarray(self.control_log)
        fig, (ax_pos, ax_angles) = plt.subplots(2, 1, figsize=(10, 8))
        ax_pos.plot(times, positions[:, 0], label="Ball X")
        ax_pos.plot(times, positions[:, 1], label="Ball Y")
        ax_pos.plot(times, setpoints[:, 0], "--", label="X setpoint")
        ax_pos.plot(times, setpoints[:, 1], "--", label="Y setpoint")
        ax_pos.set_ylabel("Position (m)")
        ax_pos.legend()
        ax_pos.grid(True, alpha=0.3)
        ax_angles.plot(times, controls[:, 0], label="Pitch command")
        ax_angles.plot(times, controls[:, 1], label="Roll command")
        ax_angles.set_xlabel("Time (s)")
        ax_angles.set_ylabel("Angle (degrees)")
        ax_angles.legend()
        ax_angles.grid(True, alpha=0.3)
        fig.suptitle(f"Two-axis PID (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        plt.tight_layout()
        plt.show()

    def stop(self):
        self.running = False
        if self.root is not None:
            try:
                self.root.quit()
                self.root.destroy()
            except tk.TclError:
                pass

    def run(self):
        print("[INFO] Starting 3-DOF ball-balancing controller")
        print("[INFO] PID -> pitch/roll -> IK -> servo angles -> Arduino (when serial is configured)")
        self.running = True
        self.connect_serial()
        self.create_gui()
        self.camera_thread_handle = threading.Thread(target=self.camera_thread, daemon=True)
        self.control_thread_handle = threading.Thread(target=self.control_thread, daemon=True)
        self.camera_thread_handle.start()
        self.control_thread_handle.start()
        self.root.mainloop()
        self.running = False
        for thread in (self.camera_thread_handle, self.control_thread_handle):
            if thread is not None:
                thread.join(timeout=1.0)
        self.shutdown_serial()
        print("[INFO] Controller stopped")


if __name__ == "__main__":
    try:
        BasicPIDController().run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run the calibration setup first.")
    except Exception as exc:
        print(f"[ERROR] {exc}")