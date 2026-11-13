# from ui import
# from pid import
from serial_controller import connect_serial, disconnect_serial, send_ardunio_angle
from kinematics import calculate_attachment_points, calculate_actuator_angles, convert_ik_to_servo_angles
from camera import Camera
import json
import cv2

class BasicPIDController():
    def __init__(self, config_file="config.json"):
        with open(config_file, 'r') as f:
            self.config = json.load(f)

        # PID gains (controlled by sliders in GUI)
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0

        # Initalize servo
        servo_config = self.config["servo"]
        self.servo_port = servo_config["port"]
        self.min_tick = servo_config["min_tick"]
        self.max_tick = servo_config["max_tick"]
        self.neutral_tick = servo_config["neutral_tick"]
        self.min_angle = servo_config["min_angle_deg"]
        self.max_angle = servo_config["max_angle_deg"]
        self.neutral_angle = servo_config["neutral_angle_deg"]
        self.servo_ticks = self.neutral_tick.copy()

        self.servo = None

        # Initalize kinematics
        kinematics_config = self.config["kinematics"]
        self.platform_radius = kinematics_config["platform_radius_m"]
        self.center_height = kinematics_config["center_height_m"]
        self.base_offsets = kinematics_config["base_offsets_m"]
        self.upper_links = kinematics_config["upper_links_m"]
        self.lower_links = kinematics_config["lower_links_m"]
        self.ik_branch = kinematics_config["ik_branch"]

        # Controller Status 
        self.running = False    # Main run flag for clean shutdown

    def send_servo_angle(self, angle):
        send_ardunio_angle(angle, self.servo, self.min_tick, self.max_tick)

    def start(self):
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        self.servo = connect_serial(self.servo_port)    # connect to ardiuno
        
        # Calculate neutral IK
        neutral_points = calculate_attachment_points(
            self.platform_radius, self.center_height, 0, 0
        )
        neutral_ik = calculate_actuator_angles(
            neutral_points, self.base_offsets, self.upper_links, self.lower_links
        )[:, 0]

        # Calculate IK for desired tilt
        points = calculate_attachment_points(
            self.platform_radius, 
            self.center_height, 
            tilt_x=0, tilt_y=0 
        )
        ik_angles = calculate_actuator_angles(
            points, self.base_offsets, self.upper_links, self.lower_links
        )[:, 0]

        # Convert to servo angles
        servo_angles = convert_ik_to_servo_angles(
            ik_angles, neutral_ik, self.neutral_angle
        )

        print("Servo angles:", servo_angles)

        self.send_servo_angle(servo_angles)

    def shutdown(self):
        self.running = False
        if self.servo:
            disconnect_serial(self.servo)
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    controller = None
    camera = None

    try:
        # --- camera ---
        camera = Camera()
        cv2.namedWindow("Stewart Platform Camera")

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                camera.sample_color(x, y)

        cv2.setMouseCallback("Stewart Platform Camera", mouse_callback)

        while True:
            frame = camera.get_frame()
            camera.current_frame = frame

            ball_position = camera.detect_ball(frame)

            if ball_position is not None:
                x, y, radius = ball_position

                cv2.circle(frame, (x, y), radius, (0, 255, 0), 2)
                cv2.circle(frame, (x, y), 3, (0, 255, 255), -1)
                print("Ball Position:", (x, y))

            cv2.imshow("Stewart Platform Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # --- controller ---
        controller = BasicPIDController()
        controller.start()
        input("Press Q to quit. ")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        if controller is not None:
            controller.shutdown()