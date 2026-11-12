# from ui import
# from camera import
# from kinematics import
# from pid import
from serial_controller import connect_serial, disconnect_serial, send_ardunio_angle

import json

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
        self.servo_ticks = self.neutral_tick.copy()

        self.servo = None

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
        #create_gui(self)                               # launch ui 

        # send it neutral angle
        self.send_servo_angle([161, 175, 175])

    def shutdown(self):
        self.running = False
        if self.servo:
            disconnect_serial(self.servo)
        print("[INFO] Controller stopped")
if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.start()
        input("Press Q to quit. ")

    except Exception as e:
        print(f"[ERROR] {e}")

    finally:
        controller.shutdown()