import serial
import time

port = "/dev/cu.usbmodem1401" 
SERVOMIN = 102
SERVOMAX = 512

neutral_tick = [SERVOMAX-110, SERVOMAX-80, SERVOMAX-80]  # tested on servo_controller.ino
min_tick = [SERVOMAX-180, SERVOMAX-150, SERVOMAX-150]
max_tick = [SERVOMAX-40, SERVOMAX-10, SERVOMAX-10]


try:
    ser = serial.Serial(port, 9600)
    time.sleep(2)
    print("[SERVO] Connected")

    while True:
        tick = int(input("Enter servo PWM tick: "))
        channel = int(input("Enter motor channel: "))
        print(min_tick[channel])
        print(tick)
        print(max_tick[channel])
        if(min_tick[channel] <= tick <= max_tick[channel]
           and 0 <= channel < 3 ):
            ser.write(tick.to_bytes(2, byteorder="big"))
            ser.write(bytes([channel]))
            print(f"Sent Tick: {tick}")
        else:
            print("Invalid tick")

except Exception as e:
    print(f"[SERVO] {e}")


