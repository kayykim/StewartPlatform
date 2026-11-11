import serial
import time

port = "/dev/cu.usbmodem1401" 
SERVOMIN = 102
SERVOMAX = 512

neutral_tick = [SERVOMAX-110, SERVOMAX-80, SERVOMAX-80]  # 402, 432, 432
min_tick = [SERVOMAX-180, SERVOMAX-150, SERVOMAX-150]    # 332, 362, 362
max_tick = [SERVOMAX-40, SERVOMAX-10, SERVOMAX-10]       # 472, 502, 502
angle_input = [0, 0, 0]
tick_packet = [0, 0, 0]

try:
    ser = serial.Serial(port, 9600)
    time.sleep(2)
    print("[SERVO] Connected")

    while True:
        for i in range (3):
            angle_input[i] = int(input(f"Enter Motor {i+1} angle: "))
            tick_packet[i] = int((angle_input[i]/180.0) * (SERVOMAX - SERVOMIN) + SERVOMIN)
        # validate tick angle 
        if all(min_tick[i] <= tick_packet[i] <= max_tick[i] for i in range (3)): # makes sure motor 1: 332<= tick <=472
            for i in range(3):
                 ser.write(tick_packet[i].to_bytes(2, byteorder="big"))
            print(f"Packet sent: {tick_packet}")
            print(f"Sent Ticks: {tick_packet}")   
        else:
            print("Invalid tick")
            

except Exception as e:
    print(f"[SERVO] {e}")


