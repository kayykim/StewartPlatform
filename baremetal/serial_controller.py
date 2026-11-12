import serial
import time


def connect_serial(port):
    try:
        servo = serial.Serial(port, 9600)
        time.sleep(2)
        print("[SERVO] Connected")
        return servo
    except Exception as e:
        print(f"[SERVO] Failed: {e}")
        return False

def disconnect_serial(servo):
    servo.close()
    print("[SERVO] Disconnected")

def send_ardunio_angle(angle, servo, min_tick, max_tick, SERVOMAX=512, SERVOMIN=102):
    tick = [146, 161, 165]

    for i in range(3):
        # convert angle -> tick to send to arduino
        tick[i]=( int((angle[i]/180.0) * (SERVOMAX - SERVOMIN) + SERVOMIN) )

    # validate tick angle 
    if all(min_tick[i] <= tick[i] <= max_tick[i] for i in range (3)): # makes sure motor 1: 332<= tick <=472
        for i in range(3):
                servo.write(tick[i].to_bytes(2, byteorder="big"))
                print(f"Sent Tick: {tick[i]} \n Limits: {max_tick[i]} and {min_tick[i]}")
        #print(f"Sent Ticks: {tick} \nLimits: {max_tick[i]} and {min_tick[i]}")   
    else:
        print(f"Invalid tick!\n")
            

