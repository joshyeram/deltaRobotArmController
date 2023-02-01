import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

def simpleSmoooth(ang):
    curr = kit.servo[15].angle
    while True:
        curr = (.005 * ang) + (.995 * curr)
        if(abs(curr - ang)<2):
            kit.servo[15].angle = ang
            return
        kit.servo[15].angle = curr
        print(curr)
        time.sleep(.005)

while True:
    simpleSmoooth(0)
    time.sleep(1)
    simpleSmoooth(170)
    time.sleep(1)
