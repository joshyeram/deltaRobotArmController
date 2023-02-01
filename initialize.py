import time
from adafruit_servokit import ServoKit
from enum import IntEnum
import random
from kinematics import *

frontRange = (35, 160)
rightRange = (172, 50)
leftRange = (35, 160)
kit = ServoKit(channels=16)

class Servo(IntEnum):
    FRONT = 3 # min 32, max 153: range = 121 for roughly 90
    RIGHT = 1 # min 175, max 53: range 122 for for roughly 90
    LEFT = 2 # min 33, max 158: range = 125 for roughly 90
    TOP = 8 # min 33, max 158: range = 125 for roughly 90
    BOTTOM = 9 # min 33, max 158: range = 125 for roughly 90


def indvControl(which):
    testing = 90
    kit.servo[which].angle = testing
    while True:
        i = input("please input")
        if(i == "p"):
            print(testing)
        if(i == "u"):
            testing +=1
        if(i == "d"):
            testing -=1
        if(i == "ul"):
            testing +=5
        if(i == "dl"):
            testing -=5
        kit.servo[which].angle = testing
        time.sleep(1)

def control(servo, degree):
    if(degree < 0):
        degree = 0
    if(degree > 80):
        degree = 80
    if(servo == Servo.FRONT):
        ratio = (frontRange[1] - frontRange[0])/ 90
        pos = ratio * degree + frontRange[0]
        kit.servo[Servo.FRONT].angle = pos
    if(servo == Servo.RIGHT):
        ratio = (rightRange[0] - rightRange[1])/ 90
        pos = rightRange[0] - ratio * degree 
        kit.servo[Servo.RIGHT].angle = pos
    if(servo == Servo.LEFT):
        ratio = (leftRange[1] - leftRange[0])/ 90
        pos = ratio * degree + leftRange[0]
        kit.servo[Servo.LEFT].angle = pos

def getAngle(servo):
    front = kit.servo[Servo.FRONT].angle
    right = kit.servo[Servo.RIGHT].angle
    left = kit.servo[Servo.LEFT].angle
    if(servo == Servo.FRONT):
        ratio = (frontRange[1] - frontRange[0])/ 90
        pos = (front - frontRange[0])/ratio
        return pos
    if(servo == Servo.RIGHT):
        ratio = (rightRange[0] - rightRange[1])/ 90
        pos = (rightRange[0]-right)/ratio
        return pos
    if(servo == Servo.LEFT):
        ratio = (leftRange[1] - leftRange[0])/ 90
        pos = (left - leftRange[0])/ ratio
        return pos

def init():
    front = getAngle(Servo.FRONT)
    right = getAngle(Servo.RIGHT)
    left = getAngle(Servo.LEFT)
    grab(False)
    while True:
        front = getAngle(Servo.FRONT)
        right = getAngle(Servo.RIGHT)
        left = getAngle(Servo.LEFT)
        control(Servo.FRONT, front * .9)
        control(Servo.RIGHT, right * .9)
        control(Servo.LEFT, left * .9)
        time.sleep(.001)
        if (front < 5 and right < 5 and left < 5):
            break
    
    control(Servo.FRONT, 0)
    control(Servo.RIGHT, 0)
    control(Servo.LEFT, 0)
        
def randomPos():
    control(Servo.FRONT, random.randint(45, 80))
    control(Servo.RIGHT, random.randint(45, 80))
    control(Servo.LEFT, random.randint(45, 80))

def getPointsOnCircle(r,theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return np.array([x,y])

def moveTo(pos):
    if(pos is None):
        return
    x,y,z = pos
    f, r, l = inverseKinematics(x,y,z)
    control(Servo.FRONT, f)
    control(Servo.RIGHT, r)
    control(Servo.LEFT, l)

def grab(open): 
    #true to grab
    #false to open
    if(open):
        kit.servo[Servo.TOP].angle = 128
        kit.servo[Servo.BOTTOM].angle = 130
    else:
        kit.servo[Servo.TOP].angle = 5
        kit.servo[Servo.BOTTOM].angle = 5
