from initialize import *
from threading import Thread

servo = (0,0,0)
delay = .01

def servoController():
    while True:
        if(servo == None):
            continue
        front, right, left = servo 
        currF = getAngle(Servo.FRONT)
        currR = getAngle(Servo.RIGHT)
        currL = getAngle(Servo.LEFT)

        if(abs(currF-front)<1 and abs(currR-right)<1 and abs(currL-left)<1):
            continue

        adjF = (front * .2) + (currF * .8)
        adjR = (right * .2) + (currR * .8)
        adjL = (left * .2) + (currL * .8)

        control(Servo.FRONT, adjF)
        control(Servo.RIGHT, adjR)
        control(Servo.LEFT, adjL)

        time.sleep(delay)

def circle():
    i = 0
    while True:
        x, y = getPointsOnCircle(100, i)
        global servo
        servo = tuple(inverseKinematics(x,y,-300))
        i+=5
        time.sleep(.1)

init()
time.sleep(1)
thread = Thread(target = servoController)
thread.start()

circle()
        
