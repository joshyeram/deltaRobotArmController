from initialize import *
from webcam import *
from threading import Thread
import cv2
import numpy as np

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

def servoSimpleController():
    while True:
        if(servo == None):
            continue
        front, right, left = servo 
        control(Servo.FRONT, front)
        control(Servo.RIGHT, right)
        control(Servo.LEFT, left)
        time.sleep(delay)

def servoSmooterController(servoT):
    currF = getAngle(Servo.FRONT)
    currR = getAngle(Servo.RIGHT)
    currL = getAngle(Servo.LEFT)
    while True:
        if(servoT == None):
            break
        front, right, left = servoT 

        if(abs(currF-front)<3 and abs(currR-right)<3 and abs(currL-left)<3):
            control(Servo.FRONT, front)
            control(Servo.RIGHT, right)
            control(Servo.LEFT, left)
            break

        currF = (front * .05) + (currF * .95)
        currR = (right * .05) + (currR * .95)
        currL = (left * .05) + (currL * .95)

        control(Servo.FRONT, currF)
        control(Servo.RIGHT, currR)
        control(Servo.LEFT, currL)

        time.sleep(delay)

def circle():
    i = 0
    while True:
        x, y = getPointsOnCircle(100, i)
        global servo
        servo = tuple(inverseKinematics(x,y,-300))
        i+=5
        time.sleep(.1)

def getCurrentPose():
    currF = getAngle(Servo.FRONT)
    currR = getAngle(Servo.RIGHT)
    currL = getAngle(Servo.LEFT)
    calc = forwardKinematics(currF, currR, currL)
    return calc

def hover():
    vid = cv2.VideoCapture(0)
    vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    count = 0
    while(True):
        print("running")
        ret, frame = vid.read()
        frame = cv2.resize(frame, (320, 240))
        count +=1
        contours = whiteCubeExtract(frame)
        deltaPose = estimatedCubePose(contours)
        currPose = getCurrentPose()
        if(deltaPose is not None and np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)>=3 and count >=30):
            if(np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)>=150):
                d = np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)
                deltaPose =  np.array([deltaPose[0]/d * 140, deltaPose[1]/d * 140, deltaPose[2]])
            if(currPose is None):
                print("current pose is not valid")
            x,y,z = np.add(deltaPose, currPose)
            z = currPose[2]
            print("pose",x,y,z)
            angles = inverseKinematics(x, y, z)
            if(angles is not None):
                global servo
                servo = tuple(angles)
                servoSmooterController(servo)
                count = 0
            else:
                print("not valid spot for arm")
        #SanityCheck
        print("cube pose", deltaPose)
        print("current arm pose", currPose)
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

    vid.release()
    cv2.destroyAllWindows()

init()
time.sleep(1)
"""thread = Thread(target = servoSimpleController)
thread.start()"""

hover()
        
