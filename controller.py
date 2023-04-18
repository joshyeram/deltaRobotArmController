from initialize import *
from webcam import *
from threading import Thread
import cv2
import numpy as np

servo = (0,0,0)
delay = .005

def servoController():
    while True:
        if(servo == None):
            continue
        front, right, left = servo 
        currF = getAngle(Servo.FRONT)
        currR = getAngle(Servo.RIGHT)
        currL = getAngle(Servo.LEFT)

        if(abs(currF-front)<2 and abs(currR-right)<2 and abs(currL-left)<2):
            time.sleep(delay)
            continue

        adjF = (front * .005) + (currF * .995)
        adjR = (right * .005) + (currR * .995)
        adjL = (left * .005) + (currL * .995)

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

        currF = (front * .02) + (currF * .98)
        currR = (right * .02) + (currR * .98)
        currL = (left * .02) + (currL * .98)

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
    notVis = 0
    while(True):
        ret, frame = vid.read()
        frame = cv2.resize(frame, (320, 240))
        count +=1
        contours = whiteCubeExtract(frame)
        if(len(contours) == 0):
            notVis+=1
            if(notVis >=60):
                notVis = 0
                grab(False)
                servoSmooterController((0,0,0))
        else:
            notVis = 0
        deltaPose = estimatedCubePose(contours)
        currPose = getCurrentPose()
        if(deltaPose is not None and np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)>=5 and count >=15):
            grab(False)
            if(np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)>=150):
                d = np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)
                deltaPose =  np.array([deltaPose[0]/d * 140, deltaPose[1]/d * 140, deltaPose[2]])
            if(currPose is None):
                print("current pose is not valid")
            x,y,z = np.add(deltaPose, currPose)
            z = currPose[2]
            angles = inverseKinematics(x, y, -300)
            if(angles is not None):
                global servo
                servo = tuple(angles)
                servoSmooterController(servo)
                count = 0
            else:
                print("not valid spot for arm")
        elif(deltaPose is not None and np.sqrt(deltaPose[0]**2 + deltaPose[1]**2)<5 and count >=15):
            grabPos = getCurrentPose()
            angles = inverseKinematics(grabPos[0], grabPos[1], -420)
            if(angles is not None):
                servo = tuple(angles)
                servoSmooterController(servo)
                time.sleep(1)
                grab(True)
                time.sleep(1)
                servoSmooterController((0,0,0))
                time.sleep(3)
                grab(False)
                count = 0
        #SanityCheck
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        frame = cv2.resize(frame, (1280, 960))
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

    vid.release()
    cv2.destroyAllWindows()

init()
time.sleep(1)

hover()
