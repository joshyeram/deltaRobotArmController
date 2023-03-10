import cv2
import numpy as np

def whiteCubeExtract(frame):
    dimensions = frame.shape
    height = dimensions[0]
    width = dimensions[1]
    imgGry = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thrash = cv2.threshold(imgGry, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    temp = []
    for cou in contours:
        M = cv2.moments(cou)
        if cv2.contourArea(cou) >= 50 and M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            d = np.sqrt((cx - width / 2) ** 2 + (cy - height / 2) ** 2)
            r = height / 2 * .95
            if (d < r):
                temp.append(cou)
    return temp

def distance(cous):
    if(len(cous) == 0):
        return None
    temp = []
    for cou in cous:
        area = cv2.contourArea(cou)
        dis = 1359 * area ** (-.568)
        temp.append(dis)
    return min(temp) * -1

def deltaPos(cous):
    if (len(cous) == 0):
        return None
    def deltaDis(cou):
        M = cv2.moments(cou)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            d = np.sqrt((cx - 320 / 2) ** 2 + (cy - 240 / 2) ** 2)
            return d
        else:
            exit("delta dis failed")
    center = sorted(cous, key=lambda x: deltaDis(x))[0]
    sideLength = cv2.contourArea(center) ** .5
    scale = (30 / sideLength) * 1
    M = cv2.moments(center)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    dx = (320 / 2 - cx) * scale
    dy = (240 / 2 - cy) * scale
    return np.array([dx, dy])

def estimatedCubePose(cous):
    z = distance(cous)
    temp = deltaPos(cous)
    if(z is None or temp is None):
        return None
    return np.array([temp[0], temp[1], z])

"""vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while(True):
    ret, frame = vid.read()
    frame = cv2.resize(frame, (320, 240))
    contours = whiteCubeExtract(frame)
    print(estimatedCubePose(contours))
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
vid.release()
cv2.destroyAllWindows()"""