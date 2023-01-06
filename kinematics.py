import math
import time

import numpy as np

effR = 32
baseR = 63
forearm = 285
bicep = 200

"""
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;
 """

fMotor = [0, baseR, 0]
rMotor = [math.cos(math.radians(30)) * baseR, -baseR / 2, 0]
lMotor = [-math.cos(math.radians(30)) * baseR, -baseR / 2, 0]


def inverseKinematics(x, y, z):
    if(math.sqrt(x**2 + y**2)>=150):
        #print("not reachable1")
        return None
    def inverseKinematicsHelper(endPos):
        endPos[1] += effR
        org = np.abs(np.linalg.norm(endPos - fMotor))
        if org >= forearm + bicep:
            #print("not reachable2")
            return None
        proj = np.array([0, endPos[1], endPos[2]])
        dis = np.abs(np.linalg.norm(proj - fMotor))
        projForearm = math.sqrt(forearm ** 2 - endPos[0] ** 2)
        xNum = projForearm ** 2 - bicep ** 2 - dis ** 2
        xDen = -2 * bicep * dis
        temp = xNum / xDen
        inner = math.degrees(math.acos(temp))
        diffY = fMotor[1] - endPos[1]
        phi = math.degrees(math.acos(diffY / dis))
        theta = 180 - inner - phi
        return round(theta,3)

    cos120 = math.cos(math.radians(120))
    sin120 = math.sin(math.radians(120))

    cos240 = math.cos(math.radians(240))
    sin240 = math.sin(math.radians(240))

    f = inverseKinematicsHelper(np.array([x, y, z]))
    r = inverseKinematicsHelper(np.array([(x * cos120) - (y * sin120), (x * sin120 + y * cos120), z]))
    l = inverseKinematicsHelper(np.array([(x * cos240) - (y * sin240), (x * sin240 + y * cos240), z]))
    if f is None or r is None or l is None:
        return None
    return np.array([f, r, l])


# given front, right, and left angles, gives the effector's position
def forwardKinematics(front, right, left):
    front = math.radians(front)
    right = math.radians(right)
    left = math.radians(left)

    frontLen = math.cos(front) * bicep - effR
    fy = frontLen
    fz = -1 * math.sin(front) * bicep
    f = [0, fy, fz]
    f = [f[i] + fMotor[i] for i in range(3)]
    f = np.array(f)

    rightLen = math.cos(right) * bicep - effR
    rx = math.cos(math.radians(30)) * rightLen
    ry = -rightLen / 2
    rz = -1 * math.sin(right) * bicep
    r = (rx, ry, rz)
    r = [r[i] + rMotor[i] for i in range(3)]
    r = np.array(r)

    leftLen = math.cos(left) * bicep - effR
    lx = - math.cos(math.radians(30)) * leftLen
    ly = -leftLen / 2
    lz = -1 * math.sin(left) * bicep
    l = (lx, ly, lz)
    l = [l[i] + lMotor[i] for i in range(3)]
    l = np.array(l)

    def trilaterate(P1, P2, P3, r1, r2, r3):
        temp1 = P2 - P1
        e_x = temp1 / np.linalg.norm(temp1)
        temp2 = P3 - P1
        i = np.dot(e_x, temp2)
        temp3 = temp2 - i * e_x
        e_y = temp3 / np.linalg.norm(temp3)
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(P2 - P1)
        j = np.dot(e_y, temp2)
        x = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
        y = (r1 * r1 - r3 * r3 - 2 * i * x + i * i + j * j) / (2 * j)
        temp4 = r1 * r1 - x * x - y * y
        if temp4 < 0:
            return None
        z = np.sqrt(temp4)
        # p_12_b = P1 + x * e_x + y * e_y - z * e_z
        p_12_b = P1 + x * e_x + y * e_y + z * e_z
        return p_12_b

    point = trilaterate(f, r, l, forearm, forearm, forearm)

    return [round(point[i],2) for i in range(3)]


def getPointsOnCircle(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return np.array([x, y])

def speedTest():
    count = 0
    for i in range (0, 90):
        for j in range(0, 90):
            for k in range(0, 90):
                print(i,j,k)
                predPos = forwardKinematics(i, j, k)
                if(predPos == None):
                    continue
                predAngles = np.array(inverseKinematics(predPos[0], predPos[1], predPos[2]))
                if (predAngles == None).all():
                    continue
                count +=1
                if(i != round(predAngles[0])):
                    print("failed for i", i, j, k, predAngles)
                if (j != round(predAngles[1])):
                    print("failed for i", i, j, k, predAngles)
                if (k != round(predAngles[2])):
                    print("failed for i", i, j, k, predAngles)
    print("all passed", count)
