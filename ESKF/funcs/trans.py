import math
import numpy as np


def ppprint(data):
    for i in range(len(data)):
        print data[i]
def euler2q(euler):
    q = [0, 0, 0, 0]
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    q[0] = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(yaw/2)*math.sin(pitch/2)
    q[1] = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(yaw/2)*math.sin(pitch/2)
    q[2] = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(yaw/2)*math.sin(pitch/2)
    q[3] = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(yaw/2)*math.cos(pitch/2)
    return q/np.linalg.norm(q)

def q2R(q):
    R = np.zeros((3,3))
    R[0,0] = pow(q[0],2) + pow(q[1],2) - pow(q[2],2) - pow(q[3],2)
    R[0,1] = 2*(q[1] * q[2] - q[0] * q[3])
    R[0,2] = 2*(q[1] * q[3] + q[0] * q[2])

    R[1,0] = 2*(q[1] * q[2] + q[0] * q[3])
    R[1,1] = pow(q[0],2) - pow(q[1],2) + pow(q[2],2) - pow(q[3],2)
    R[1,2] = 2*(q[2] * q[3] - q[0] * q[1])
    
    R[2,0] = 2*(q[1] * q[3] - q[0] * q[2])
    R[2,1] = 2*(q[2] * q[3] + q[0] * q[1])
    R[2,2] = pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2)
    return R

def from_axis_angle(vec):
    q = np.zeros(4)
    theta = np.linalg.norm(vec)
    if (theta < 1e-10):
        q[0] = 1.0
        q[1] = 0.
        q[2] = 0.
        q[3] = 0.
    else:
        vec = vec / theta
        magnitude = math.sin(theta / 2.0)
        q[0] = math.cos(theta / 2.0)
        q[1] = vec[0] * magnitude
        q[2] = vec[1] * magnitude
        q[3] = vec[2] * magnitude

    return q

def q2euler(q):
    euler = [math.atan2(2.0*(q[0]*q[1]+q[2]*q[3]), (1.0-(2.0*(pow(q[1],2)+pow(q[2],2))))),
            math.asin(2.0*(q[0]*q[2]-q[3]*q[1])),
            math.atan2(2.0*(q[0]*q[3]+q[1]*q[2]), (1.0-(2.0*(pow(q[2],2)+pow(q[3],2)))))]
    return euler

def euler2skewSymmetric(euler):
    R = np.zeros((3,3))
    x = euler[0]
    y = euler[1]
    z = euler[2]
    R[0,1] = -z
    R[0,2] = y
    R[1,0] = z
    R[1,2] = -x
    R[2,0] = -y
    R[2,1] = x
    return R
def qUpdateDq(q, dq):
    qr = [q[0] * dq[0] - q[1] * dq[1] - q[2] * dq[2] - q[3] * dq[3],
          q[0] * dq[1] + q[1] * dq[0] + q[2] * dq[3] - q[3] * dq[2],
          q[0] * dq[2] - q[1] * dq[3] + q[2] * dq[0] + q[3] * dq[1],
          q[0] * dq[3] + q[1] * dq[2] - q[2] * dq[1] + q[3] * dq[0]]
    return qr
    
