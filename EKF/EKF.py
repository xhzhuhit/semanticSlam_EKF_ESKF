import csv
import numpy as np
import math
'''
with open('../data/imu20685_3.csv', 'r') as f:
    f_csv = csv.reader(f)
    data = np.array(f_csv, dtype=np.float64)
    for row in f_csv:
        print row
'''
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

DEG_TO_RAD_DOUBLE = 3.1415926 / 180.0
gps_buf = 10
'''
read data
'''
data = np.loadtxt(open('../data/imu20685_3.csv', 'rb'), delimiter=',', skiprows= 0)
acc = data[:, 21:24]
gyro = data[:, 18:21]
gps = data[:, 24:30]
gps_heading = data[:, 29]
vision_heading = data[:, 32]
gps_s_var = data[:, 33]
vehicle_speed = data[:, 35]

att = data[:, 2:5] * 57.3
v = data[:,5:8]
pos = data[:, 8:11]
bias = data[:, 11:17]
gps_vel2d = data[:, 34]
gps_speed = [math.sqrt(pow(gps[i,0],2) + pow(gps[i,1],2) + pow(gps[i,2],2)) for i in range(len(data))]
vision_pos = data[:, 30:32]
'''
init kf parameters
'''
x = np.array(np.zeros(16))
x[0] = 1.0
H_zp = np.zeros([2,16])#for zero speed modification zupt
H_zp[0][7] = 1
H_zp[1][8] = 1
H_gps = np.zeros([3,16])#guance matrix
H_gps[0][4] = 1
H_gps[1][5] = 1
H_gps[2][6] = 1

R_baro = 9
R_zp = np.array([[1, 0],[0, 1]])
R_speed = pow(0.5,2)
gyrox_bias_noise = pow((5e-3 * DEG_TO_RAD_DOUBLE),2)
gyroy_bias_noise = pow((5e-3 * DEG_TO_RAD_DOUBLE),2)
gyroz_bias_noise = pow((5e-3 * DEG_TO_RAD_DOUBLE),2)
gyrox_noise = pow((1e-3 * DEG_TO_RAD_DOUBLE),2)
gyroy_noise = pow((1e-3 * DEG_TO_RAD_DOUBLE),2)
gyroz_noise = pow((1e-3 * DEG_TO_RAD_DOUBLE),2)
accx_bias_noise = pow((1e-19),2)
accy_bias_noise = pow((1e-19),2)
accz_bias_noise = pow((1e-5),2)
accx_noise = pow((1e-4),2)
accy_noise = pow((1e-4),2)
accz_noise = pow((1e-0),2)

#flags
gps_correct = 1
gps_heading_correct = 1
vehicle_speed_correct = 0
land_correct = 0
zp_correct = 0
gps_init = 0
filter_init = 0
predictState = 1

len_data = len(data)
for i in range(gps_buf, len_data-gps_buf):
    #print math.sqrt(pow(acc[i][0],2) + pow(acc[i][1],2) + pow(acc[i][2],2))
    if not filter_init:
        pitch = math.asin(-np.mean(acc[100:150,0]) / 9.80665)
        roll = math.atan(np.mean(acc[100:150,1]) / np.mean(acc[100:150, 2]))
        yaw = gps_heading[i]
        x[0:4] = euler2q([roll, pitch, yaw])
        print x[0:4]
