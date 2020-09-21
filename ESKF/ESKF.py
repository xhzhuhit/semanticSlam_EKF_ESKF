import csv
import copy
import numpy as np
import math
import matplotlib.pyplot as plt
from funcs.trans import ppprint, euler2q, q2R, from_axis_angle, q2euler, euler2skewSymmetric, qUpdateDq
from funcs.initialize_quat_covariance import initialize_quat_covariance
'''
with open('../data/imu20685_3.csv', 'r') as f:
    f_csv = csv.reader(f)
    data = np.array(f_csv, dtype=np.float64)
    for row in f_csv:
        print row
'''

DEG_TO_RAD_DOUBLE = 3.1415926 / 180.0
gps_buf = 10
'''
read data
'''
data = np.loadtxt(open('../data/imu20685_3.csv', 'rb'), delimiter=',', skiprows= 0)
timestamp = data[:, 0]
acc = data[:, 21:24]
gyro = data[:, 18:21]
gps = data[:, 24:30] #3,4,5 notes x,y,z of pose
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
vision_pos = data[:, 30:32] #notes x,y,z of pose vision
'''
init eskf parameters
'''
x_nom = np.array(np.zeros(16)) #[p v q a_b w_b ]    //maybe g
x_err = np.array(np.zeros(15)) #[delta_p delta_v delta_theta delta_a_b  delta_w_b]  //maybe delta_g


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
filter_init = 0
predictState = 1

len_data = len(data)
#data for plot
fusion_pose = copy.deepcopy(gps[:, 3:6])
fusion_speed = copy.deepcopy(data[:, 0:3])
fusion_heading = copy.deepcopy(vision_heading)
#len_data = 6000
for i in range(gps_buf, len_data):
    #print math.sqrt(pow(acc[i][0],2) + pow(acc[i][1],2) + pow(acc[i][2],2))
    if not filter_init:
        pitch = math.asin(-np.mean(acc[100:150,0]) / 9.80665)
        roll = math.atan2(np.mean(acc[100:150,1]), np.mean(acc[100:150, 2]))
        yaw = gps_heading[i]
        x_nom[6:10] = euler2q([roll, pitch, yaw])
        rpy = q2euler(x_nom[6:10])
        print "yaw = %f, rpy = %f, %f, %f" %(yaw, rpy[0], rpy[1], rpy[2])
        #print x[0:4]
        P = np.diag([0.25, 0.25, 0.25, 0.01, 0.01, 0.01, 0, 0, 0, 
            pow(2.0*DEG_TO_RAD_DOUBLE,2), pow(2.0*DEG_TO_RAD_DOUBLE,2), pow(2.0*DEG_TO_RAD_DOUBLE,2), 
            pow(3.0e-3,2), pow(3.0e-3,2), pow(3.0e-3,2)])
        P[6:9,6:9] = np.diag([1e-4, 1e-4, 1e-4]) #P for err_state, use euler not quaternion.
        filter_init = 1
        x_nom[3:6] = v[i]
        x_nom[0:3] = gps[i,3:6]
        #a_b and w_b is 0, so do not do init
    #end fi filter_init
    dt = (timestamp[i] - timestamp[i-1]) / 1e6

    if predictState == 1:
        #predict nominate state
        delVel = (acc[i,:].transpose() - x_nom[10:13]) * dt
        delAng = (gyro[i,:].transpose() - x_nom[13:16]) * dt
        Cbn = q2R(x_nom[6:10])
        delVel = np.dot(Cbn, delVel) - np.array([0, 0, 9.80665*dt])
        dq = from_axis_angle(delAng)
        x_nom[6:10] = [x_nom[6] * dq[0] - x_nom[7] * dq[1] - x_nom[8] * dq[2] - x_nom[9] * dq[3],
                x_nom[6] * dq[1] + x_nom[7] * dq[0] + x_nom[8] * dq[3] - x_nom[9] * dq[2],
                x_nom[6] * dq[2] - x_nom[7] * dq[3] + x_nom[8] * dq[0] + x_nom[9] * dq[1],
                x_nom[6] * dq[3] + x_nom[7] * dq[2] - x_nom[8] * dq[1] + x_nom[9] * dq[0]]
        x_nom[6:10] = x_nom[6:10] / np.linalg.norm(x_nom[6:10])

        Vel_pre = x_nom[3:6].copy()
        x_nom[3:6] = x_nom[3:6] + delVel
        x_nom[0:3] = x_nom[0:3] + (x_nom[3:6] + Vel_pre) * dt/2
        #predict error state
        #under predict is wrong, 267 in book is simply update
        #x_err[9:12] = x_err[9:12]
        #x_err[12:15] = x_err[12:15]
        #R = q2R(dq)
        #x_err[6:9] = np.dot(R.transpose(), x_err[6:9]) - x_err[12:15] * dt
        #x_err[3:6] = x_err[3:6] + (- np.dot(tmp_Rx, x_err[6:9]) - np.dot(Cbn, x_err[9:12])) * dt
        #x_err[0:3] = x_err[0:3] + x_err[3:6] * dt
        #end predict, then need to update P and Q
        tmp_Rx = np.dot(Cbn, euler2skewSymmetric(acc[i,:].transpose() - x_nom[10:13]))
        delta_theta = (gyro[i,:].transpose() - x_nom[13:16]) * dt
        tmp_Rr = q2R(euler2q(delta_theta))

        FF = np.zeros((15,15))
        FF[0:3,0:3] = np.eye(3)
        FF[0:3,3:6] = np.eye(3) * dt
        FF[3:6,3:6] = np.eye(3)
        FF[3:6,6:9] = - tmp_Rx * dt
        FF[3:6,9:12] = - Cbn * dt
        FF[6:9,6:9] = tmp_Rr.transpose()
        FF[6:9,12:15] = - np.eye(3) * dt
        FF[9:12,9:12] = np.eye(3)
        FF[12:15,12:15] = np.eye(3)

        #do error state predict
        x_err = np.dot(FF, x_err)

        F= np.array(FF)
        #print F.shape
        Q = np.diag([gyrox_noise, gyroy_noise, gyroz_noise, accx_noise, accy_noise, accz_noise,gyrox_bias_noise, gyroy_bias_noise, gyroz_bias_noise, accx_bias_noise, accy_bias_noise, accz_bias_noise])
        Fi = np.zeros((15,12))
        #Fi[0:3,3:6] = np.eye(3) * dt
        Fi[3:15,0:12] = np.eye(12)
        Fi = np.array(Fi)
        Q = np.dot(np.dot(Fi, Q), Fi.transpose())
        P = np.dot(np.dot(F, P), F.transpose()) + Q

    #fusion_pose.append(x[0:10])
    fusion_pose[i,:]= x_nom[0:3]
    fusion_speed[i,:] = x_nom[3:6]
    rpy = q2euler(x_nom[6:10])
    fusion_heading[i] = rpy[2]

#2019-11-24 20:09
    #if have gps data, then do follow, else just say continue to return to the begin
    #if gps_heading[i] != gps_heading[i-1]:
    #use gps data: speed_n speed_e speed_d x y yaw
    #calc H
    if math.fabs(gps[i,3] - gps[i-1,3]) > 0.000001:
        H_x = np.zeros((15,16))
        H_x[0,0] = 1
        H_x[1,1] = 1
        H_x[3,3] = 1
        H_x[4,4] = 1
        H_x[5,5] = 1

        euler = q2euler(x_nom[6:10])
        t1 = 2*(x_nom[6]*x_nom[9] + x_nom[7]*x_nom[8])
        t2 = pow(x_nom[6],2) + pow(x_nom[7],2) - pow(x_nom[8],2) - pow(x_nom[9],2)
        t3 = t1 / t2
        t4 = 1 / (1 + pow(t3,2))
        t5 = pow(t2,2)
        t6 = t4 / t5
        H_x[8,6:10] = [t6*(2*x_nom[9]*t2 - 2*x_nom[6]*t1), t6*(2*x_nom[8]*t2 - 2*x_nom[7]*t1), 
                        t6*(2*x_nom[7]*t2 + 2*x_nom[8]*t1), t6*(2*x_nom[6]*t2 + 2*x_nom[9]*t1)]

        #calc X_dx
        X_dx = np.zeros((16,15))
        X_dx[0:6,0:6] = np.eye(6)
        X_dx[10:16,9:15] = np.eye(6)
        qw,qx,qy,qz = x_nom[6:10]
        
        mm = [[-qx, -qy, -qz], 
              [ qw, -qz,  qy], 
              [ qz,  qw, -qx], 
              [-qy,  qx,  qw]]
        X_dx[6:10,6:9] = 0.5 * np.array(mm) 
        H = np.dot(H_x, X_dx)
        
        # calc K, update P, and calc delta_x
        V = np.eye(15) * 1e-9
        nom = np.dot(P,H.transpose())
        denom = np.dot(np.dot(H,P), H.transpose()) + V
        K = np.dot(nom, np.linalg.pinv(denom))
        y = [gps[i,3], gps[i,4], 0, gps[i,0], gps[i,1], gps[i,2], 0, 0, gps_heading[i], 0,0,0,0,0,0]
        rpy0 = q2euler(x_nom[6:10])
        h_xv = [x_nom[0], x_nom[1], 0, x_nom[3], x_nom[4], x_nom[5], 0, 0, rpy0[2], 0,0,0,0,0,0]
        a = np.array(y) - np.array(h_xv)
        print y
        print h_xv
        print "minus residual:"
        print a
        print K
        delta_x = np.dot(K, np.array(y) - np.array(h_xv))
    
        gps_correct = 1
        if gps_correct:
            #do update, injection of observed error into nominal state
            print delta_x
            x_nom[0:6] += delta_x[0:6]
            delta_q = euler2q(delta_x[6:9])
            #print delta_q
            drpy1 = q2euler(x_nom[6:10])
            x_nom[6:10] = qUpdateDq(x_nom[6:10], delta_q)
            x_nom[6:10] = x_nom[6:10] / np.linalg.norm(x_nom[6:10])
            drpy2 = q2euler(x_nom[6:10])
            delta_drpy = np.array(drpy2) - np.array(drpy1)
            if delta_drpy[2] > 0.1:
                print "Divergence !"
            print drpy1
            print drpy2

            #do ESKF reset
            '''
            G = np.eye(15)
            G[0:6,0:6] = np.eye(6)
            G[9:15,9:15] = np.eye(6)
            G[6:9,6:9] = np.eye(3) - euler2skewSymmetric(0.5*delta_x[6:9]) 
            P = np.dot(np.dot(G,P), G.transpose())
            '''

lmin = 10
lmax = 90000 #2000
plt.plot([gps[i,3] for i in range(lmin,lmax)], [gps[i,4] for i in range(lmin,lmax)], '-')
plt.plot([fusion_pose[i,0] for i in range(lmin,lmax)], [fusion_pose[i,1] for i in range(lmin,lmax)], '-r')
plt.legend(['gps_position', 'fusion_position'])
plt.savefig("../pic/position_eskf.png")
plt.close()

plt.plot([i for i in range(lmin,lmax)],[v[i,0] for i in range(lmin,lmax)],'-')
plt.plot([i for i in range(lmin,lmax)],[fusion_speed[i,0] for i in range(lmin,lmax)],'-r')
plt.plot([i for i in range(lmin,lmax)],[v[i,1] for i in range(lmin,lmax)],'-')
plt.plot([i for i in range(lmin,lmax)],[fusion_speed[i,1] for i in range(lmin,lmax)],'-r')
plt.legend(['x_speed_raw', 'x_speed_fuse', 'y_speed_raw', 'y_speed_fuse'])
plt.savefig("../pic/speed_eskf.png")
plt.close()

plt.plot([i for i in range(lmin,lmax)],[fusion_heading[i] for i in range(lmin,lmax)],'-r')
plt.plot([i for i in range(lmin,lmax)],[vision_heading[i] for i in range(lmin,lmax)],'-b')
plt.plot([i for i in range(lmin,lmax)],[gps_heading[i] for i in range(lmin,lmax)],'-y')
plt.legend(['fusion_heading', 'vision_heading', 'gps_heading'])
plt.savefig("../pic/yaw_eskf.png")
plt.close()
#for n in range(10,50):
#    print "%f   %f" %(gps[n,3],gps[n,4])
#    print "%f   %f" %(fusion_pose[n,0], fusion_pose[n,1])



