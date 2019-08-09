import csv
import copy
import numpy as np
import math
import matplotlib.pyplot as plt
from funcs.trans import ppprint, euler2q, q2R, from_axis_angle, q2euler
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
#data for plot
fusion_pose = copy.deepcopy(gps[:, 3:6])
fusion_speed = copy.deepcopy(data[:, 0:3])
fusion_heading = copy.deepcopy(vision_heading)
#ppprint(gps)
len_data = 6000
for i in range(gps_buf, len_data):
    #print math.sqrt(pow(acc[i][0],2) + pow(acc[i][1],2) + pow(acc[i][2],2))
    if not filter_init:
        pitch = math.asin(-np.mean(acc[100:150,0]) / 9.80665)
        roll = math.atan2(np.mean(acc[100:150,1]), np.mean(acc[100:150, 2]))
        yaw = gps_heading[i]
        x[0:4] = euler2q([roll, pitch, yaw])
        rpy = q2euler(x[0:4])
        print "yaw = %f, rpy = %f, %f, %f" %(yaw, rpy[0], rpy[1], rpy[2])
        #print x[0:4]
        p = np.diag([0, 0, 0, 0, 0.01, 0.01, 0.01, 4, 4, 4, 
            pow(2.0*DEG_TO_RAD_DOUBLE,2), pow(2.0*DEG_TO_RAD_DOUBLE,2), pow(2.0*DEG_TO_RAD_DOUBLE,2), 
            pow(3.0e-3,2), pow(3.0e-3,2), pow(3.0e-3,2)])
        rotation_vector_variance = [pow(3.0 * DEG_TO_RAD_DOUBLE, 2), pow(3.0 * DEG_TO_RAD_DOUBLE, 2), pow(3.0 * DEG_TO_RAD_DOUBLE, 2)]
        p[0:4,0:4] = np.diag([1e-4, 1e-4, 1e-4, 1e-4])
        #p[0:4,0:4] = initialize_quat_covariance(x[0:4], rotation_vector_variance)
        filter_init = 1
        x[4:7] = v[i]
        x[7:10] = gps[i,3:6]
    #end fi filter_init
    dt = (timestamp[i] - timestamp[i-1]) / 1e6

    if predictState == 1:
        delAng = (gyro[i,:].transpose() - x[10:13]) * dt
        delVel = (acc[i,:].transpose() - x[13:16]) * dt
        Cbn = q2R(x[0:4])
        delVel = np.dot(Cbn, delVel) - np.array([0, 0, 9.80665*dt])
        dq = from_axis_angle(delAng)
        x[0:4] = [x[0] * dq[0] - x[1] * dq[1] - x[2] * dq[2] - x[3] * dq[3],
                x[0] * dq[1] + x[1] * dq[0] + x[2] * dq[3] - x[3] * dq[2],
                x[0] * dq[2] - x[1] * dq[3] + x[2] * dq[0] + x[3] * dq[1],
                x[0] * dq[3] + x[1] * dq[2] - x[2] * dq[1] + x[3] * dq[0]]
        x[0:4] = x[0:4] / np.linalg.norm(x[0:4])

        Vel_pre = x[4:7]
        x[4:7] = x[4:7] + delVel
        x[7:10] = x[7:10] + (x[4:7] + Vel_pre) * dt/2
    #end predict, then need to update P and Q
    FF = [[1, -(gyro[i,0]-x[10])*dt/2, -(gyro[i,1]-x[11])*dt/2, -(gyro[i,2]-x[12])*dt/2, 0, 0, 0, 0, 0, 0, 0.5*x[1]*dt, 0.5*x[2]*dt, 0.5*x[3]*dt, 0, 0, 0],
         [(gyro[i,0]-x[10])*dt/2, 1, (gyro[i,2]-x[12])*dt/2, -(gyro[i,1]-x[11])*dt/2, 0, 0, 0, 0, 0, 0, -0.5*x[0]*dt, 0.5*x[3]*dt, -0.5*x[2]*dt, 0, 0, 0],
         [(gyro[i,1]-x[11])*dt/2, -(gyro[i,2]-x[12])*dt/2, 1, (gyro[i,0]-x[10])*dt/2, 0, 0, 0, 0, 0, 0, -0.5*x[3]*dt, -0.5*x[0]*dt, 0.5*x[1]*dt, 0, 0, 0],
         [(gyro[i,2]-x[12])*dt/2, (gyro[i,1]-x[11])*dt/2, -(gyro[i,0]-x[10])*dt/2, 1, 0, 0, 0, 0, 0, 0, 0.5*x[2]*dt, -0.5*x[1]*dt, -0.5*x[0]*dt, 0, 0, 0],

         [(2*x[0]*(acc[i,0]-x[13])-2*x[3]*(acc[i,1]-x[14])+2*x[2]*(acc[i,2]-x[15]))*dt, (2*x[1]*(acc[i,0]-x[13])+2*x[2]*(acc[i,1]-x[14])+2*x[3]*(acc[i,2]-x[15]))*dt, (-2*x[2]*(acc[i,0]-x[13])+2*x[1]*(acc[i,1]-x[14])+2*x[0]*(acc[i,2]-x[15]))*dt, (-2*x[3]*(acc[i,0]-x[13])-2*x[0]*(acc[i,1]-x[14])+2*x[1]*(acc[i,2]-x[15]))*dt, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         -(pow(x[0],2)+pow(x[1],2)-pow(x[2],2)-pow(x[3],2))*dt, -2*(x[1]*x[2]-x[0]*x[3])*dt, -2*(x[1]*x[3]+x[0]*x[2])*dt],

         [(2*x[3]*(acc[i,0]-x[13])+2*x[0]*(acc[i,1]-x[14])-2*x[1]*(acc[i,2]-x[15]))*dt, (2*x[2]*(acc[i,0]-x[13])-2*x[1]*(acc[i,1]-x[14])-2*x[0]*(acc[i,2]-x[15]))*dt, (2*x[1]*(acc[i,0]-x[13])+2*x[2]*(acc[i,1]-x[14])+2*x[3]*(acc[i,2]-x[15]))*dt, (2*x[0]*(acc[i,0]-x[13])-2*x[3]*(acc[i,1]-x[14])+2*x[2]*(acc[i,2]-x[15]))*dt, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         -2*(x[1]*x[2]+x[0]*x[3])*dt, -(pow(x[0],2)-pow(x[1],2)+pow(x[2],2)-pow(x[3],2))*dt, -2*(x[3]*x[2]-x[0]*x[1])*dt],

         [(-2*x[2]*(acc[i,0]-x[13])+2*x[1]*(acc[i,1]-x[14])+2*x[0]*(acc[i,2]-x[15]))*dt, (2*x[3]*(acc[i,0]-x[13])+2*x[0]*(acc[i,1]-x[14])-2*x[1]*(acc[i,2]-x[15]))*dt, (-2*x[0]*(acc[i,0]-x[13])+2*x[3]*(acc[i,1]-x[14])-2*x[2]*(acc[i,2]-x[15]))*dt, (2*x[1]*(acc[i,0]-x[13])+2*x[2]*(acc[i,1]-x[14])+2*x[3]*(acc[i,2]-x[15]))*dt, 0, 0, 1, 0, 0, 0, 0, 0, 0,
         -2*(x[1]*x[3]-x[0]*x[2])*dt, -2*(x[3]*x[2]+x[0]*x[1])*dt, -(pow(x[0],2)-pow(x[1],2)-pow(x[2],2)+pow(x[3],2))*dt],

         [0, 0, 0, 0, dt, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, dt, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, dt, 0, 0, 1, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]

    F= np.array(FF)
    #print F.shape
    Q = np.diag([gyrox_noise, gyroy_noise, gyroz_noise,accx_noise, accy_noise, accz_noise,gyrox_bias_noise, gyroy_bias_noise, gyroz_bias_noise,accx_bias_noise, accy_bias_noise, accz_bias_noise])
    G =[[(dt *x[1]) / 2, (dt *x[2]) / 2, (dt *x[3]) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-(dt *x[0]) / 2, (dt *x[3]) / 2,  -(dt *x[2]) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-(dt *x[3]) / 2,  -(dt *x[0]) / 2, (dt *x[1]) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [(dt *x[2]) / 2,  -(dt *x[1]) / 2,  -(dt *x[0]) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -(pow(x[0],2)+pow(x[1],2)-pow(x[2],2)-pow(x[3],2))*dt,-2*(x[1]*x[2]-x[0]*x[3])*dt,-2*(x[1]*x[3]+x[0]*x[2])*dt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -2*(x[1]*x[2]+x[0]*x[3])*dt,-(pow(x[0],2)-pow(x[1],2)+pow(x[2],2)-pow(x[3],2))*dt,-2*(x[3]*x[2]-x[0]*x[1])*dt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, -2*(x[1]*x[3]-x[0]*x[2])*dt,-2*(x[3]*x[2]+x[0]*x[1])*dt,-(pow(x[0],2)-pow(x[1],2)-pow(x[2],2)+pow(x[3],2))*dt, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, dt, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt]]
    G = np.array(G)
    Q = np.dot(np.dot(G, Q), G.transpose())
    p = np.dot(np.dot(F, p), F.transpose()) + Q

    #fusion_pose.append(x[0:10])
    fusion_pose[i,:]= x[7:10]
    fusion_speed[i,:] = x[4:7]
    rpy = q2euler(x[0:4])
    fusion_heading[i] = rpy[2]

    gps_heading_correct = 1
    if gps_heading_correct:
        if gps_heading[i] != gps_heading[i-1]:
            euler = q2euler(x[0:4])
            predict_yaw = euler[2]
            measure_yaw = gps_heading[i]
            #below look as yaw in q2culer, make partial-diff to quarternion, 
            #can be seen as,while yaw got correction, what correction should on quarternion.
            t1 = 2*(x[0]*x[3] + x[1]*x[2])
            t2 = pow(x[0],2) + pow(x[1],2) - pow(x[2],2) - pow(x[3],2)
            t3 = t1 / t2
            t4 = 1 / (1 + pow(t3,2))
            t5 = pow(t2,2)
            t6 = t4 / t5
            H_gps_heading = np.array([t6*(2*x[3]*t2 - 2*x[0]*t1), t6*(2*x[2]*t2 - 2*x[1]*t1), 
                    t6*(2*x[1]*t2 + 2*x[2]*t1), t6*(2*x[0]*t2 + 2*x[3]*t1),
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            r_gps_heading = measure_yaw - predict_yaw
            if r_gps_heading > math.pi:
                r_gps_heading = r_gps_heading - 2 * math.pi
            elif r_gps_heading < - math.pi:
                r_gps_heading = r_gps_heading + 2 * math.pi
            R_mag = pow(5.0 / vehicle_speed[i] * DEG_TO_RAD_DOUBLE, 2)
            print "R_mag: "
            print p
            print H_gps_heading
            si_mag = 1.0 / (np.dot(np.dot(H_gps_heading, p), H_gps_heading.transpose()) + R_mag) #pinv need 2 dim
            beta_mag = pow(r_gps_heading,2) * si_mag
            print "r_gps_heading = %f, si+mag = %f" %(r_gps_heading, si_mag)
            if 1:
            #if beta_mag < 20000000:
                k_mag = np.dot(np.dot(p, H_gps_heading.transpose()), si_mag)
                correction = k_mag * r_gps_heading
                print correction
                #correction[4:10] = [0,0,0,0,0,0]
                #correction[13:16] = [0,0,0]
                x = x + correction
                p = np.dot(np.eye(16)-np.dot(k_mag, H_gps_heading), p)
                x[0:4] = x[0:4] / np.linalg.norm(x[0:4])
    
    #use gps speed
    gps_correct = 0
    if gps_correct:
        if gps[i,0] != gps[i-1,0]:
        #    print '1'
        #else:
            #euler = q2euler(x[0:4])
            R_gps = np.diag([pow(gps_s_var[i],2), pow(gps_s_var[i],2), pow(gps_s_var[i],2)])
            z_gps = gps[i, 0:3]
            x_predict = x
            r_gps = z_gps - np.dot(H_gps, x_predict)
            #print r_gps
            #print p
            #print (np.dot(np.dot(H_gps, p), H_gps.transpose()) + R_gps)
            si_gps = np.linalg.pinv(np.dot(np.dot(H_gps, p), H_gps.transpose()) + R_gps)
            #print z_gps
            #print np.dot(H_gps, x_predict)
            #print r_gps
            beta_gps = np.dot(np.dot(r_gps.transpose(), si_gps), r_gps)
            #if beta_gps < 20000000:
            if 1:
                k_gps = np.dot(np.dot(p, H_gps.transpose()), si_gps)
                correction = np.dot(k_gps, r_gps)
                #print correction
                #[wrong] p is not diagnoal matrix, so update should not use first kind which regard p as diagnoal, 
                #should use second in book-QinYongYuan
                #correction[0:4] = [0, 0, 0, 0]
                #correction[6] = 0
                #correction[10:13] = [0, 0, 0]
                x = x + correction
                p = np.dot(np.eye(16)-np.dot(k_gps, H_gps),p)
                x[0:4] = x[0:4] / np.linalg.norm(x[0:4])
lmin = 10
lmax = 500
plt.plot([gps[i,3] for i in range(lmin,lmax)], [gps[i,4] for i in range(lmin,lmax)], '-')
plt.plot([fusion_pose[i,0] for i in range(lmin,lmax)], [fusion_pose[i,1] for i in range(lmin,lmax)], '-r')
plt.legend(['gps_position', 'fusion_position'])
plt.savefig("../pic/position.png")
plt.close()

plt.plot([i for i in range(lmin,lmax)],[v[i,0] for i in range(lmin,lmax)],'-')
plt.plot([i for i in range(lmin,lmax)],[fusion_speed[i,0] for i in range(lmin,lmax)],'-r')
plt.plot([i for i in range(lmin,lmax)],[v[i,1] for i in range(lmin,lmax)],'-')
plt.plot([i for i in range(lmin,lmax)],[fusion_speed[i,1] for i in range(lmin,lmax)],'-r')
plt.legend(['x_speed_raw', 'x_speed_fuse', 'y_speed_raw', 'y_speed_fuse'])
plt.savefig("../pic/speed.png")
plt.close()

plt.plot([i for i in range(lmin,lmax)],[fusion_heading[i] for i in range(lmin,lmax)],'-r')
plt.plot([i for i in range(lmin,lmax)],[vision_heading[i] for i in range(lmin,lmax)],'-b')
plt.plot([i for i in range(lmin,lmax)],[gps_heading[i] for i in range(lmin,lmax)],'-y')
plt.legend(['fusion_heading', 'vision_heading', 'gps_heading'])
plt.savefig("../pic/yaw.png")
plt.close()
#for n in range(10,50):
#    print "%f   %f" %(gps[n,3],gps[n,4])
#    print "%f   %f" %(fusion_pose[n,0], fusion_pose[n,1])



