%% ****EKalmanFilter--16states*******%
%Author:ZhangJianxu
%Date:20181225
%% *****model description******************%
%states:q(4)\velocity(3-enu)\position(3-lon/lat/h or Penu)\gyro bias(3-body frame)\acc bias(3-body frame)=16
% ---------------state equation------------------------
%|Qk = Qk-1 + 1/2*M(q)*(w_b - bias_k - w)*dt;
%|Vk = Vk-1 + Cbn*(fb - bias_k - w)*dt;
%|Xk = Xk-1 + Vk*dt;/lon+=ve/(Rn*cos(lat))*dt\lat+=vn/Rm*dt\h+=vu*dt;
%|bias_k = bias_k-1;
% -----------------------------------------------------
% -------------measurement-----------------------------
%|GpsHeadingCorrect: gps_heading = atan(q);
%|GpsVel/PosCorrect: gps_vel = Vk;gps_pos = Xk;
%|VehicleSpeedCorrect: vehicle_speed = 1/Vk_norm*Vk;
%|VisionHeadingCorrect: vision_heading = atan(q);
%|VisionPosCorrect:  vision_pos = Xk;
% -----------------------------------------------------
% -------------------ordinate---------------------------
% body frame:front-left-up  navigation frame:east-north-up
%         up  front        up   north
%         |  /              |  /
%         | /               | /
%left_____|/                |/______east
% -----------------------------------------------------
% -----------------------------------------------------
%Cbn = [1.0-2.0*(q3q3 + q4q4)  2.0*(q2q3 - q1q4)  2.0*(q2q4 + q1q3);
%       2.0*(q2q3 + q1q4)  1.0-2.0*(q2q2 + q4q4)  2.0*(q3q4 - q1q2);
%       2.0*(q2q4 - q1q3)  2.0*(q3q4 + q1q2)  1.0-2.0*(q2q2 + q3q3)]
%
% cr2 = cos(roll*0.5); cp2 = cos(pitch*0.5);cy2 = cos(yaw*0.5);
% sr2 = sin(roll*0.5);sp2 = sin(pitch*0.5);sy2 = sin(yaw*0.5);
% q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
% q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
% q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
% q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
% s1 = sin(psi)
%     c1 = cos(psi)
%     s2 = sin(theta)
%     c2 = cos(theta)
%     s3 = sin(phi)
%     c3 = cos(phi)
%[c1*c2, c1*s2*s3 - s1*c3, c1*s2*c3 + s1*s3],
%        [s1*c2, s1*s2*s3 + c1*c3, s1*s2*c3 - c1*s3],
%        [-s2, c2*s3, c2*c3],
% -----------------------------------------------------

%%
% clc;
clear all;
% close all;
set(0,'DefaultFigureWindowStyle','docked');

% path_default = '/home/nvidia/workspace/gps_check/';
% list = dir('data*');
% list_len = length(list);

gps_buf = 10;
count = 1;

% for index = 6:list_len
%% Data loading and preprocess
% count = 1;    
% foldername = list(index).name;
% DataPath = strcat(path_default,foldername);
% cd(DataPath);
% if(~exist('data.mat'))    
%     data = load('data.csv');
%     save data.mat data;
% else
    data = load('imu20685_3.csv');
% end
% cd ..
timestamp = data(:,1);
acc = [data(:,22) data(:,23) data(:,24)];
gyro = [data(:,19) data(:,20) data(:,21)];
acc = acc - mean(acc)+[0 0 9.80665];
gyro = gyro - mean(gyro);
gps = data(:,25:30);%gps-(28:29)
gps_heading = data(:,30);
vision_heading = data(:,33);
gps_s_var = data(:,34);
vehicle_speed = data(:,36);

att = data(:,3:5)*57.3;
v = data(:,6:8);
pos = data(:,9:11);%mercator
bias = data(:,12:17);%Gyro+acc bias
gps_vel2d = data(:,35);
gps_speed = sqrt(gps(:,1).^2+gps(:,2).^2+gps(:,3).^2);
vision_pos = data(:,31:32);%mercator

len = length(acc);
%% variables
x_show(:,1:7) = zeros(16,7);
%%  test variables
vx_test(1) = 0;
vy_test(1) = 0;
vz_test(1) = 0;
xx_test(1) = 0;
yy_test(1) = 0;
vxx = 0;
vyy = 0;
vzz = 0;
xx = 0;
yy = 0;
zz = 0;
gps_show(:,1) = zeros(2,1);
beta_speed = 0;
beta_gps = 0;
k_land = [0 0;0 0];
r_land = [0;0];
corr_speed = zeros(16,1);
corr_gps = zeros(16,1);
corr_gps_heading(1) = 0;
k_gps = zeros(16,3);
hpht = zeros(3,3);
%% KalmanFilter Parameters
DEG_TO_RAD_DOUBLE = pi/180;
i_baro = 3;
i_sonar = 3;
i_gps = 3;
i_land = 3;
i_last = 1;

x = [1;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
H_zp = [0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0];

H_gps = [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];
%          0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
%          0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0;
%          0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0];

R_baro = 9;
R_zp = [1 0;0 1];
R_speed = 0.5^2;
gyrox_bias_noise = (5e-3 * DEG_TO_RAD_DOUBLE)^2;
gyroy_bias_noise = (5e-3 * DEG_TO_RAD_DOUBLE)^2;
gyroz_bias_noise = (5e-3 * DEG_TO_RAD_DOUBLE)^2;
gyrox_noise = (1e-3 * DEG_TO_RAD_DOUBLE)^2;
gyroy_noise = (1e-3 * DEG_TO_RAD_DOUBLE)^2;
gyroz_noise = (1e-3 * DEG_TO_RAD_DOUBLE)^2;
accx_bias_noise = (1e-19)^2;
accy_bias_noise = (1e-19)^2;
accz_bias_noise = (1e-5)^2;
accx_noise = (1e-4)^2;
accy_noise = (1e-4)^2;
accz_noise = (1e-0)^2;
%% corr flag
gps_correct = 1;
gps_heading_correct = 1;
vehicle_speed_correct = 0;
land_correct = 0;
zp_correct = 0;
gps_init = 0;
filter_init = 0;
predictState = 1;
%%
for i = 2 : len-gps_buf
%%    initialized
    if(~filter_init)
    %state initialization
        pitch = asin(-mean(acc(100:150,1)) / 9.80665);
        roll = atan2(mean(acc(100:150,2)),mean(acc(100:150,3)));
        yaw = gps_heading(i);
        x(1:4) = euler2q([roll,pitch,yaw]);
        x(5:7) = gps(i+gps_buf,1:3)';
    %covariance initialization
        p = diag([0 0 0 0 0.1^2 0.1^2 0.1^2 2^2 2^2 2^2 ...
            (2.0 * DEG_TO_RAD_DOUBLE)^2, (2.0 * DEG_TO_RAD_DOUBLE)^2, (2.0 * DEG_TO_RAD_DOUBLE)^2 ...
            (3.0e-3)^2, (3.0e-3)^2, (3.0e-3)^2]);
        rotation_vector_variance = [(3.0 * DEG_TO_RAD_DOUBLE)^2;(3.0 * DEG_TO_RAD_DOUBLE)^2;(3.0 * DEG_TO_RAD_DOUBLE)^2];
        p(1:4,1:4) = initialize_quat_covariance(x(1:4),rotation_vector_variance);
%         p = diag([1e-3 1e-3 1e-3 1e-3 1e-2 1e-2 1e-2 0.5 0.5 0 5e-7 5e-7 5e-7 5e-6 5e-6 5e-5]);
        filter_init = 1;
    end
    dt = (timestamp(i) - timestamp(i-1))/1e6;
    dt_test(i) = dt;
    if(dt > 0.05)
%         dt = 0.01;
    end
%% strapdown update        
   if(predictState == 1)
        delAng = (gyro(i,:)' - x(11:13))*dt;
        delVel = (acc(i,:)' - x(14:16))*dt;
        Cbn = q2R(x(1:4));
        delVel = Cbn * delVel - [0;0;9.80665 * dt];
        dq = from_axis_angle(delAng);
        x(1:4) = [x(1) * dq(1) - x(2) * dq(2) - x(3) * dq(3) - x(4) * dq(4);
              x(1) * dq(2) + x(2) * dq(1) + x(3) * dq(4) - x(4) * dq(3);
              x(1) * dq(3) - x(2) * dq(4) + x(3) * dq(1) + x(4) * dq(2);
              x(1) * dq(4) + x(2) * dq(3) - x(3) * dq(2) + x(4) * dq(1)];
        x(1:4) = x(1:4)/norm(x(1:4));  
        
        Vel_pre = x(5:7);
        x(5:7) = x(5:7) + delVel;
%         x(8:10) = x(8:10) + (x(5:7) + Vel_pre) * dt / 2;
   else   
    %    0.5*[x(1) -x(2) -x(3) -x(4);q1x(1) -x(4) x(3);x(3) q3x(1) -x(2);x(4) -x(3) q1x(1)]*[0; wx; wy; wz] 
    %  = [-(x(2)*wx)/2-(x(3)*wy)/2-(x(4)*wz)/2; (x(1)*wx)/2-(x(4)*wy)/2+(x(3)*wz)/2; (x(4)*wx)/2+(x(1)*wy)/2-(x(2)*wz)/2; (x(2)*wy)/2-(x(3)*wx)/2 + (x(1)*wz)/2]
        F = [1 -gyro(i,1)/2*dt -gyro(i,2)/2*dt -gyro(i,3)/2*dt 0 0 0 0 0 0 -1/2*x(2)*dt -1/2*x(3)*dt -1/2*x(4)*dt 0 0 0;
             gyro(i,1)/2*dt 1 gyro(i,3)/2*dt -gyro(i,2)/2*dt 0 0 0 0 0 0 1/2*x(1)*dt -1/2*x(4)*dt 1/2*x(3)*dt 0 0 0;
             gyro(i,2)/2*dt -gyro(i,3)/2*dt 1 gyro(i,1)/2*dt 0 0 0 0 0 0 1/2*x(4)*dt 1/2*x(1)*dt -1/2*x(2)*dt 0 0 0;
             gyro(i,3)/2*dt gyro(i,2)/2*dt -gyro(i,1)/2*dt 1 0 0 0 0 0 0 -1/2*x(3)*dt 1/2*x(2)*dt 1/2*x(1)*dt 0 0 0;
             0 0 0 0 1 0 0 0 0 0 0 0 0 (x(1)^2+x(2)^2-x(3)^2-x(4)^2)*dt 2*(x(2)*x(3)-x(1)*x(4))*dt 2*(x(2)*x(4)+x(1)*x(3))*dt;
             0 0 0 0 0 1 0 0 0 0 0 0 0 2*(x(2)*x(3)+x(1)*x(4))*dt (x(1)^2-x(2)^2+x(3)^2-x(4)^2)*dt 2*(x(4)*x(3)-x(1)*x(2))*dt;
             0 0 0 0 0 0 1 0 0 0 0 0 0 2*(x(2)*x(4)-x(1)*x(3))*dt 2*(x(4)*x(3)+x(1)*x(2))*dt (x(1)^2-x(2)^2-x(3)^2+x(4)^2)*dt;
             0 0 0 0 dt 0 0 1 0 0 0 0 0 0 0 0;
             0 0 0 0 0 dt 0 0 1 0 0 0 0 0 0 0;
             0 0 0 0 0 0 dt 0 0 1 0 0 0 0 0 0;
             0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
             0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;
             0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;
             0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1];

        Cbn = q2R(x(1:4));
        a_n = (Cbn*acc(i,:)')*dt - [0;0;9.80665*dt];
        B = [0;0;0;0;a_n;0;0;0;0;0;0;0;0;0];
        x = F * x + B;
        x(1:4) = x(1:4)/norm(x(1:4));
   end
   rpy = q2euler(x(1:4));
   qq = euler2q([rpy(1),rpy(2),0]);
   Cbn_earth = q2R(qq);
   an_1(i,:) =  Cbn*acc(i,:)';
a_n = (Cbn*acc(i,:)')*dt - [0;0;9.80665*dt];
    a_n_test = Cbn*acc(i,:)';
    vxx = vxx + a_n(1) + x(14)*dt;
    vyy = vyy + a_n(2) + x(15)*dt;
    vzz = vzz + a_n(3) + x(16)*dt;
    xx = xx +  x(3)*dt;
    yy = yy +  x(4)*dt;
    zz = zz +  x(7)*dt;
 %%  Covariance update    
    FF = [1, -(gyro(i,1)-x(11))*dt/2, -(gyro(i,2)-x(12))*dt/2, -(gyro(i,3)-x(13))*dt/2, 0, 0, 0, 0, 0, 0, 1/2*x(2)*dt, 1/2*x(3)*dt, 1/2*x(4)*dt, 0 0 0;
         (gyro(i,1)-x(11))*dt/2, 1, (gyro(i,3)-x(13))*dt/2, -(gyro(i,2)-x(12))*dt/2, 0, 0 0 0 0 0 -1/2*x(1)*dt, 1/2*x(4)*dt, -1/2*x(3)*dt, 0 0 0;
         (gyro(i,2)-x(12))*dt/2, -(gyro(i,3)-x(13))*dt/2, 1 (gyro(i,1)-x(11))*dt/2, 0 0 0 0 0 0 -1/2*x(4)*dt, -1/2*x(1)*dt, 1/2*x(2)*dt, 0 0 0;
         (gyro(i,3)-x(13))*dt/2, (gyro(i,2)-x(12))*dt/2, -(gyro(i,1)-x(11))*dt/2, 1, 0 0 0 0 0 0 1/2*x(3)*dt, -1/2*x(2)*dt, -1/2*x(1)*dt, 0 0 0;
         (2*x(1)*(acc(i,1)-x(14))-2*x(4)*(acc(i,2)-x(15))+2*x(3)*(acc(i,3)-x(16)))*dt, (2*x(2)*(acc(i,1)-x(14))+2*x(3)*(acc(i,2)-x(15))+2*x(4)*(acc(i,3)-x(16)))*dt, (-2*x(3)*(acc(i,1)-x(14))+2*x(2)*(acc(i,2)-x(15))+2*x(1)*(acc(i,3)-x(16)))*dt, (-2*x(4)*(acc(i,1)-x(14))-2*x(1)*(acc(i,2)-x(15))+2*x(2)*(acc(i,3)-x(16)))*dt, 1 0 0 0 0 0 0 0 0 ...
         -(x(1)^2+x(2)^2-x(3)^2-x(4)^2)*dt, -2*(x(2)*x(3)-x(1)*x(4))*dt, -2*(x(2)*x(4)+x(1)*x(3))*dt;
         (2*x(4)*(acc(i,1)-x(14))+2*x(1)*(acc(i,2)-x(15))-2*x(2)*(acc(i,3)-x(16)))*dt, (2*x(3)*(acc(i,1)-x(14))-2*x(2)*(acc(i,2)-x(15))-2*x(1)*(acc(i,3)-x(16)))*dt, (2*x(2)*(acc(i,1)-x(14))+2*x(3)*(acc(i,2)-x(15))+2*x(4)*(acc(i,3)-x(16)))*dt, (2*x(1)*(acc(i,1)-x(14))-2*x(4)*(acc(i,2)-x(15))+2*x(3)*(acc(i,3)-x(16)))*dt 0 1 0 0 0 0 0 0 0 ...
         -2*(x(2)*x(3)+x(1)*x(4))*dt, -(x(1)^2-x(2)^2+x(3)^2-x(4)^2)*dt, -2*(x(4)*x(3)-x(1)*x(2))*dt;
         (-2*x(3)*(acc(i,1)-x(14))+2*x(2)*(acc(i,2)-x(15))+2*x(1)*(acc(i,3)-x(16)))*dt, (2*x(4)*(acc(i,1)-x(14))+2*x(1)*(acc(i,2)-x(15))-2*x(2)*(acc(i,3)-x(16)))*dt, (-2*x(1)*(acc(i,1)-x(14))+2*x(4)*(acc(i,2)-x(15))-2*x(3)*(acc(i,3)-x(16)))*dt, (2*x(2)*(acc(i,1)-x(14))+2*x(3)*(acc(i,2)-x(15))+2*x(4)*(acc(i,3)-x(16)))*dt 0 0 1 0 0 0 0 0 0 ...
         -2*(x(2)*x(4)-x(1)*x(3))*dt, -2*(x(4)*x(3)+x(1)*x(2))*dt, -(x(1)^2-x(2)^2-x(3)^2+x(4)^2)*dt;
         0 0 0 0 dt 0 0 1 0 0 0 0 0 0 0 0;
         0 0 0 0 0 dt 0 0 1 0 0 0 0 0 0 0;
         0 0 0 0 0 0 dt 0 0 1 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0;
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1];
     
    Q = diag([gyrox_noise, gyroy_noise, gyroz_noise,accx_noise, accy_noise, accz_noise,gyrox_bias_noise, gyroy_bias_noise, gyroz_bias_noise,accx_bias_noise, accy_bias_noise, accz_bias_noise]);
    G =[(dt *x(2)) / 2, (dt *x(3)) / 2, (dt *x(4)) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0
        -(dt *x(1)) / 2, (dt *x(4)) / 2,  -(dt *x(3)) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0
        -(dt *x(4)) / 2,  -(dt *x(1)) / 2, (dt *x(2)) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0
        (dt *x(3)) / 2,  -(dt *x(2)) / 2,  -(dt *x(1)) / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0
        0, 0, 0, -(x(1)^2+x(2)^2-x(3)^2-x(4)^2)*dt,-2*(x(2)*x(3)-x(1)*x(4))*dt,-2*(x(2)*x(4)+x(1)*x(3))*dt, 0, 0, 0, 0, 0, 0
        0, 0, 0, -2*(x(2)*x(3)+x(1)*x(4))*dt,-(x(1)^2-x(2)^2+x(3)^2-x(4)^2)*dt,-2*(x(4)*x(3)-x(1)*x(2))*dt, 0, 0, 0, 0, 0, 0
        0, 0, 0, -2*(x(2)*x(4)-x(1)*x(3))*dt,-2*(x(4)*x(3)+x(1)*x(2))*dt,-(x(1)^2-x(2)^2-x(3)^2+x(4)^2)*dt, 0, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, dt, 0, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt, 0
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt];
    Q = G*Q*G';
    Q_test(i,:) = [Q(1,1),Q(2,2),Q(3,3),Q(4,4),Q(5,5),Q(6,6),Q(7,7),Q(8,8),Q(9,9),Q(10,10),Q(11,11),Q(12,12),Q(13,13),Q(14,14),Q(15,15),Q(16,16)];
%     Q = diag([0.05 0.05 0.05 0.05 0.05 0.05 0.001 0.005 0.005 0.001 0.5 0.5 0.5 0.005 0.005 0.0001]);
    p = FF * p * FF' + Q; 
    % Force symmetry on the covariance matrix to prevent ill-conditioning
    % of the matrix which would cause the filter to blow-up
%     p = 0.5*(p + p');
%     % ensure diagonals are positive
%     for m=1:16
%         if p(m,m) < 0
%             p(m,m) = 0;
%         end
%     end 
%         p_predict_test(:,i)=[p(1,1);p(2,2);p(3,3)];    
%% gps_heading correct
    if(gps_heading_correct)
if(gps_heading(i+gps_buf) == gps_heading(i+gps_buf-1))
else

        euler = q2euler(x(1:4));
        predict_yaw = euler(3);
        measure_yaw = gps_heading(i + gps_buf);
        t1 = 2*(x(1)*x(4) + x(2)*x(3));
        t2 = x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2;
        t3 = t1/t2;
        t4 = 1/(1 + t3^2);
        t5 = t2^2;
        t6 = t4/t5;
        H_gps_heading = [t6*(2*x(4)*t2-2*x(1)*t1), t6*(2*x(3)*t2-2*x(2)*t1), t6*(2*x(2)*t2+2*x(3)*t1), t6*(2*x(1)*t2+2*x(4)*t1),0,0,0,0,0,0,0,0,0,0,0,0];
%         H_gps_heading = [ -(2*(x(1)^2*x(4) + 2*x(1)*x(2)*x(3) - x(2)^2*x(4) + x(3)^2*x(4) + x(4)^3))/(((4*(x(1)*x(4) + x(2)*x(3))^2)/(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + 1)*(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2), ...
%                  -(2*(- x(1)^2*x(3) + 2*x(1)*x(2)*x(4) + x(2)^2*x(3) + x(3)^3 + x(3)*x(4)^2))/(((4*(x(1)*x(4) + x(2)*x(3))^2)/(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + 1)*(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2), ...
%                  (2*(x(1)^2*x(2) + 2*x(1)*x(3)*x(4) + x(2)^3 + x(2)*x(3)^2 - x(2)*x(4)^2))/(((4*(x(1)*x(4) + x(2)*x(3))^2)/(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + 1)*(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2), ...
%                  (2*(x(1)^3 + x(1)*x(2)^2 - x(1)*x(3)^2 + x(1)*x(4)^2 + 2*x(2)*x(3)*x(4)))/(((4*(x(1)*x(4) + x(2)*x(3))^2)/(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + 1)*(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2), ...
%                  0,0,0,0,0,0,0,0,0,0,0,0];
%         H_gps_heading = [((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 * ((2 * x(4)) / (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2) - (2 * x(1) * (2 * x(1) * x(4) + 2 * x(2) * x(3))) / (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2)) / ...
%             ((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + (2 * x(1) * x(4) + 2 * x(2) * x(3))^2), ((x(1)^2 + x(2)^2 - x(3)^2 -  x(4)^2)^2 * ((2 * x(3)) / (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2) - (2 * x(2) * (2 * x(1) * x(4) + 2 * x(2) * x(3))) / ...
%         (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2)) / ((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + (2 * x(1) * x(4) + 2* x(2) * x(3))^2), ((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 * ((2 * x(2)) / (x(1)^2 + x(2)^2 - x(3)^2 -x(4)^2) + (2 * x(3) * (2 * x(1) * x(4) + 2 * x(2) * x(3))) / (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2)) / ((x(1)^2 ...
%         +  x(2)^2 - x(3)^2 - x(4)^2)^2 + (2 * x(1) * x(4) + 2 * x(2) * x(3))^2), ((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 *  ((2 * x(1)) / (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2) + (2 * x(4) * (2 * x(1) * x(4) + 2 * x(2) * x(3))) /(x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2)) / ((x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2)^2 + (2 * x(1) * x(4) + 2 *  x(2) * x(3))^2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
       
        r_gps_heading = measure_yaw - predict_yaw;
        if(r_gps_heading > pi)
            r_gps_heading = r_gps_heading-2*pi;
        elseif(r_gps_heading < -pi)
            r_gps_heading = 2*pi+r_gps_heading;
        end
        R_mag = (5/vehicle_speed(i)*DEG_TO_RAD_DOUBLE)^2;
        si_mag = pinv(H_gps_heading * p * H_gps_heading' + R_mag);
        beta_mag = r_gps_heading^2 * si_mag;
%         if(beta_mag < 20000000)
            k_mag = p * H_gps_heading' * si_mag;
            correction = k_mag * r_gps_heading;
            x = x + correction;
            p = (eye(16) - k_mag * H_gps_heading) * p;
            x(1:4) = x(1:4)/norm(x(1:4)); 
            temp = q2euler(x(1:4));
            corr_gps_heading(i) = (temp(1) - euler(1))*57.3;
%         end
end
    end
    %% gps correct           
    if(gps_correct)       
if(gps(i+gps_buf,1) == gps(i+gps_buf-1,1))
else
euler = q2euler(x(1:4));
            R_gps = diag([gps_s_var(i+gps_buf) gps_s_var(i+gps_buf) gps_s_var(i+gps_buf)].^2);
            z_gps = gps(i+gps_buf,1:3)';
            x_buf = x;
            r_gps = z_gps  - H_gps * x_buf;
            hpht = H_gps * p * H_gps';
            si_gps = pinv(H_gps * p * H_gps' + R_gps);
            beta_gps = r_gps' * si_gps * r_gps;
%             if(beta_gps < 1000000000)
                k_gps = p * H_gps' * si_gps;
                correction = k_gps * r_gps;
% correction(1:4) = 0;
% correction(11:13) = 0;
                x = x + correction;
                p = (eye(16) - k_gps * H_gps) * p;    
                x(1:4) = x(1:4)/norm(x(1:4)); 
%             end
                corr_gps = corr_gps+ k_gps * r_gps;
                temp = q2euler(x(1:4));
            corr_gps_vel(i) = (temp(1) - euler(1))*57.3;
end
    end
    si_gps_test(i,:) = [hpht(1,1) hpht(2,2) hpht(3,3)];
%% zupt
    if(zp_correct)
        if(vehicle_speed(i) < 1e-4)
            r_zp = [0;0] - H_zp * x;
            si_zp = inv(H_zp * p * H_zp' + R_zp);
            beta_zp = r_zp' * si_zp * r_zp;
            if(beta_zp < 20000000)
                k_zp = p * H_zp' * si_zp;
    %             k_baro(1:4) = zeros(4,1);
                x = x + k_zp * r_zp;
                p = (eye(16) - k_zp * H_zp) * p;
                x(1:4) = x(1:4)/norm(x(1:4)); 
            end
    %         p_test(:,i_baro)=[p(1,1);p(2,2);p(3,3)];
    %         x_show(:,i_baro-1) = x;
    % 
    %         corr_baro(:,i)= corr_baro(:,i-1)+ k_baro * r_baro;
    corr_zp(:,i)=  k_zp * r_zp;
    %         vz_test(ii) = vzz ;%acc�����ٶ�
    %         zzz(ii) = vz_test(ii) + corr_baro(2,i_baro-1);%acc�����ٶ�+baro���ٶȵ�����
    %         ii=ii+1;
        end
    end

%% vehicle_speed correct 
    if(vehicle_speed_correct)
            
        vel = sqrt(x(5)^2 + x(6)^2 + x(7)^2);
        H_speed = [0 0 0 0 x(5)/vel x(6)/vel x(7)/vel 0 0 0 0 0 0 0 0 0];
        r_speed = vehicle_speed(i) - vel;
        
        si_speed = inv(H_speed * p * H_speed' + R_speed);
        beta_speed = r_speed^2 * si_speed;
        if(beta_speed < 10000000 )
            k_speed = p * H_speed' * si_speed;
            x = x + k_speed * r_speed;
            p = (eye(16) - k_speed * H_speed) * p;
        end
        corr_speed = corr_speed + k_speed * r_speed;
    end
%% buffer
    x_show(:,i) = x;
    yaw_temp = q2euler(x(1:4));
    if(yaw_temp > pi)
        yaw_temp = yaw_temp-2*pi;
    elseif(yaw_temp < -pi)
        yaw_temp = 2*pi+yaw_temp;
    end
    att_show(:,i) = yaw_temp*57.3;
    beta_speed_test(i) = beta_speed;
    beta_gps_test(i) = beta_gps;

%     corr_speed_test(:,i) = k_gps;
    corr_gps_test(:,i) = corr_gps;
    vx_test(i) = vxx;% + corr_speed(3);
    vy_test(i) = vyy;% + corr_speed(4);
    vz_test(i) = vzz;% + corr_speed(4);
    xx_test(i) = xx + corr_speed(1);
    yy_test(i) = yy + corr_speed(2);
    p_test(i,:) = [p(1,1),p(2,2),p(3,3),p(4,4),p(5,5),p(6,6),p(7,7),p(11,11),p(12,12),p(13,13),p(14,14),p(15,15),p(16,16)];
%     k_test(:,i) = k_mag;
%     k_test(:,i) = [k_baro(7);k_baro(10)];
    an_test(:,i) = a_n_test;
end

figure;
plot(att_show(1,:),'r','LineWidth',2);hold on;ylabel('roll');
plot(att(:,1),'b','LineWidth',2);hold on;
legend('estimate','att');title('roll');grid on;

figure;
plot(att_show(2,:),'r','LineWidth',2);hold on;ylabel('pitch');
plot(att(:,2),'b','LineWidth',2);hold on;
% plot(yy_test,'m','LineWidth',2);hold on;
legend('estimate','att');title('pitch');grid on;

figure;
plot(att_show(3,:),'r','LineWidth',2);hold on;ylabel('yaw');
plot(att(:,3)+90,'b','LineWidth',2);hold on;plot(gps_heading*57.3)
% plot(baro-baro_origin,'k','LineWidth',1);hold on;
% plot(yy_test,'m','LineWidth',2);hold on;
legend('estimate','att');title('yaw');grid on;

figure;
plot(x_show(5,:),'r','LineWidth',2);hold on;ylabel('x');
plot(v(:,1),'b','LineWidth',2);hold on;
plot(gps(:,1),'k','LineWidth',2);hold on;
% plot(xx_test,'m','LineWidth',2);hold on;
legend('vx_estimate','odom','gps');title('vx');grid on;

figure;
plot(x_show(6,:),'r','LineWidth',2);hold on;ylabel('y');
plot(v(:,2),'b','LineWidth',2);hold on;
plot(gps(:,2),'k','LineWidth',2);hold on;
% plot(yy_test,'m','LineWidth',2);hold on;
legend('vy_estimate','local_y','integral_y');title('vy');grid on;

figure;
plot(-x_show(7,:),'r','LineWidth',2);hold on;ylabel('z');
plot(-v(:,3),'b','LineWidth',2);hold on;
plot(gps(:,3),'k','LineWidth',2);hold on;
% plot(yy_test,'m','LineWidth',2);hold on;
legend('vz_estimate','local_z','integral_z');title('vz');grid on;

% figure;
% plot(corr_gps_test(5,:),'r');hold on;grid on;
% plot(corr_speed_test(5,:),'b');hold on;
% % plot(k_test(1,:));hold on;
% legend('gps','corr_speed');


figure;
plot(x_show(11:13,:)'*57.3,'LineWidth',2);hold on;ylabel('bx');title('gyro bias');grid on;
plot(bias(:,1:3)*57.3,'LineWidth',2);hold on;
% plot(tz,'y','LineWidth',2);hold on;
% legend('estimated','gyro bias');
% 
figure;
plot(x_show(14:16,:)','LineWidth',2);hold on;ylabel('by');title('acc bias');grid on;
plot(bias(:,4:6),'LineWidth',2);hold on;
% plot(tz,'y','LineWidth',2);hold on;
% legend('estimated','acc bias');
% 
% figure;
% plot(x_show(13,:)*57.3,'r','LineWidth',2);hold on;ylabel('bz');title('bias');grid on;
% plot(bias(:,3),'b','LineWidth',2);hold on;
% % plot(tz,'y','LineWidth',2);hold on;
% legend('estimated','gyro z');
clear x_show;
clear gps_s_var;
clear v;
clear gps;
index
% figure;
% % plot(flow_v_show(1,:),'g','LineWidth',2);hold on;
% plot(x_show(5,:),'r','LineWidth',2);hold on;ylabel('vx');
% plot(v(:,1),'b','LineWidth',2);hold on;
% % plot(vx_test(:),'m','LineWidth',2);hold on;
% % plot(-FLOW_RawX(:)*10,'y','LineWidth',2);hold on;
% % plot(GPS_VelN,'m','LineWidth',2);hold on;
% legend('vx_estimate','local_vx','acc_integral');title('ˮƽ�����ٶ�');grid on;
% 
% figure;
% % plot(flow_v_show(2,:),'g','LineWidth',2);hold on;
% plot(x_show(6,:),'r','LineWidth',2);hold on;ylabel('vy');
% plot(v(:,2),'b','LineWidth',2);hold on;
% % plot(vy_test(:),'m','LineWidth',2);hold on;
% % plot(-FLOW_RawY(:)*10,'y','LineWidth',2);hold on;
% % plot(GPS_VelE,'m','LineWidth',2);hold on;
% legend('vy_estimate','local_vy','acc_integral');title('ˮƽ�����ٶ�');grid on;
% 
% figure;
% % plot(flow_v_show(1,:),'g','LineWidth',2);hold on;
% plot(-x_show(7,:),'r','LineWidth',2);hold on;ylabel('vz');
% plot(-v(:,3),'b','LineWidth',2);hold on;
% plot(vz_test(:),'m','LineWidth',2);hold on;
% % plot(-FLOW_RawX(:)*10,'y','LineWidth',2);hold on;
% % plot(GPS_VelD,'m','LineWidth',2);hold on;
% legend('vz_estimate','local_vz','acc_integral');title('�����ٶ�');grid on;
% 


% figure;
% plot(k_test(1,:),'r');hold on;grid on;
% plot(k_test(2,:),'b');hold on;
% % plot(k_test(1,:));hold on;
% legend('p_test','k_test');
% 


% figure;
% plot(beta_speed_test);hold on;title('beta');grid on;
% figure;
% plot(beta_gps_test);hold on;
% legend('beta_speed','beta_gps');
% end