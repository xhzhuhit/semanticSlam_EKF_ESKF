clc;
clear all;
% close all;
set(0,'DefaultFigureWindowStyle','docked');
format long;


data = load('imu_4.csv');
data1 = load('imu20685_4.csv');
t=data(:,1)-data(1,1);
t1=data1(:,1)-data1(1,1);
delta_mer1 = [];
delta_pos1 = [];
j=1;
%% hi219
%odom attitude-roll,pitch,yaw
odom_rpy = data(:,3:5)*57.3;
%odom velocity-xyz
odom_vel = data(:,6:8);
%odom mercator-xyz
odom_mer = data(:,9:10);
odom_pos = sqrt((data(:,9)-data(1,9)).^2+(data(:,10)-data(1,10)).^2);

%yaw
odom_yaw = data(:,5)*57.3 + 90;
gps_heading = data(:,30)*57.3;
vision_heading = data(:,33)*57.3;

%imu-gyro+acc
imu = data(:,19:24);
%imu_bias - gyro bias + acc bias
bias = data(:,12:17);

%gps -s_variance
gps_s_var = data(:,34);
%gps -vel
gps_vel = data(:,25:27);
%gps -lat/lon
gps_pos = data(:,28:29);
%gps -heading
% gps_heading = data(:,30)*57.3;
%gps -vel_2d
gps_vel2d = data(:,35);

%vehicle speed
vehicle_speed = data(:,36);

% %vision heading-heading,heading_had,heading_relative
% vision_h = [data(:,33) data(:,38:39)]*57.3;
%vision pos-xy
vision_pos = data(:,31:32);

%cpt
cpt_rpy = data(:,44:46)*57.3;
cpt_vel = data(:,47:49);
cpt_mer = data(:,50:51);
cpt_pos = sqrt((data(:,50)-data(1,50)).^2+(data(:,51)-data(1,51)).^2);
s1 = 0; s2 = 0; s3 = 0; s4 = 0;
for i = 26761:26980
    s1 = s1 + sqrt((cpt_mer(i,1) - cpt_mer(i-1,1))^2 + (cpt_mer(i,2) - cpt_mer(i-1,2))^2);
    s2 = s2 + (sqrt(cpt_vel(i,1)^2 + cpt_vel(i,2)^2 + cpt_vel(i,3)^2) + sqrt(cpt_vel(i-1,1)^2 + cpt_vel(i-1,2)^2 + cpt_vel(i-1,3)^2))/2 * (data(i,1) - data(i-1,1))/1e6;
    s3 = s3 + vehicle_speed(i) * (data(i,1) - data(i-1,1))/1e6;
    s4 = s4 + (gps_vel2d(i) + gps_vel2d(i))/2 * (data(i,1) - data(i-1,1))/1e6; 
end

 

figure;
plot(data(:,1),odom_yaw);hold on;plot(data(:,1),gps_heading);hold on;
plot(data(:,1),vision_heading);hold on;
legend('odom_yaw','gps_heading','vision_heading');grid on;
