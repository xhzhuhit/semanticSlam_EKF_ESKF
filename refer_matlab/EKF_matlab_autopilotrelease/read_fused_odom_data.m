clc;
% clear all;
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

%% IAM20685
%odom attitude-roll,pitch,yaw
odom_rpy1 = data1(:,3:5)*57.3;
%odom velocity-xyz
odom_vel1 = data1(:,6:8);
%odom mercator-xyz
odom_mer1 = data1(:,9:10);
odom_pos1 = sqrt((data1(:,9)-data1(1,9)).^2+(data1(:,10)-data1(1,10)).^2);

%imu-gyro+acc
imu1 = data1(:,19:24);
%imu_bias - gyro bias + acc bias
bias1 = data1(:,12:17);

%cpt
cpt_rpy1 = data1(:,44:46)*57.3;
cpt_vel1 = data1(:,47:49);
cpt_mer1 = data1(:,50:51);
cpt_pos1 = sqrt((data1(:,50)-data1(1,50)).^2+(data1(:,51)-data1(1,51)).^2);

for i = 1:length(data1(:,1))
    q = from_euler(data1(i,2),data1(i,3),data1(i,4));
    R = rotation_matrix(q(1),q(2),q(3),q(4));
    acc_n(:,i) = R * imu1(i,4:6)';
end

len = length(t1);
for i = 1:len
    
    j = find(abs(t(j:end,1)-t1(i,1))<3000);
    if(isempty(j))
        j=1;
        continue;
    else
    delta_pos1 = [delta_pos1;odom_pos(j)-odom_pos1(i)];
    delta_mer1 = [delta_mer1;odom_mer(j,:)-odom_mer1(i,:)];
    end

end
% 
% figure;
% plot(data(:,1),odom_mer-odom_mer(1,:));hold on;grid on;
% plot(data1(:,1),odom_mer1-odom_mer1(1,:));hold on;
% 
% figure;
% plot(data(:,1),odom_pos-odom_pos(1));hold on;grid on;
% plot(data1(:,1),odom_pos1-odom_pos1(1));hold on;
% figure;
% plot(delta_mer1);hold on;grid on;
% figure;
% plot(delta_pos1);hold on;grid on;

figure;
plot(data(:,1),odom_pos-cpt_pos);hold on;
plot(data1(:,1),odom_pos1-cpt_pos1);hold on;legend('HI219M','IAM20685');grid on;
title('墨卡托坐标系下水平位置与CPT之差');

figure;
plot(data(:,1),odom_rpy-cpt_rpy);hold on;legend('roll','pitch','yaw');grid on;
plot(data1(:,1),odom_rpy1-cpt_rpy1);hold on;
figure;
plot(data(:,1),odom_vel-cpt_vel);hold on;legend('vel x','y','z');grid on;
plot(data1(:,1),odom_vel1-cpt_vel1);hold on;
figure;
plot(data(:,1),odom_mer-cpt_mer);hold on;
plot(data1(:,1),odom_mer1-cpt_mer1);hold on;legend('x__219','y__219','x__20685','y__20685');grid on;
title('墨卡托坐标系下水平位置与CPT之差');


figure;
plot(data(:,1),bias(:,1:3)*57.3);hold on;legend('bias_gyro_x','gyro_y','gyro_z');grid on;
plot(data1(:,1),bias1(:,1:3)*57.3);hold on;

figure;
plot(data(:,1),bias(:,4:6));hold on;legend('bias_acc_x','acc_y','acc_z');grid on;
plot(data1(:,1),bias1(:,4:6));hold on;

figure;
plot(data(:,1),imu(:,1:3)*57.3);hold on;legend('gyro_x','gyro_y','gyro_z');grid on;
plot(data1(:,1),imu1(:,1:3)*57.3);hold on;

figure;
plot(data(:,1),imu(:,4:6));hold on;legend('acc_x','acc_y','acc_z');grid on;
plot(data1(:,1),imu1(:,4:6));hold on;

figure;
plot(vehicle_speed);hold on;legend('vehicle_speed');grid on;

figure;
plot(vision_h);hold on;legend('vision_heading','heading_had','heading_relative');
% figure;
% plot(ll_start_y);hold on;plot(ll_end_y);hold on;legend('right start_y','end_y');



