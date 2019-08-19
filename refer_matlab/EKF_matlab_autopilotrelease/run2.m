addpath(genpath(pwd))
% clear all;
% close all;
set(0,'DefaultFigureWindowStyle','docked');

%% 数据输入
data = load('imu_1.csv');
data1 = load('imu20685_1.csv');
pos1_pre = data(:, [1 9 10 50 51]);% time/2odom_mer/2cpt_mer
pos2_pre = data1(:, [1 9 10 50 51]);% time/2odom_mer/2cpt_mer

%% 时间去皮 and 将时间单位从usec改为sec
[pos1_pre, pos2_pre] = predeal_time(pos1_pre, pos2_pre);

%% 通过插值匹配数据
%将20685的mercator坐标与219进行时间对齐
pos1 = pos1_pre;
x = interp1(pos2_pre(:,1), pos2_pre(:,2), pos1_pre(:,1));
y = interp1(pos2_pre(:,1), pos2_pre(:,3), pos1_pre(:,1));
pos2 = [pos1(:,1) x y];
%将20685的mercator坐标与CPT之差与219进行时间对齐
deltax = interp1(pos2_pre(:,1), (pos2_pre(:,2)-pos2_pre(:,4)), pos1_pre(:,1));
deltay = interp1(pos2_pre(:,1), (pos2_pre(:,3)-pos2_pre(:,5)), pos1_pre(:,1));
%% 计算xy差值
det = [pos1(:,1) (pos1(:, 2:3) - pos2(:, 2:3))];
det_pos = sqrt(det(:, 2).*det(:, 2) + det(:, 3).*det(:, 3));

det1 = [pos1(:,1) deltax deltay];%data2与CPT差
det2 = [pos1(:,1) pos1_pre(:,2)-pos1_pre(:,4) pos1_pre(:,3)-pos1_pre(:,5)];%data1与CPT差
det3 = [pos1(:,1) (det1(:,2:3) - det2(:,2:3))];
det_pos1 = sqrt(det3(:, 2).*det3(:, 2) + det3(:, 3).*det3(:, 3));
%% 画图
figure;
plot(det(:,1), det_pos);hold on;grid on;
% plot(det(:,1), det_pos1);hold on;grid on;legend('det_pos','det_pos1');
det_pos(isnan(det_pos))=0;
det_pos(isinf(det_pos))=0;
mean(det_pos)
det_pos_sorted = sort(abs(det_pos));
det_pos_sorted(ceil(0.9544*length(det_pos_sorted)))


figure;
plot(data(:,1),data(:,22:24));hold on;legend('acc_x','acc_y','acc_z');grid on;
plot(data1(:,1),data1(:,22:24));hold on;

figure;
plot(det(:,1), det(:, 2:3));hold on;grid on;

figure;
plot(det(:,1), sqrt(det(:, 2).*det(:, 2) + det(:, 3).*det(:, 3)));hold on;grid on;

figure;
plot(pos1(:,1), pos1(:, 2), 'o');hold on;grid on;
plot(pos2(:,1), pos2(:, 2), 'x');hold on;grid on;

figure;
plot(pos1_pre(:,1), pos1_pre(:, 2), 'o');hold on;grid on;
plot(pos2_pre(:,1), pos2_pre(:, 2), 'x');hold on;grid on;