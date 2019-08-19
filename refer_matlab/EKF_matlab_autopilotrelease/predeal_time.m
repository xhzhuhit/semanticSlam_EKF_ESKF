function [ A, B ] = predeal_time( A, B )
%% 时间去皮 and 将时间单位从usec改为sec
% A: [n * n], 第一列为时间
% B: [n * n], 第一列为时间
t_bias = min(A(1,1), B(1,1));
A(:, 1) = (A(:, 1) - t_bias)/1000000;
B(:, 1) = (B(:, 1) - t_bias)/1000000;

end

