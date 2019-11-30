# semanticSlam_EKF_ESKF
Fusion imu,gps,vehicle data and intermediate result of vision.  Compare EKF &amp; ESKF in python.

##EKF
use IMU and gps's speed(N-E-D) & heading. so Position increases. Position update is linear and easy to add. So the same to vision-pose.

##ESKF
use IMU and all gps's speed(N-E-D) & heading & position(xy,no height). 
