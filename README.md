# odom_by_twist_fusion
This package subscribes twist stamped message from two different IMUs, removes noise using a kalman filter, fuses using a complimentary filter then calculates the odometry (x,y and theta). It then publishes the calculated Odometry and fused velocities. The code is implemented in C++ and works with ROS2.

Next step is to replace the complimentary filter with kalman filter for fusion and implement the same for all velocities (x,y,z,wx,wy,wz) and generalize for N number of sensor inputs.

Traced odometry path using two sources of velocities:
![Screenshot from 2023-11-30 13-12-37](https://github.com/Nisarg236/odometry_by_twist_fusion/assets/71684502/352af704-3daa-49ed-a035-85a984398b17)
