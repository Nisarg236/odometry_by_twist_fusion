Usage:
First build the package using colcon "build", then use "ros2 run fusion_node fusion_node" to run. It will subscribe to /odom_velocity and /imu_velocity.
And it will publish /fused_velocity and /odom. To plot odometry in another terminal open rviz using 'rviz2' and publish transform using 'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map' to be able to visualize the path.
