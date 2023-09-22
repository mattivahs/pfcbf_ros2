# pfcbf_ros2
 ROS Package for Risk-Aware Control under non-Gaussian Beliefs

## Requirements ##
* Eigen3
* OsqpEigen (https://github.com/robotology/osqp-eigen)

## Subscribers ##
* /particle_cloud (nav2_msgs/ParticleCloud)
* /cmd_vel_ref (geometry_msgs/Twist)

## Publishers ##
* /cmd_vel (geometry_msgs/Twist)
