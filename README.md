# pfcbf_ros2
 ROS Package for Risk-Aware Control under non-Gaussian Beliefs. This package can be used to avoid areas in a known map for robots that use particle filters as state estimator. We provide a package that can be easily used in combination with the ROS2 navigation stack.

## Requirements ##
* Eigen3
* OsqpEigen (https://github.com/robotology/osqp-eigen)

## Subscribers ##
* /particle_cloud (nav2_msgs/ParticleCloud)
* /cmd_vel_ref (geometry_msgs/Twist)

## Publishers ##
* /cmd_vel (geometry_msgs/Twist)

## Example Using Turtlebot3
> ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
