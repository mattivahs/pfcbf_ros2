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
Start gazebo simulation of turtlebot
> ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

Run navigation stack (requires map of the environment, also need to remap /cmd_vel topic to /cmd_vel_ref)
> ros2 launch turtlebot3_navigation2 navigation2.launch.py

Run risk-aware controller
> ros2 run pfcbf pfcbf_node
