# ros_sample: visualization_msgs/Marker and urdf
ROS sample


Tested on ROS kinetic on Ubuntu 16.04

URDF Reference: https://github.com/AlessioTonioni/Autonomous-Flight-ROS

visualization_msgs/Marker
urdf


frame 'odom' as world frame, and the frame 'base_link'
tf broadcaster: /node_drone_uwb
the '/node_drone_uwb' broadcastes a transformation between the two frames.

the 'base_link' is defined in the urdf file.



### Build

Put this package under '~/catkin_ws/src'

$ catkin_make


### Run

~/catkin $ source devel/setup.bash

~/catkin $ roslaunch pkg_drone_uwb pkg_drone_uwb.launch

Rviz comes up, and set 'odom'
(Displays -> Global Options -> Fixed Frame ----> odom)
add Marker and RobotModel


![](https://github.com/jungwonkang/sample_ros_visualization/blob/master/screen_shot_rviz.jpg)


