# ros_sample: visualization_msgsMarker and urdf
ROS sample


Tested on ROS kinetic on Ubuntu 16.04

URDF Reference: https://github.com/AlessioTonioni/Autonomous-Flight-ROS

visualization_msgs/Marker
urdf


### Build

Put this package under '~/catkin_ws/src'

$ catkin_make


### Run

~/catkin $ source devel/setup.bash

~/catkin $ roslaunch pkg_drone_uwb pkg_drone_uwb.launch

Rviz comes up, and set 'odom'
(Displays -> Global Options -> Fixed Frame ----> odom)
add Marker and RobotModel


![](https://github.com/jungwonkang/doc_supp/blob/master/screen_shot_rviz.jpg)



