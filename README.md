# VM2
Vertiefungsfach - Implementation of an robotic scrub nurse, using dynamic motion planning

## UR package includeing UR3e
https://github.com/ros-industrial/universal_robot


## Tutorial for moving the UR3e
https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp


## IP-Addressen Roboter
rechts: 192.168.1.11


links:  192.168.1.10




sudo apt-get install ros-melodic-rqt-joint-trajectory-controller


## Starten am echten roboter
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.11

roslaunch ur3e_moveit_config moveit_planning_execution.launch

roslaunch ur3e_moveit_config moveit_rviz.launch


## Starten der Simulation auf Ubuntu 22.04 in ROS2

