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

1. UR3e Simulation starten
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

2. IP Adresse nachschauen mit:
```bash
ifconfig
```
Step 3 und 4 in PolyScope
3. IP Adresse eingeben (Installation -> External Control)
4. IP Adresse berechtigung geben (Run -> URCaps -> External Control)
5. Treiber starten
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```

6. Externes Program im Polyscope mit dem Play-Button starten
7. Starten von MoveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

8. Starten der Node
```bash
ros2 launch moving_ur3e_ros2 move_ur3e.launch.py
```
