import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import os
from ament_index_python import get_package_share_directory

#ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=172.17.0.2 use_fake_hardware:=true launch_rviz:=false
#ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('robot_ip', default_value='172.17.0.2'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_robot_driver'),
                         'launch/ur_control.launch.py')),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),
    ])

#def generate_launch_description():
#    return LaunchDescription([
#        DeclareLaunchArgument('ur_type', default_value='ur3e'),
#        DeclareLaunchArgument('launch_rviz', default_value='true'),
#
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                os.path.join(get_package_share_directory('ur_moveit_config'),
#                        'launch/ur_moveit.launch.py')),
#            launch_arguments={
#                'ur_type': LaunchConfiguration('ur_type'),
#                'launch_rviz': LaunchConfiguration('launch_rviz'),
#            }.items(),
#        ),
#    ])