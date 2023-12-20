# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
#from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # robot_description_planning = {
    # "robot_description_planning": load_yaml_abs(str(joint_limit_params.perform(context)))
    # }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    #ompl_planning_yaml = "/home/sebi/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/ompl_planning.yaml"
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    #ompl_planning_pipeline_config["move_group"]["ur_manipulator"]["enforce_constrained_state_space"] = ["True"]
    #ompl_planning_pipeline_config["move_group"]["ur_manipulator"]["projection_evaluator"] = "joints(shoulder_pan_joint,shoulder_lift_joint)"


    # CHOMP
    #chomp_planning_pipeline_config = {
    #    "move_group": {
    #        "planning_plugin": "chomp_interface/CHOMPPlanner",
    #        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #        "start_state_max_bounds_error": 0.1,
    #    }
    #}
    #chomp_planning_yaml = load_yaml("ur_moveit_config", "config/chomp_planning.yaml")
    #chomp_planning_pipeline_config["move_group"].update(chomp_planning_yaml)

    # STOMP
    #stomp_planning_pipeline_config = {
    #    "move_group": {
    #        "planning_plugin": "stomp_interface/StompPlanner",
    #        "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #        "start_state_max_bounds_error": 0.1,
    #    }
    #}
    #stomp_planning_yaml = load_yaml("ur_moveit_config", "config/stomp_planning.yaml")
    #stomp_planning_pipeline_config["move_group"].update(stomp_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    #controllers_yaml = "/home/sebi/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/controllers.yaml"
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            # robot_description_planning,
            ompl_planning_pipeline_config,
            #chomp_planning_pipeline_config,
            #stomp_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
            #{'move_group': {'default_planner_config': 'chomp_planner/CHOMPPlanner',
            #                    'planners': ['chomp_planner/CHOMPPlanner', 'ompl_interface/OMPLPlanner'],
            #                    'planning_plugin': 'ompl_interface/OMPLPlanner'}}
        ],
    )

    # rviz with moveit configuration
    #rviz_config_file = PathJoinSubstitution(
    #    [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    #)
    #rviz_config_file = "/home/sebi/ws_moveit2/src/moving_ur3e_ros2/config/my_rviz_config.rviz" # eigene rviz config
    rviz_config_file = FindPackageShare("moving_ur3e_ros2"), "/config/my_rviz_config.rviz"
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            #chomp_planning_pipeline_config,
            #stomp_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
            warehouse_ros_config,
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    #servo_yaml = "/home/sebi/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/ur_servo.yaml"
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # Any parameters that are unique to your plugins go here
    #common_hybrid_planning_param = load_yaml(
    #    "moving_ur3e_ros2", "config/common_hybrid_planning_params.yaml"
    #)
    #global_planner_param = load_yaml(
    #    "moving_ur3e_ros2", "config/global_planner.yaml"
    #)
    #local_planner_param = load_yaml(
    #    "moving_ur3e_ros2", "config/local_planner.yaml"
    #)
    #hybrid_planning_manager_param = load_yaml(
    #    "moving_ur3e_ros2", "config/hybrid_planning_manager.yaml"
    #)
    
    
    # loading yamls from own files
    #common_hybrid_planning_param = "/home/sebi/ws_moveit2/src/moveit2/moveit_ros/hybrid_planning/test_ur3e/config/common_hybrid_planning_params.yaml"
    #global_planner_param = "/home/sebi/ws_moveit2/src/moveit2/moveit_ros/hybrid_planning/test_ur3e/config/global_planner.yaml"
    #local_planner_param = "/home/sebi/ws_moveit2/src/moveit2/moveit_ros/hybrid_planning/test_ur3e/config/local_planner.yaml"
    #hybrid_planning_manager_param = "/home/sebi/ws_moveit2/src/moveit2/moveit_ros/hybrid_planning/test_ur3e/config/hybrid_planning_manager.yaml"

    #common_hybrid_planning_param = "/home/sebi/ws_moveit2/src/moving_ur3e_ros2/config/common_hybrid_planning_params.yaml"
    #global_planner_param = "/home/sebi/ws_moveit2/src/moving_ur3e_ros2/config/global_planner.yaml"
    #local_planner_param = "/home/sebi/ws_moveit2/src/moving_ur3e_ros2/config/local_planner.yaml"
    #hybrid_planning_manager_param = "/home/sebi/ws_moveit2/src/moving_ur3e_ros2/config/hybrid_planning_manager.yaml"

    common_hybrid_planning_param = FindPackageShare("moving_ur3e_ros2"), "/config/common_hybrid_planning_params.yaml"
    global_planner_param = FindPackageShare("moving_ur3e_ros2"), "/config/global_planner.yaml"
    local_planner_param = FindPackageShare("moving_ur3e_ros2"), "/config/local_planner.yaml"
    hybrid_planning_manager_param = FindPackageShare("moving_ur3e_ros2"), "/config/hybrid_planning_manager.yaml"


    # Generate launch description with multiple components
    container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    ompl_planning_pipeline_config,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                ],
            ),
        ],
        output="screen",
    )
    
    # Demo node
    #common_hybrid_planning_param = load_yaml(
    #    "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
    #)
    demo_node = Node(
        package="moveit_hybrid_planning",
        executable="hybrid_planning_ur3e_node",
        name="hybrid_planning_ur3e_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            common_hybrid_planning_param,
        ],
    )

    nodes_to_start = [move_group_node, rviz_node, servo_node, container]#, demo_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])



#def generate_launch_description():
#    # generate_common_hybrid_launch_description() returns a list of nodes to launch
#    common_launch = generate_common_hybrid_launch_description()
#    robot_description = get_robot_description()
#    robot_description_semantic = get_robot_description_semantic()
#
#    # Demo node
#    common_hybrid_planning_param = load_yaml(
#        "moveit_hybrid_planning", "config/common_hybrid_planning_params.yaml"
#    )
#    demo_node = Node(
#        package="moveit_hybrid_planning",
#        executable="hybrid_planning_ur3e_node",
#        name="hybrid_planning_ur3e_node",
#        output="screen",
#        parameters=[
#            get_robot_description(),
#            get_robot_description_semantic(),
#            common_hybrid_planning_param,
#        ],
#    )
#
#    return LaunchDescription(common_launch + [demo_node])
