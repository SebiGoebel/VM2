#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "std_msgs/msg/color_rgba.hpp"

geometry_msgs::msg::Quaternion eul2quat(const tf2Scalar &roll, const tf2Scalar &pitch, const tf2Scalar &yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion ros2_quaternion;
    tf2::convert(quaternion, ros2_quaternion);
    return ros2_quaternion;
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moving_ur3e",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("moving_ur3e");

    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    // ---------------------------- Setting up Robot ----------------------------

    static const std::string PLANNING_GROUP = "ur_manipulator";

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // ---------------------------- Creating and Initialize MoveItVisualTools ----------------------------

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // ---------------------------- Lamda Functions for Visualisations with MoveItVisualTools ----------------------------

    // Drawing a title
    auto const draw_title = [&moveit_visual_tools](auto text)
    {
        auto const text_pose = []
        {
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0; // Place text 1m above the base link
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };

    // stopping code and wating for user to press "Next"
    auto const prompt = [&moveit_visual_tools](auto text)
    {
        moveit_visual_tools.prompt(text);
    };

    // Lamda Funktion zum anzeigen einer Trajectorie
    auto const draw_trajectory_tool_path = [&moveit_visual_tools,
                                            jmg_link = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getLinkModel("tool0"),
                                            jmg = joint_model_group,
                                            color = rviz_visual_tools::LIME_GREEN](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg_link, jmg, color);
    };

    auto const draw_trajectory_tool_path_red = [&moveit_visual_tools,
                                            jmg_link = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getLinkModel("tool0"),
                                            jmg = joint_model_group,
                                            color = rviz_visual_tools::RED](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg_link, jmg, color);
    };

    auto const draw_trajectory_tool_path_blue = [&moveit_visual_tools,
                                            jmg_link = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getLinkModel("tool0"),
                                            jmg = joint_model_group,
                                            color = rviz_visual_tools::BLUE](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg_link, jmg, color);
    };

    // ---------------------------- Lamda Functions for getting a relative pose to the start pose (Constraints) ----------------------------

    auto current_pose = move_group_interface.getCurrentPose();

    // Creates a pose at a given positional offset from the current pose
    auto get_relative_pose = [current_pose, &moveit_visual_tools](double x, double y, double z) {
       auto target_pose = current_pose;
       target_pose.pose.position.x += x;
       target_pose.pose.position.y += y;
       target_pose.pose.position.z += z;
       moveit_visual_tools.publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
       moveit_visual_tools.publishSphere(target_pose.pose, rviz_visual_tools::GREEN, 0.05);
       moveit_visual_tools.trigger();
       return target_pose;
    };

    // ---------------------------- Adding a BOX as Floor ----------------------------

    prompt("Press 'Next' in the RvizVisualToolsGui window to add the floor");
    draw_title("Adding Floor");
    moveit_visual_tools.trigger();

    // Create collision object for the robot to avoid
    auto const collision_object_floor = [frame_id = move_group_interface.getPlanningFrame()]
    {
        moveit_msgs::msg::CollisionObject collision_object_floor;
        collision_object_floor.header.frame_id = frame_id;
        collision_object_floor.id = "box_floor";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.5;
        primitive.dimensions[primitive.BOX_Y] = 1.5;
        primitive.dimensions[primitive.BOX_Z] = 0.01;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.01;

        collision_object_floor.primitives.push_back(primitive);
        collision_object_floor.primitive_poses.push_back(box_pose);
        collision_object_floor.operation = collision_object_floor.ADD;

        return collision_object_floor;
    }();

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //const std_msgs::msg::ColorRGBA& greyColor = std_msgs::msg::ColorRGBA{0.5, 0.5, 0.5, 1.0};
    //planning_scene::PlanningScene::setObjectColor("box_floor", greyColor);
    planning_scene_interface.applyCollisionObject(collision_object_floor);

    // ---------------------------- Adding a BOX as Constraint ----------------------------
/*
    // Let's try the simple box constraints first!
    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
    box_constraint.link_name = move_group_interface.getEndEffectorLink();

    //shape_msgs::msg::SolidPrimitive box;
    shape_msgs::msg::SolidPrimitive primitive_box_constraint;
    primitive_box_constraint.type = shape_msgs::msg::SolidPrimitive::BOX;
    //primitive_box_constraint.type = primitive_box_constraint.BOX;
    primitive_box_constraint.dimensions = { 1.0, 1.0, 1.0 };
    //primitive_box_constraint.dimensions[primitive_box_constraint.BOX_X] = 1.0;
    //primitive_box_constraint.dimensions[primitive_box_constraint.BOX_Y] = 1.0;
    //primitive_box_constraint.dimensions[primitive_box_constraint.BOX_Z] = 1.0;
    box_constraint.constraint_region.primitives.emplace_back(primitive_box_constraint);

    geometry_msgs::msg::Pose constraint_box_pose;
    constraint_box_pose.position.x = 0.0;
    constraint_box_pose.position.y = 0.5;
    constraint_box_pose.position.z = 0.5;
    constraint_box_pose.orientation.w = 1.0;
    box_constraint.constraint_region.primitive_poses.emplace_back(constraint_box_pose);
    box_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints box_constraints;
    box_constraints.position_constraints.emplace_back(box_constraint);

    move_group_interface.setPathConstraints(box_constraints);
    move_group_interface.setPlanningTime(10.0);
*/
    // ---------------------------- Moving to start pose ----------------------------

    // Set a target Pose
    auto const start_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = -0.15;
        msg.position.y = 0.1;
        msg.position.z = 0.25;
        return msg;
    }();
    move_group_interface.setPoseTarget(start_pose);
    moveit_visual_tools.publishAxisLabeled(start_pose, "start_pose");

    // RvizVisualToolsGui step
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan to start goal");
    draw_title("Planing");
    moveit_visual_tools.trigger();

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        // drawing the plan
        draw_trajectory_tool_path(plan.trajectory_);
        moveit_visual_tools.trigger();

        // Executing
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    moveit_visual_tools.deleteAllMarkers();

    // --------------- adding Orientation Constraints ---------------
/*
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
    orientation_constraint.link_name = move_group_interface.getEndEffectorLink();

    auto current_pose_start_pose = move_group_interface.getCurrentPose();
    orientation_constraint.orientation = current_pose_start_pose.pose.orientation;
    orientation_constraint.absolute_x_axis_tolerance = 0.4;
    orientation_constraint.absolute_y_axis_tolerance = 0.4;
    orientation_constraint.absolute_z_axis_tolerance = 0.4;
    orientation_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints mixed_constraints;
    mixed_constraints.position_constraints.emplace_back(box_constraint);
    mixed_constraints.orientation_constraints.emplace_back(orientation_constraint);

    move_group_interface.setPathConstraints(mixed_constraints);
    move_group_interface.setPlanningTime(15.0);
*/
    // ---------------------------- Moving to joint target ----------------------------

    draw_title("Planing to joint target");
    moveit_visual_tools.trigger();

    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785};
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group_interface.setJointValueTarget(joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    
    // RvizVisualToolsGui step
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planing");
    moveit_visual_tools.trigger();

/*
    // Create a plan to that target pose
    auto const [success_target, plan_target] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success_target)
    {
        // drawing the plan
        draw_trajectory_tool_path_red(plan_target.trajectory_);
        moveit_visual_tools.trigger();

        // Executing
        prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan_target);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }
*/



    // call Planner to compute the plan and visualizing it
    // No actual movement of the move_group_interface
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    //
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rviz_visual_tools::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); // --> wie im tutorial, funktioniert aber nicht !!!
    // visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_); // möglich lösung falls die trajectory verschoben ist

    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path");
/*
    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rviz_visual_tools::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    moveit_visual_tools.trigger();
    moveit_visual_tools.prompt("Press 'next' to execute planned path");

    // Executing plan
    move_group_interface.execute(my_plan);
*/
    // ----------------------------------------------------------------------------------
/*
    draw_title("Planing to target pose");
    moveit_visual_tools.trigger();

    // Set a target Pose
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = 0.0;
        msg.position.y = 0.4; // 0.5
        msg.position.z = 0.25;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);
    moveit_visual_tools.publishAxisLabeled(target_pose, "target_pose");

    // RvizVisualToolsGui step
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planing");
    moveit_visual_tools.trigger();

    // Create a plan to that target pose
    auto const [success_target, plan_target] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success_target)
    {
        // drawing the plan
        draw_trajectory_tool_path_red(plan_target.trajectory_);
        moveit_visual_tools.trigger();

        // Executing
        // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        // draw_title("Executing");
        // moveit_visual_tools.trigger();
        // move_group_interface.execute(plan_target);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Pressing 'Next' to add a collision box
    prompt("Press 'Next' in the RvizVisualToolsGui window to add a collision box");
*/

    // Pressing 'Next' to add a collision box
    prompt("Press 'Next' in the RvizVisualToolsGui window to add a collision box");

    // ---------------------------- Adding a BOX ----------------------------

    draw_title("Adding Box");
    moveit_visual_tools.trigger();

    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.025;
        primitive.dimensions[primitive.BOX_Y] = 1.0;
        primitive.dimensions[primitive.BOX_Z] = 0.3;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation = eul2quat(0.0, 0.0, M_PI / 2);
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.3;
        box_pose.position.z = 0.15;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    // Add the collision object to the scene
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // bereits beim floor deklariert
    planning_scene_interface.applyCollisionObject(collision_object);

    // Pressing 'Next' to plan around the collision box
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan around the collision box");

    // ---------------------------- Planning around the box ----------------------------

    draw_title("Planning around the Box");
    moveit_visual_tools.trigger();

    // Create a plan to that target pose
    auto const [success_target_box, plan_target_box] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success_target_box)
    {
        // drawing the plan
        draw_trajectory_tool_path(plan_target_box.trajectory_);
        moveit_visual_tools.trigger();

        // Executing
        // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        // draw_title("Executing");
        // moveit_visual_tools.trigger();
        // move_group_interface.execute(plan_target);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Pressing 'Next' to add an object to the robot
    prompt("Press 'Next' in the RvizVisualToolsGui window to add an object to the robot");

    // ---------------------------- Attaching object to robot ----------------------------

    draw_title("Attaching object to robot");
    moveit_visual_tools.trigger();

    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::msg::SolidPrimitive primitive;
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.04;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    // defining a frame/pose for this cylinder so that it appears at the gripper
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.025;

    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    planning_scene_interface.applyCollisionObject(object_to_attach);
    object_to_attach.operation = object_to_attach.ADD;

    move_group_interface.attachObject(object_to_attach.id, "tool0");

    // Pressing 'Next' to plan with the attached object
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan with the attached object");

    // ---------------------------- Planning with attached object ----------------------------

    draw_title("Planning with attached object");
    moveit_visual_tools.trigger();

    // Create a plan to that target pose
    auto const [success_target_box_attached, plan_target_box_attached] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success_target_box_attached)
    {
        // drawing the plan
        draw_trajectory_tool_path_blue(plan_target_box_attached.trajectory_);
        moveit_visual_tools.trigger();

        // Executing
        // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
        // draw_title("Executing");
        // moveit_visual_tools.trigger();
        // move_group_interface.execute(plan_target);
    }
    else
    {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown(); // <--- This will cause the spin function in the thread to return
    spinner.join();     // <--- Join the thread before exiting
    return 0;
}