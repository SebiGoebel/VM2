#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

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

    //static const std::string PLANNING_GROUP = "ur_manipulator";

    // Create the MoveIt MoveGroup Interface
    //using moveit::planning_interface::MoveGroupInterface;
    //auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
    //const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    // Setup motion planning goal taken from motion_planning_api tutorial
    const std::string planning_group = "ur_manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(planning_group);

    // Configure a valid robot state
    //robot_state->setToDefaultValues(joint_model_group, "test_configuration");
    //robot_state->update();
    //// Lock the planning scene as briefly as possible
    //{
    //    planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
    //    locked_planning_scene->setCurrentState(*robot_state);
    //}

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
    jmg_link = move_group_interface.getRobotModel()->getJointModelGroup(planning_group)->getLinkModel("tool0"),
    jmg = joint_model_group,
    color = rviz_visual_tools::LIME_GREEN](auto const trajectory)
    {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg_link, jmg, color);
    };


    // ---------------------------- Create desired motion goal ----------------------------
    moveit_msgs::msg::MotionPlanRequest goal_motion_request;

    moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
    goal_motion_request.group_name = planning_group;
    goal_motion_request.num_planning_attempts = 100; // 10
    goal_motion_request.max_velocity_scaling_factor = 0.1;
    goal_motion_request.max_acceleration_scaling_factor = 0.1;
    goal_motion_request.allowed_planning_time = 10.0; // 2.0
    goal_motion_request.planner_id = "ompl";
    goal_motion_request.pipeline_id = "ompl";

    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = {-0.35, -1.87, 0.785, -0.47, -1.6, 0.0};
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    goal_motion_request.goal_constraints.resize(1);
    goal_motion_request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    // ---------------------------- Setting up a pose + planning + executing ----------------------------

    // Set a target Pose
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);
    moveit_visual_tools.publishAxisLabeled(target_pose, "target_pose");

    // RvizVisualToolsGui step
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
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

    // Shutdown ROS
    rclcpp::shutdown(); // <--- This will cause the spin function in the thread to return
    spinner.join();     // <--- Join the thread before exiting
    return 0;
}