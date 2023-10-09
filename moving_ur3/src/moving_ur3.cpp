#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// für trajectory
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moving_ur3");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    // static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // ROS_INFO_NAMED("tutorial", " ----------- Getting Basic Information -----------");
    std::cout << "\n----------- Getting Basic Information -----------\n"
              << std::endl;

    // printing name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // printing the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo"); // stops code until "Next" is pressed

    // =================================================================================================================================

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose1);

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
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); // --> wie im tutorial, funktioniert aber nicht !!!
    // visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_); // möglich lösung falls die trajectory verschoben ist
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path");

    // -------------------------- EXECUTING PLAN --------------------------

    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    move_group_interface.execute(my_plan);

    // -------------------------- END: EXECUTING PLAN --------------------------

    // -------------------------- PLAN & EXECUTING --------------------------

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // If you do not want to inspect the planned trajectory,
    // the following is a more robust combination of the two-step plan+execute pattern shown above
    // and should be preferred. Note that the pose goal we had set earlier is still active,
    // so the robot will try to move to that goal.

    // move_group_interface.move();

    // -------------------------- END: PLAN & EXECUTING --------------------------

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // =================================================================================================================================

    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    const double tau = 2 * M_PI;
    joint_group_positions[0] = -tau / 3; // -1/3 turn in radians // [0] --> weil man einfach den ersten joint nimmt
    move_group_interface.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path");

    // -------------------------- EXECUTING PLAN --------------------------

    move_group_interface.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // =================================================================================================================================

    // Planning path to the start Pose
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can plan a motion for this group to a desired pose for the end-effector.
    geometry_msgs::Pose start_pose_helper;
    start_pose_helper.orientation.w = 1.0;
    start_pose_helper.position.x = 0.35;
    start_pose_helper.position.y = -0.05;
    start_pose_helper.position.z = 0.6;
    move_group_interface.setPoseTarget(start_pose_helper);

    // call Planner to compute the plan and visualizing it
    // No actual movement of the move_group_interface
    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan to start pose %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    //
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan to start pose as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose_helper, "start pose");
    visual_tools.publishText(text_pose, "Start Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path and go to start pose");

    // -------------------------- EXECUTING PLAN --------------------------

    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    move_group_interface.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // -------------------------- END: EXECUTING PLAN --------------------------

    // Planning with Path Constraints
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Path constraints can easily be specified for a link on the robot.
    // Let's specify a path constraint and a pose goal for our group.
    // First define the path constraint.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "tool0";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);

    // Enforce Planning in Joint Space
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Depending on the planning problem MoveIt chooses between
    // ``joint space`` and ``cartesian space`` for problem representation.
    // Setting the group parameter ``enforce_joint_model_state_space:true`` in
    // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
    //
    // By default planning requests with orientation path constraints
    // are sampled in ``cartesian space`` so that invoking IK serves as a
    // generative sampler.
    //
    // By enforcing ``joint space`` the planning process will use rejection
    // sampling to find valid requests. Please note that this might
    // increase planning time considerably.
    //
    // We will reuse the old goal that we had and plan to it.
    // Note that this will only work if the current state already satisfies the path constraints.
    // So we need to set the start state to a new pose.
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.35;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.6;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group_interface.setStartState(start_state);

    // Now we will plan to the earlier pose target from the new start state that we have just created.
    move_group_interface.setPoseTarget(target_pose1);

    // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
    // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
    move_group_interface.setPlanningTime(10.0);

    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path");

    // -------------------------- EXECUTING PLAN --------------------------

    move_group_interface.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // When done with the path constraint be sure to clear it.
    move_group_interface.clearPathConstraints();

    // =================================================================================================================================

    // Planning path to the start Pose
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can plan a motion for this group to a desired pose for the end-effector.
    //geometry_msgs::Pose start_pose_helper_2;
    //start_pose_helper.orientation.w = 1.0;
    //start_pose_helper.position.x = 0.35;
    //start_pose_helper.position.y = -0.05;
    //start_pose_helper.position.z = 0.6;
    move_group_interface.setStartStateToCurrentState(); // <-- wichtig nach .clearPathConstrains()
    move_group_interface.setPoseTarget(start_pose2);

    // call Planner to compute the plan and visualizing it
    // No actual movement of the move_group_interface
    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan to start pose %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    //
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan to start pose as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start pose");
    visual_tools.publishText(text_pose, "Start Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("tool0"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute planned path and go to start pose");

    // -------------------------- EXECUTING PLAN --------------------------

    // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
    // Note that this can lead to problems if the robot moved in the meanwhile.
    move_group_interface.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // -------------------------- END: EXECUTING PLAN --------------------------

    // ------------- moving to start pose -------------

    //visual_tools.deleteAllMarkers();
    //move_group_interface.setPoseTarget(start_pose2);
    //move_group_interface.move();

    // ------------- END: moving to start pose -------------

    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a Cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3); // down

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3); // right

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3); // up and left

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), PLANNING_GROUP);

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool timing_success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s", timing_success ? "SUCCEDED" : "FAILED");

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    /*
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        double time = 0.0; // Startzeitpunkt

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            trajectory_msgs::JointTrajectoryPoint trajectory_point;
            trajectory_point.positions.push_back(waypoints[i].position.x);
            trajectory_point.positions.push_back(waypoints[i].position.y);
            trajectory_point.positions.push_back(waypoints[i].position.z);
            trajectory_point.time_from_start = ros::Duration(time);
            trajectory.joint_trajectory.points.push_back(trajectory_point);
            time += 1.0; // Inkrementieren Sie die Zeit für jeden Punkt um 1 Sekunde
        }

        ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path)");
    */
    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group); ////////////////////////////////////////////////
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to execute trajectory");

    // -------------------------- EXECUTING TRAJECTORY --------------------------
    /*
        // timing waypoints of trajectory
        ros::Time current_time = ros::Time::now();

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(i * eef_step); // Annehmen, dass eef_step die Zeitspanne zwischen den Wegpunkten ist
            trajectory.joint_trajectory.header.stamp = current_time;
            current_time += ros::Duration(eef_step);
        }

        // checken ob die zeiten richtig gesetzt worden sind
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        bool timing_success = time_param.computeTimeStamps(trajectory);
        if (timing_success)
        {
            ROS_INFO("Time stamps computed successfully");
        }
        else
        {
            ROS_ERROR("Failed to compute time stamps for trajectory");
        }
    */

    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
    // Pull requests are welcome.
    //
    // You can execute a trajectory like this:
    // move_group_interface.execute(trajectory);
    my_plan.trajectory_ = trajectory;
    move_group_interface.execute(my_plan);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // =================================================================================================================================

    // next: adding objects to the environment

    return 0;
}