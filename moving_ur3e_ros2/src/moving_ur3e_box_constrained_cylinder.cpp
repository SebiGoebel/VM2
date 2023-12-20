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

// für collision objects as .stls
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#define loadingHumans false
#define visualizeArea true

geometry_msgs::msg::Quaternion eul2quat(const tf2Scalar &roll, const tf2Scalar &pitch, const tf2Scalar &yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion ros2_quaternion;
    tf2::convert(quaternion, ros2_quaternion);
    return ros2_quaternion;
}

geometry_msgs::msg::Quaternion euler2Quaternion_deg(double roll, double pitch, double yaw)
{

    double roll_rad = roll * M_PI / 180;
    double pitch_rad = pitch * M_PI / 180;
    double yaw_rad = yaw * M_PI / 180;

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll_rad, pitch_rad, yaw_rad);
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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
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
    /*
        auto const draw_trajectory_tool_path_blue = [&moveit_visual_tools,
                                                jmg_link = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getLinkModel("tool0"),
                                                jmg = joint_model_group,
                                                color = rviz_visual_tools::BLUE](auto const trajectory)
        {
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg_link, jmg, color);
        };
    */
    // ---------------------------- Lamda Functions for getting a relative pose to the start pose (Constraints) ----------------------------

    auto current_pose = move_group_interface.getCurrentPose();

    // Creates a pose at a given positional offset from the current pose
    auto get_relative_pose = [current_pose, &moveit_visual_tools](double x, double y, double z)
    {
        auto target_pose = current_pose;
        target_pose.pose.position.x += x;
        target_pose.pose.position.y += y;
        target_pose.pose.position.z += z;
        moveit_visual_tools.publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
        moveit_visual_tools.publishSphere(target_pose.pose, rviz_visual_tools::GREEN, 0.05);
        moveit_visual_tools.trigger();
        return target_pose;
    };

    // ---------------------------- Attaching object to robot ----------------------------

    shape_msgs::msg::SolidPrimitive primitive;
    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    /*
        prompt("Press 'Next' in the RvizVisualToolsGui to attach a cylinder to the robot");
        draw_title("Attaching object to robot");
        moveit_visual_tools.trigger();

        moveit_msgs::msg::CollisionObject object_to_attach;
        object_to_attach.id = "cylinder1";

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
        prompt("Press 'Next' in the RvizVisualToolsGui window to add MIR");
        draw_title("Adding MIR");
        moveit_visual_tools.trigger();
    */
    // ---------------------------- Adding a stl as Collision ----------------------------

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // --- Adding MIR ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1_mir";

    // Define a box to add to the world.
    float x_value_mir = 1.0;
    float y_value_mir = 0.6;
    float z_value_mir = 0.75;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = x_value_mir; // 1.5
    primitive.dimensions[primitive.BOX_Y] = y_value_mir; // 1.5
    primitive.dimensions[primitive.BOX_Z] = z_value_mir; // 0.01

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -x_value_mir / 4 - 0.1;    // 0.0
    box_pose.position.y = -y_value_mir / 4;          // 0.0
    box_pose.position.z = -(z_value_mir / 2) - 0.01; // -0.01 // -0.01 -> safty abstand für simulation damit keine collisions entstehen

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    planning_scene_interface.addCollisionObjects(collision_objects);

    prompt("Press 'Next' in the RvizVisualToolsGui window to add collision object");
    draw_title("Adding collision objects");
    moveit_visual_tools.trigger();

    // --- adding operation bed from stl ---

    moveit_msgs::msg::CollisionObject co_bed;
    co_bed.header.frame_id = move_group_interface.getPlanningFrame();
    co_bed.id = "bed_mesh";
    move_group_interface.getPlanningFrame();

    Eigen::Vector3d scale_bed(0.01, 0.01, 0.01);
    shapes::Mesh *m_bed = shapes::createMeshFromResource("package://moving_ur3e_ros2/collisionObjects/stls/medicalBed.stl", scale_bed);

    shape_msgs::msg::Mesh mesh_bed;
    shapes::ShapeMsg mesh_msg_bed;
    shapes::constructMsgFromShape(m_bed, mesh_msg_bed);
    mesh_bed = boost::get<shape_msgs::msg::Mesh>(mesh_msg_bed);

    geometry_msgs::msg::Pose bed_pose;
    bed_pose.position.x = 0.5;
    bed_pose.position.y = 0.8;
    bed_pose.position.z = 0.0;
    bed_pose.orientation = euler2Quaternion_deg(90, 0.0, 90);

    co_bed.meshes.push_back(mesh_bed);
    co_bed.mesh_poses.push_back(bed_pose);
    co_bed.operation = co_bed.ADD;

    collision_objects.push_back(co_bed);

    // planning_scene_interface.addCollisionObjects(collision_objects);

    // prompt("Press 'Next' in the RvizVisualToolsGui window to add more collision objects");
    // draw_title("Adding more collision objects");
    // moveit_visual_tools.trigger();

    // --- adding doctorOperation doctor with cylinder ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::msg::CollisionObject collision_object_doctor;
    collision_object_doctor.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object_doctor.id = "cylinder1_doctor";

    // shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.6;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.25;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::msg::Pose doctor_pose;
    doctor_pose.orientation.w = 1.0;
    doctor_pose.position.x = -0.2;
    doctor_pose.position.y = 0.65;
    doctor_pose.position.z = 0.05;

    collision_object_doctor.primitives.push_back(cylinder_primitive);
    collision_object_doctor.primitive_poses.push_back(doctor_pose);
    collision_object_doctor.operation = collision_object_doctor.ADD;

    collision_objects.push_back(collision_object_doctor);

    // --- adding patient with cylinder ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::msg::CollisionObject collision_object_patient;
    collision_object_patient.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object_patient.id = "cylinder2_patient";

    // shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.6;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.25;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::msg::Pose patient_pose;
    patient_pose.orientation = euler2Quaternion_deg(90, 0, 0);
    patient_pose.position.x = 0.5;
    patient_pose.position.y = 0.8;
    patient_pose.position.z = 0.0;

    collision_object_patient.primitives.push_back(cylinder_primitive);
    collision_object_patient.primitive_poses.push_back(patient_pose);
    collision_object_patient.operation = collision_object_patient.ADD;

    collision_objects.push_back(collision_object_patient);

    planning_scene_interface.addCollisionObjects(collision_objects);

    // --- adding doctorOperation doctor from stl ---
    shapes::Mesh *m_doctorOperation;
    shapes::Mesh *m_patient;

    if (loadingHumans)
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects4;

        moveit_msgs::msg::CollisionObject co_doctorOperation;
        co_doctorOperation.header.frame_id = move_group_interface.getPlanningFrame();
        co_doctorOperation.id = "doctorOperation_mesh";
        move_group_interface.getPlanningFrame();

        Eigen::Vector3d scale_doctorOperation(0.0008, 0.0008, 0.0008);
        m_doctorOperation = shapes::createMeshFromResource("package://moving_ur3e_ros2/collisionObjects/stls/human.STL", scale_doctorOperation);

        shape_msgs::msg::Mesh mesh_doctorOperation;
        shapes::ShapeMsg mesh_msg_doctorOperation;
        shapes::constructMsgFromShape(m_doctorOperation, mesh_msg_doctorOperation);
        mesh_doctorOperation = boost::get<shape_msgs::msg::Mesh>(mesh_msg_doctorOperation);

        geometry_msgs::msg::Pose docOperation_pose;
        docOperation_pose.position.x = -0.4;
        docOperation_pose.position.y = 0.5;
        docOperation_pose.position.z = -0.75;
        docOperation_pose.orientation = euler2Quaternion_deg(90, 0.0, 90);

        co_doctorOperation.meshes.push_back(mesh_doctorOperation);
        co_doctorOperation.mesh_poses.push_back(docOperation_pose);
        co_doctorOperation.operation = co_doctorOperation.ADD;

        collision_objects4.push_back(co_doctorOperation);

        planning_scene_interface.addCollisionObjects(collision_objects4);

        prompt("Press 'Next' in the RvizVisualToolsGui window to add more collision objects");
        draw_title("Adding more collision objects");
        moveit_visual_tools.trigger();

        // --- adding patient from stl ---

        std::vector<moveit_msgs::msg::CollisionObject> collision_objects5;

        moveit_msgs::msg::CollisionObject co_patient;
        co_patient.header.frame_id = move_group_interface.getPlanningFrame();
        co_patient.id = "patient_mesh";
        move_group_interface.getPlanningFrame();

        Eigen::Vector3d scale_patient(0.001, 0.001, 0.001);
        m_patient = shapes::createMeshFromResource("package://moving_ur3e_ros2/collisionObjects/stls/human.STL", scale_patient);

        shape_msgs::msg::Mesh mesh_patient;
        shapes::ShapeMsg mesh_msg_patient;
        shapes::constructMsgFromShape(m_patient, mesh_msg_patient);
        mesh_patient = boost::get<shape_msgs::msg::Mesh>(mesh_msg_patient);

        geometry_msgs::msg::Pose patient_pose;
        patient_pose.position.x = 0.2;
        patient_pose.position.y = 0.0;
        patient_pose.position.z = -0.2;
        patient_pose.orientation = euler2Quaternion_deg(0.0, 0.0, 0.0);

        co_patient.meshes.push_back(mesh_patient);
        co_patient.mesh_poses.push_back(patient_pose);
        co_patient.operation = co_patient.ADD;

        collision_objects5.push_back(co_patient);

        planning_scene_interface.addCollisionObjects(collision_objects5);

        prompt("Press 'Next' in the RvizVisualToolsGui window to add more collision objects");
        draw_title("Adding more collision objects");
        moveit_visual_tools.trigger();
    }

    // --- adding tray from stl ---

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects2;

    moveit_msgs::msg::CollisionObject co_tray;
    co_tray.header.frame_id = move_group_interface.getPlanningFrame();
    co_tray.id = "tray_mesh";
    move_group_interface.getPlanningFrame();

    Eigen::Vector3d scale_tray(0.0008, 0.0008, 0.0008);
    shapes::Mesh *m_tray = shapes::createMeshFromResource("package://moving_ur3e_ros2/collisionObjects/stls/emptyTray.STL", scale_tray);

    shape_msgs::msg::Mesh mesh_tray;
    shapes::ShapeMsg mesh_msg_tray;
    shapes::constructMsgFromShape(m_tray, mesh_msg_tray);
    mesh_tray = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tray);

    geometry_msgs::msg::Pose tray_pose;
    tray_pose.position.x = -0.3;
    tray_pose.position.y = -0.15;
    tray_pose.position.z = -0.75 + 0.02;
    tray_pose.orientation = euler2Quaternion_deg(90, 0.0, 0.0);

    co_tray.meshes.push_back(mesh_tray);
    co_tray.mesh_poses.push_back(tray_pose);
    co_tray.operation = co_tray.ADD;

    collision_objects2.push_back(co_tray);

    planning_scene_interface.addCollisionObjects(collision_objects2);

    prompt("Press 'Next' in the RvizVisualToolsGui window to add more collision objects");
    draw_title("Adding more collision objects");
    moveit_visual_tools.trigger();

    // --- adding emptyTray from stl ---

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects3;

    moveit_msgs::msg::CollisionObject co_emptyTray;
    co_emptyTray.header.frame_id = move_group_interface.getPlanningFrame();
    co_emptyTray.id = "emptyTray_mesh";
    move_group_interface.getPlanningFrame();

    Eigen::Vector3d scale_emptyTray(0.0008, 0.0008, 0.0008);
    shapes::Mesh *m_emptyTray = shapes::createMeshFromResource("package://moving_ur3e_ros2/collisionObjects/stls/emptyTray.STL", scale_emptyTray);

    shape_msgs::msg::Mesh mesh_emptyTray;
    shapes::ShapeMsg mesh_msg_emptyTray;
    shapes::constructMsgFromShape(m_emptyTray, mesh_msg_emptyTray);
    mesh_emptyTray = boost::get<shape_msgs::msg::Mesh>(mesh_msg_emptyTray);

    geometry_msgs::msg::Pose emptyTray_pose;
    emptyTray_pose.position.x = -0.6;
    emptyTray_pose.position.y = -0.3;
    emptyTray_pose.position.z = -0.75 + 0.02;
    emptyTray_pose.orientation = euler2Quaternion_deg(90, 0.0, 90);

    co_emptyTray.meshes.push_back(mesh_emptyTray);
    co_emptyTray.mesh_poses.push_back(emptyTray_pose);
    co_emptyTray.operation = co_emptyTray.ADD;

    collision_objects3.push_back(co_emptyTray);

    planning_scene_interface.addCollisionObjects(collision_objects3);

    // prompt("Press 'Next' in the RvizVisualToolsGui window to add a constraint from stl");
    // draw_title("Adding constraint from stl");
    // moveit_visual_tools.trigger();

    // ---------------------------- Moving to start pose ----------------------------

    // Set a start Pose
    auto const start_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 1.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 0.0;
        msg.position.x = -0.15;
        msg.position.y = -0.25;
        msg.position.z = 0.15;
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

    // --- adding constraint with cylinder ---
    if (visualizeArea)
    {
        // Now let's define a collision object ROS message for the robot to avoid.
        moveit_msgs::msg::CollisionObject cylider_constraint;
        cylider_constraint.header.frame_id = move_group_interface.getPlanningFrame();

        // The id of the object is used to identify it.
        cylider_constraint.id = "cylinder_constraint";

        // shape_msgs::SolidPrimitive cylinder_primitive;
        cylinder_primitive.type = primitive.CYLINDER;
        cylinder_primitive.dimensions.resize(2);
        cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.75; // 0.75
        cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 2.2;  // 0.2

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::msg::Pose constraint_pose;
        constraint_pose.orientation = euler2Quaternion_deg(90, 0, 0);
        constraint_pose.position.x = 0.0;
        constraint_pose.position.y = 0.0;
        constraint_pose.position.z = 0.0;

        cylider_constraint.primitives.push_back(cylinder_primitive);
        cylider_constraint.primitive_poses.push_back(constraint_pose);
        cylider_constraint.operation = cylider_constraint.ADD;
        collision_objects.push_back(cylider_constraint);

        planning_scene_interface.addCollisionObjects(collision_objects);
    }
    // ---------------------------- Adding a Cylinder as Constraint ----------------------------

    // Let's try the simple box constraints first!
    moveit_msgs::msg::PositionConstraint cylinder_constraint;
    cylinder_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
    cylinder_constraint.link_name = move_group_interface.getEndEffectorLink();

    // shape_msgs::msg::SolidPrimitive box;
    shape_msgs::msg::SolidPrimitive primitive_cylinder_constraint;
    primitive_cylinder_constraint.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // primitive_cylinder_constraint.type = primitive.CYLINDER;
    primitive_cylinder_constraint.dimensions.resize(2);
    primitive_cylinder_constraint.dimensions[primitive.CYLINDER_HEIGHT] = 1.75; // 0.75
    primitive_cylinder_constraint.dimensions[primitive.CYLINDER_RADIUS] = 2.2;  // 0.2
    cylinder_constraint.constraint_region.primitives.emplace_back(primitive_cylinder_constraint);

    geometry_msgs::msg::Pose constraint_cylinder_pose;
    constraint_cylinder_pose.position.x = 0.0;
    constraint_cylinder_pose.position.y = 0.0;
    constraint_cylinder_pose.position.z = 0.0;
    constraint_cylinder_pose.orientation = euler2Quaternion_deg(90, 0, 0);
    cylinder_constraint.constraint_region.primitive_poses.emplace_back(constraint_cylinder_pose);
    cylinder_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints cylinder_constraints;
    cylinder_constraints.position_constraints.emplace_back(cylinder_constraint);

    move_group_interface.setPathConstraints(cylinder_constraints);
    move_group_interface.setPlanningTime(10.0);

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
    // ---------------------------- Moving to target pose ----------------------------

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
        msg.position.z = 0.3; // 0.25
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

    // Pressing 'Next' to add a collision box
    // prompt("Press 'Next' in the RvizVisualToolsGui window to end demo");

    // ---------------------------- Adding a Wall ----------------------------
    /*
        draw_title("Adding Box");
        moveit_visual_tools.trigger();

        // Create collision object for the robot to avoid
        auto const collision_object2 = [frame_id = move_group_interface.getPlanningFrame()]
        {
            moveit_msgs::msg::CollisionObject collision_object2;
            collision_object2.header.frame_id = frame_id;
            collision_object2.id = "box1";
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

            collision_object2.primitives.push_back(primitive);
            collision_object2.primitive_poses.push_back(box_pose);
            collision_object2.operation = collision_object2.ADD;

            return collision_object2;
        }();

        // Add the collision object to the scene
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // bereits beim floor deklariert
        planning_scene_interface.applyCollisionObject(collision_object2);

        // Pressing 'Next' to plan around the collision box
        prompt("Press 'Next' in the RvizVisualToolsGui window to plan around the collision box");
    */
    // ---------------------------- Planning around the box ----------------------------
    /*
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
    */
    // Shutdown ROS
    rclcpp::shutdown(); // <--- This will cause the spin function in the thread to return
    spinner.join();     // <--- Join the thread before exiting
    return 0;
}