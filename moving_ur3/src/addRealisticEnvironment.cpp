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

// für collision objects as .stls
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// euler 2 quaternions
#include <tf/LinearMath/Quaternion.h>
#include <cmath> // für PI
#include <tf/transform_datatypes.h>

#define loadingHumans true

geometry_msgs::Quaternion euler2Quaternion_deg(double roll, double pitch, double yaw)
{

    double roll_rad = roll * M_PI / 180;
    double pitch_rad = pitch * M_PI / 180;
    double yaw_rad = yaw * M_PI / 180;

    // Convert Euler angles to quaternion
    tf::Quaternion tf_quaternion = tf::createQuaternionFromRPY(roll_rad, pitch_rad, yaw_rad);

    // Convert tf::Quaternion to geometry_msgs::Quaternion
    geometry_msgs::Quaternion quaternion_msg;
    tf::quaternionTFToMsg(tf_quaternion, quaternion_msg);

    return quaternion_msg;
}

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

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // ROS_INFO_NAMED("tutorial", " ----------- Getting Basic Information -----------");
    std::cout << "\n----------- Getting Basic Information -----------\n"
              << std::endl;

    // printing name of the reference frame for this robot.
    ROS_INFO_NAMED("realistic environment", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // printing the name of the end-effector link for this group.
    ROS_INFO_NAMED("realistic environment", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("realistic environment", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // ------------------------------------------- Attaching objects to the robot -------------------------------------------

    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::SolidPrimitive primitive;
    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.04;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    // defining a frame/pose for the cylinder
    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.025;

    // Adding object to the world
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Attaching Object to given link
    ROS_INFO_NAMED("realistic environment", "Attach the object to the robot");
    move_group_interface.attachObject(object_to_attach.id, "tool0");

    // ------------------------------------------- Adding objects to the world -------------------------------------------

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // --- Adding MIR ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::CollisionObject collision_object;
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
    geometry_msgs::Pose box_pose;
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
    ROS_INFO_NAMED("realistic environment", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    // --- adding operation bed from stl ---

    Eigen::Vector3d scale_bed(0.01, 0.01, 0.01);
    moveit_msgs::CollisionObject co_bed;
    co_bed.header.frame_id = move_group_interface.getPlanningFrame();
    co_bed.id = "bed_mesh";
    move_group_interface.getPlanningFrame();
    shapes::Mesh *m_bed = shapes::createMeshFromResource("package://moving_ur3/collisionObjects/Medical Operating Table.stl", scale_bed);
    ROS_INFO("operation bed loaded");

    shape_msgs::Mesh mesh_bed;
    shapes::ShapeMsg mesh_msg_bed;
    shapes::constructMsgFromShape(m_bed, mesh_msg_bed);
    mesh_bed = boost::get<shape_msgs::Mesh>(mesh_msg_bed);

    co_bed.meshes.resize(1);
    co_bed.mesh_poses.resize(1);
    co_bed.meshes[0] = mesh_bed;
    co_bed.mesh_poses[0].orientation = euler2Quaternion_deg(90, 0, 90);
    co_bed.mesh_poses[0].position.x = 0.5;
    co_bed.mesh_poses[0].position.y = 0.8;
    co_bed.mesh_poses[0].position.z = 0.0;

    co_bed.meshes.push_back(mesh_bed);
    co_bed.mesh_poses.push_back(co_bed.mesh_poses[0]);
    co_bed.operation = co_bed.ADD;

    collision_objects.push_back(co_bed);
    ROS_INFO("Operation bed added into the world");

    // --- adding doctorOperation doctor from stl ---
    shapes::Mesh *m_doctorOperation;
    shapes::Mesh *m_patient;

    if (loadingHumans)
    {
        Eigen::Vector3d scale_doctorOperation(0.0008, 0.0008, 0.0008);
        moveit_msgs::CollisionObject co_doctorOperation;
        co_doctorOperation.header.frame_id = move_group_interface.getPlanningFrame();
        co_doctorOperation.id = "doctorOperation_mesh";
        move_group_interface.getPlanningFrame();
        m_doctorOperation = shapes::createMeshFromResource("package://moving_ur3/collisionObjects/human.STL", scale_doctorOperation);
        ROS_INFO("operation doctorOperation loaded");

        shape_msgs::Mesh mesh_doctorOperation;
        shapes::ShapeMsg mesh_msg_doctorOperation;
        shapes::constructMsgFromShape(m_doctorOperation, mesh_msg_doctorOperation);
        mesh_doctorOperation = boost::get<shape_msgs::Mesh>(mesh_msg_doctorOperation);

        co_doctorOperation.meshes.resize(1);
        co_doctorOperation.mesh_poses.resize(1);
        co_doctorOperation.meshes[0] = mesh_doctorOperation;
        co_doctorOperation.mesh_poses[0].orientation = euler2Quaternion_deg(90, 0, 90);
        co_doctorOperation.mesh_poses[0].position.x = -0.4;
        co_doctorOperation.mesh_poses[0].position.y = 0.5;
        co_doctorOperation.mesh_poses[0].position.z = -0.75;

        co_doctorOperation.meshes.push_back(mesh_doctorOperation);
        co_doctorOperation.mesh_poses.push_back(co_doctorOperation.mesh_poses[0]);
        co_doctorOperation.operation = co_doctorOperation.ADD;

        collision_objects.push_back(co_doctorOperation);
        ROS_INFO("Operation doctorOperation added into the world");

        // --- adding patient from stl ---

        Eigen::Vector3d scale_patient(0.001, 0.001, 0.001);
        moveit_msgs::CollisionObject co_patient;
        co_patient.header.frame_id = move_group_interface.getPlanningFrame();
        co_patient.id = "patient_mesh";
        move_group_interface.getPlanningFrame();
        m_patient = shapes::createMeshFromResource("package://moving_ur3/collisionObjects/human.STL", scale_patient);
        ROS_INFO("operation patient loaded");

        shape_msgs::Mesh mesh_patient;
        shapes::ShapeMsg mesh_msg_patient;
        shapes::constructMsgFromShape(m_patient, mesh_msg_patient);
        mesh_patient = boost::get<shape_msgs::Mesh>(mesh_msg_patient);

        co_patient.meshes.resize(1);
        co_patient.mesh_poses.resize(1);
        co_patient.meshes[0] = mesh_patient;
        co_patient.mesh_poses[0].orientation = euler2Quaternion_deg(0, 0, 0);
        co_patient.mesh_poses[0].position.x = 0.2;
        co_patient.mesh_poses[0].position.y = 0;
        co_patient.mesh_poses[0].position.z = -0.2;

        co_patient.meshes.push_back(mesh_patient);
        co_patient.mesh_poses.push_back(co_patient.mesh_poses[0]);
        co_patient.operation = co_patient.ADD;

        collision_objects.push_back(co_patient);
        ROS_INFO("Operation patient added into the world");
    }

    // --- adding tray from stl ---

    Eigen::Vector3d scale_tray(0.0008, 0.0008, 0.0008);
    moveit_msgs::CollisionObject co_tray;
    co_tray.header.frame_id = move_group_interface.getPlanningFrame();
    co_tray.id = "tray_mesh";
    move_group_interface.getPlanningFrame();
    shapes::Mesh *m_tray = shapes::createMeshFromResource("package://moving_ur3/collisionObjects/surgicalTray.STL", scale_tray);
    ROS_INFO("operation tray loaded");

    shape_msgs::Mesh mesh_tray;
    shapes::ShapeMsg mesh_msg_tray;
    shapes::constructMsgFromShape(m_tray, mesh_msg_tray);
    mesh_tray = boost::get<shape_msgs::Mesh>(mesh_msg_tray);

    co_tray.meshes.resize(1);
    co_tray.mesh_poses.resize(1);
    co_tray.meshes[0] = mesh_tray;
    co_tray.mesh_poses[0].orientation = euler2Quaternion_deg(90, 0, 0);
    co_tray.mesh_poses[0].position.x = -0.3;
    co_tray.mesh_poses[0].position.y = -0.15;
    co_tray.mesh_poses[0].position.z = -0.75 + 0.02;

    co_tray.meshes.push_back(mesh_tray);
    co_tray.mesh_poses.push_back(co_tray.mesh_poses[0]);
    co_tray.operation = co_tray.ADD;

    collision_objects.push_back(co_tray);
    ROS_INFO("Operation tray added into the world");

    // --- adding emptyTray from stl ---

    Eigen::Vector3d scale_emptyTray(0.0008, 0.0008, 0.0008);
    moveit_msgs::CollisionObject co_emptyTray;
    co_emptyTray.header.frame_id = move_group_interface.getPlanningFrame();
    co_emptyTray.id = "emptyTray_mesh";
    move_group_interface.getPlanningFrame();
    shapes::Mesh *m_emptyTray = shapes::createMeshFromResource("package://moving_ur3/collisionObjects/emptyTray.STL", scale_emptyTray);
    ROS_INFO("operation emptyTray loaded");

    shape_msgs::Mesh mesh_emptyTray;
    shapes::ShapeMsg mesh_msg_emptyTray;
    shapes::constructMsgFromShape(m_emptyTray, mesh_msg_emptyTray);
    mesh_emptyTray = boost::get<shape_msgs::Mesh>(mesh_msg_emptyTray);

    co_emptyTray.meshes.resize(1);
    co_emptyTray.mesh_poses.resize(1);
    co_emptyTray.meshes[0] = mesh_emptyTray;
    co_emptyTray.mesh_poses[0].orientation = euler2Quaternion_deg(90, 0, 90);
    co_emptyTray.mesh_poses[0].position.x = -0.6;
    co_emptyTray.mesh_poses[0].position.y = -0.3;
    co_emptyTray.mesh_poses[0].position.z = -0.75 + 0.02;

    co_emptyTray.meshes.push_back(mesh_emptyTray);
    co_emptyTray.mesh_poses.push_back(co_emptyTray.mesh_poses[0]);
    co_emptyTray.operation = co_emptyTray.ADD;

    collision_objects.push_back(co_emptyTray);
    ROS_INFO("Operation emptyTray added into the world");


    // adding all collision objects to the world
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(30.0).sleep();  // Wait for 30 seconds

    ros::shutdown();
    delete m_bed;
    delete m_doctorOperation;
    delete m_patient;
    delete m_tray;
    delete m_emptyTray;
    return 0;
}