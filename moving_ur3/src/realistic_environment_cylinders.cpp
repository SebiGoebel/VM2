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

#define loadingHumans false

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
    ROS_INFO_NAMED("realistic environment", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // printing the name of the end-effector link for this group.
    ROS_INFO_NAMED("realistic environment", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("realistic environment", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo"); // stops code until "Next" is pressed

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

    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Wait for MoveGroup to receive and process the attached collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

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

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

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
    // co_bed.mesh_poses[0].orientation.w = 1.0;
    // co_bed.mesh_poses[0].orientation.x = 0.0;
    // co_bed.mesh_poses[0].orientation.y = 0.0;
    // co_bed.mesh_poses[0].orientation.z = 0.0;
    co_bed.mesh_poses[0].position.x = 0.5;
    co_bed.mesh_poses[0].position.y = 0.8;
    co_bed.mesh_poses[0].position.z = 0.0;

    co_bed.meshes.push_back(mesh_bed);
    co_bed.mesh_poses.push_back(co_bed.mesh_poses[0]);
    co_bed.operation = co_bed.ADD;

    // std::vector<moveit_msgs::CollisionObject> vec;
    collision_objects.push_back(co_bed);
    // vec.push_back(co_bed);
    ROS_INFO("Operation bed added into the world");

    // --- adding doctorOperation doctor with cylinder ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::CollisionObject collision_object_doctor;
    collision_object_doctor.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object_doctor.id = "cylinder1_doctor";

    //shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.6;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.25;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose doctor_pose;
    doctor_pose.orientation.w = 1.0;
    doctor_pose.position.x = -0.2;
    doctor_pose.position.y = 0.65;
    doctor_pose.position.z = 0.05;

    collision_object_doctor.primitives.push_back(cylinder_primitive);
    collision_object_doctor.primitive_poses.push_back(doctor_pose);
    collision_object_doctor.operation = collision_object_doctor.ADD;

    collision_objects.push_back(collision_object_doctor);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("realistic environment", "Add an object into the world");
    //planning_scene_interface.addCollisionObjects(collision_objects);

    // --- adding patient with cylinder ---

    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::CollisionObject collision_object_patient;
    collision_object_patient.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object_patient.id = "cylinder2_patient";

    //shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 1.6;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.25;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose patient_pose;
    patient_pose.orientation = euler2Quaternion_deg(90, 0, 0);
    patient_pose.position.x = 0.5;
    patient_pose.position.y = 0.8;
    patient_pose.position.z = 0.0;

    collision_object_patient.primitives.push_back(cylinder_primitive);
    collision_object_patient.primitive_poses.push_back(patient_pose);
    collision_object_patient.operation = collision_object_patient.ADD;

    collision_objects.push_back(collision_object_patient);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("realistic environment", "Add an object into the world");
    //planning_scene_interface.addCollisionObjects(collision_objects);

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
    // co_tray.mesh_poses[0].orientation.w = 1.0;
    // co_tray.mesh_poses[0].orientation.x = 0.0;
    // co_tray.mesh_poses[0].orientation.y = 0.0;
    // co_tray.mesh_poses[0].orientation.z = 0.0;
    co_tray.mesh_poses[0].position.x = -0.3;
    co_tray.mesh_poses[0].position.y = -0.15;
    co_tray.mesh_poses[0].position.z = -0.75 + 0.02;

    co_tray.meshes.push_back(mesh_tray);
    co_tray.mesh_poses.push_back(co_tray.mesh_poses[0]);
    co_tray.operation = co_tray.ADD;

    // std::vector<moveit_msgs::CollisionObject> vec;
    collision_objects.push_back(co_tray);
    // vec.push_back(co_tray);
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
    // co_emptyTray.mesh_poses[0].orientation.w = 1.0;
    // co_emptyTray.mesh_poses[0].orientation.x = 0.0;
    // co_emptyTray.mesh_poses[0].orientation.y = 0.0;
    // co_emptyTray.mesh_poses[0].orientation.z = 0.0;
    co_emptyTray.mesh_poses[0].position.x = -0.6;
    co_emptyTray.mesh_poses[0].position.y = -0.3;
    co_emptyTray.mesh_poses[0].position.z = -0.75 + 0.02;

    co_emptyTray.meshes.push_back(mesh_emptyTray);
    co_emptyTray.mesh_poses.push_back(co_emptyTray.mesh_poses[0]);
    co_emptyTray.operation = co_emptyTray.ADD;

    // std::vector<moveit_msgs::CollisionObject> vec;
    collision_objects.push_back(co_emptyTray);
    // vec.push_back(co_emptyTray);
    ROS_INFO("Operation emptyTray added into the world");



    planning_scene_interface.addCollisionObjects(collision_objects);

    // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // ------------------------------------------- Detaching and Removing Objects -------------------------------------------

    // detaching the cylinder
    ROS_INFO_NAMED("realistic environment", "Detach the cylinder from the robot");
    move_group_interface.detachObject(object_to_attach.id);

    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Wait for MoveGroup to receive and process the attached collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

    // remove objects from world
    ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
    std::vector<std::string> object_ids;
    for (int i = 0; i < collision_objects.size(); i++)
    {
        object_ids.push_back(collision_objects.at(i).id);
    }
    // object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Wait for MoveGroup to receive and process the attached collision object message
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

    ros::shutdown();
    delete m_bed;
    delete m_tray;
    delete m_emptyTray;
    return 0;
}