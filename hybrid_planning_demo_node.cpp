/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr
   Description: A demonstration that re-plans around a moving box.
 */

#include <thread>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// für collision objects as .stls
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

using namespace std::chrono_literals;
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("test_hybrid_planning_client");
}  // namespace

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

class HybridPlanningDemo
{
public:
  HybridPlanningDemo(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    std::string hybrid_planning_action_name = "";
    if (node_->has_parameter("hybrid_planning_action_name"))
    {
      node_->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
      std::exit(EXIT_FAILURE);
    }
    hp_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(node_, hybrid_planning_action_name);

    collision_object_mir_.header.frame_id = "panda_link0";
    collision_object_mir_.id = "box1";

    collision_cylinder_doctor.header.frame_id = "panda_link0";
    collision_cylinder_doctor.id = "collision_doctor";

    collision_cylinder_patient.header.frame_id = "panda_link0";
    collision_cylinder_patient.id = "collision_patient";


    // collision_object_2_.header.frame_id = "panda_link0";
    // collision_object_2_.id = "box2";

    appearing_collision_box.header.frame_id = "panda_link0";
    appearing_collision_box.id = "box3";

    collision_bed.header.frame_id = "panda_link0";
    collision_bed.id = "bed";

    float x_value_mir = 1.0;
    float y_value_mir = 0.6;
    float z_value_mir = 0.75;

    box_1_.type = box_1_.BOX;
    box_1_.dimensions = { x_value_mir, y_value_mir, z_value_mir };

    box_appearing_.type = box_appearing_.BOX;
    box_appearing_.dimensions = { 1.0, 0.4, 0.01 }; // partial block --> replanning
    //box_appearing_.dimensions = { 2.0, 2.0, 0.01 }; // complete block --> stops the robot

    box_bed_.type = box_bed_.BOX;
    box_bed_.dimensions = { 0.6, 1.6, z_value_mir };

    cylinder_1_.type = cylinder_1_.CYLINDER;
    cylinder_1_.dimensions = {1.6, 0.25 };

    cylinder_2_.type = cylinder_2_.CYLINDER;
    cylinder_2_.dimensions = {1.6, 0.25 };
    
    // Add new collision object as soon as global trajectory is available.
    global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
        "global_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::MotionPlanResponse::ConstSharedPtr& /* unused */) {
          // Remove old collision objects
          // collision_object_mir_.operation = collision_object_mir_.REMOVE;

          // Add new collision objects
          // geometry_msgs::msg::Pose box_pose_2;
          // box_pose_2.position.x = 0.2;
          // box_pose_2.position.y = 0.4;
          // box_pose_2.position.z = 0.95;

          // collision_object_2_.primitives.push_back(box_appearing_);
          // collision_object_2_.primitive_poses.push_back(box_pose_2);
          // collision_object_2_.operation = collision_object_2_.ADD;

          geometry_msgs::msg::Pose box_pose_apprearing;
          box_pose_apprearing.position.x = 0.11;
          box_pose_apprearing.position.y = 0.18;
          box_pose_apprearing.position.z = 0.37;
          box_pose_apprearing.orientation = euler2Quaternion_deg(90.0, 0.0, 0.0);

          appearing_collision_box.primitives.push_back(box_appearing_);
          appearing_collision_box.primitive_poses.push_back(box_pose_apprearing);
          appearing_collision_box.operation = appearing_collision_box.ADD;

          // Add object to planning scene
          {  // Lock PlanningScene
            planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
            // scene->processCollisionObjectMsg(collision_object_2_);
            scene->processCollisionObjectMsg(appearing_collision_box);
            // scene->processCollisionObjectMsg(collision_object_mir_);
            //scene->processCollisionObjectMsg(collision_cylinder_doctor);
            //scene->processCollisionObjectMsg(collision_cylinder_patient);
          }  // Unlock PlanningScene
        });
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize Planning Scene Monitor");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer_, "planning_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(LOGGER, "The planning scene was not retrieved!");
      return;
    }
    else
    {
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->providePlanningSceneService();  // let RViz display query PlanningScene
      planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5))
    {
      RCLCPP_ERROR(LOGGER, "Timeout when waiting for /joint_states updates. Is the robot running?");
      return;
    }

    if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
      return;
    }

    // Pose for MIR
    float x_value_mir = 1.0;
    float y_value_mir = 0.6;
    float z_value_mir = 0.75;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -x_value_mir / 4 - 0.1;  // 0.0
    box_pose.position.y = -y_value_mir / 4;        // 0.0
    box_pose.position.z =
        -(z_value_mir / 2) - 0.01;  // -0.01 // -0.01 -> safty abstand für simulation damit keine collisions entstehen

    // geometry_msgs::msg::Pose box_pose;
    // box_pose.position.x = 0.4;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = 0.85;
    
    // Pose for doctor
    geometry_msgs::msg::Pose doctor_pose;
    doctor_pose.orientation.w = 1.0;
    doctor_pose.position.x = -0.2;
    doctor_pose.position.y = 0.65;
    doctor_pose.position.z = 0.05;
    
    // Pose for patient
    geometry_msgs::msg::Pose patient_pose;
    patient_pose.orientation = euler2Quaternion_deg(90, 0, 0);
    patient_pose.position.x = 0.5;
    patient_pose.position.y = 0.8;
    patient_pose.position.z = 0.0;

    // Pose for bed
    geometry_msgs::msg::Pose bed_pose;
    bed_pose.position.x = 0.5;
    bed_pose.position.y = 0.8;
    bed_pose.position.z = 0.0 - (z_value_mir/2);
    bed_pose.orientation.w = 1.0;
    //bed_pose.orientation = euler2Quaternion_deg(90, 0, 0);

    //Eigen::Vector3d scale_bed(0.01, 0.01, 0.01);
    //shapes::Mesh *m_bed = shapes::createMeshFromResource("/home/sebi/ws_moveit2/src/moving_ur3e_ros2/collisionObjects/stls/medicalBed.stl", scale_bed);

    //shape_msgs::msg::Mesh mesh_bed;
    //shapes::ShapeMsg mesh_msg_bed;
    //shapes::constructMsgFromShape(m_bed, mesh_msg_bed);
    //mesh_bed = boost::get<shape_msgs::msg::Mesh>(mesh_msg_bed);

    collision_object_mir_.primitives.push_back(box_1_);
    collision_object_mir_.primitive_poses.push_back(box_pose);
    collision_object_mir_.operation = collision_object_mir_.ADD;

    collision_cylinder_doctor.primitives.push_back(cylinder_1_);
    collision_cylinder_doctor.primitive_poses.push_back(doctor_pose);
    collision_cylinder_doctor.operation = collision_cylinder_doctor.ADD;

    collision_cylinder_patient.primitives.push_back(cylinder_2_);
    collision_cylinder_patient.primitive_poses.push_back(patient_pose);
    collision_cylinder_patient.operation = collision_cylinder_patient.ADD;

    //collision_bed.meshes.push_back(box_bed_);
    //collision_bed.mesh_poses.push_back(bed_pose);
    collision_bed.primitives.push_back(box_bed_);
    collision_bed.primitive_poses.push_back(bed_pose);
    collision_bed.operation = collision_bed.ADD;


    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
      scene->processCollisionObjectMsg(collision_object_mir_);
      scene->processCollisionObjectMsg(collision_cylinder_doctor);
      scene->processCollisionObjectMsg(collision_cylinder_patient);
      scene->processCollisionObjectMsg(collision_bed);
    }  // Unlock PlanningScene

    RCLCPP_INFO(LOGGER, "Wait 2s for the collision object");
    rclcpp::sleep_for(2s);

    // Setup motion planning goal taken from motion_planning_api tutorial
    const std::string planning_group = "panda_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

    // Configure a valid robot state
    robot_state->setToDefaultValues(joint_model_group, "ready");
    robot_state->update();
    // Lock the planning scene as briefly as possible
    {
      planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      locked_planning_scene->setCurrentState(*robot_state);
    }

    // Create desired motion goal
    moveit_msgs::msg::MotionPlanRequest goal_motion_request;

    moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
    goal_motion_request.group_name = planning_group;
    goal_motion_request.num_planning_attempts = 10;
    goal_motion_request.max_velocity_scaling_factor = 0.1;
    goal_motion_request.max_acceleration_scaling_factor = 0.1;
    goal_motion_request.allowed_planning_time = 2.0;
    goal_motion_request.planner_id = "ompl";
    goal_motion_request.pipeline_id = "ompl";

    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { 0.0, -0.5236, 0.6981, -2.234, 0.3491, 1.78, 1.3963 };
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    goal_motion_request.goal_constraints.resize(1);
    goal_motion_request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    // Create Hybrid Planning action request
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    sequence_item.blend_radius = 0.0;  // Single goal

    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);

    auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
    goal_action_request.planning_group = planning_group;
    goal_action_request.motion_sequence = sequence_request;

    auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(LOGGER, "Hybrid planning goal succeeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
              return;
            default:
              RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
              return;
              RCLCPP_INFO(LOGGER, "Hybrid planning result received");
          }
        };
    send_goal_options.feedback_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr& /*unused*/,
           const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback>& feedback) {
          RCLCPP_INFO_STREAM(LOGGER, feedback->feedback);
        };

    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_action_request, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr hp_action_client_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit_msgs::msg::CollisionObject collision_object_mir_;
  // moveit_msgs::msg::CollisionObject collision_object_2_;
  moveit_msgs::msg::CollisionObject appearing_collision_box;
  moveit_msgs::msg::CollisionObject collision_cylinder_doctor;
  moveit_msgs::msg::CollisionObject collision_cylinder_patient;
  moveit_msgs::msg::CollisionObject collision_bed;
  shape_msgs::msg::SolidPrimitive box_1_;
  shape_msgs::msg::SolidPrimitive box_bed_;
  shape_msgs::msg::SolidPrimitive box_appearing_;
  shape_msgs::msg::SolidPrimitive cylinder_1_;
  shape_msgs::msg::SolidPrimitive cylinder_2_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hybrid_planning_test_node", "", node_options);

  HybridPlanningDemo demo(node);
  std::thread run_demo([&demo]() {
    // This sleep isn't necessary but it gives humans time to process what's going on
    rclcpp::sleep_for(5s);
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();
  rclcpp::shutdown();
  return 0;
}
