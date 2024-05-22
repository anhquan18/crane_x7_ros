// Copyright 2022 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/constraints.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pilz_cartesian_path");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(0.2);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_arm.setPlannerId("CIRC");

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();

  //// SRDFに定義されている"home"の姿勢にする
  //move_group_arm.setNamedTarget("home");
  //move_group_arm.move();
  
  // ハンドを開く
  gripper_joint_values[0] = angles::from_degrees(30);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // ハンドを開く
  gripper_joint_values[0] = angles::from_degrees(90);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = "base_link";
  start_pose.pose.position.x = 0.2;
  start_pose.pose.position.y = 0.1;
  start_pose.pose.position.z = 0.2;
  start_pose.pose.orientation.w = 1.0;
  move_group_arm.setPoseTarget(start_pose);

  // Move to the start pose
  move_group_arm.setPoseTarget(start_pose, "crane_x7_gripper_base_link");
  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  bool success = (move_group_arm.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(move_group_arm_node->get_logger(), "Start pose plan computed successfully. Executing the plan.");
    move_group_arm.execute(start_plan);
  } else {
    RCLCPP_ERROR(move_group_arm_node->get_logger(), "Start pose planning failed.");
    rclcpp::shutdown();
    return 1;
  }

  // Define the intermediate pose (point on the arc)
  geometry_msgs::msg::PoseStamped interim_pose;
  interim_pose.header.frame_id = "base_link";
  interim_pose.pose.position.x = 0.2;
  interim_pose.pose.position.y = 0.0;
  interim_pose.pose.position.z = 0.3;
  interim_pose.pose.orientation.w = 1.0;

   // Define the goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_link";
  goal_pose.pose.position.x = 0.2;
  goal_pose.pose.position.y = -0.1;
  goal_pose.pose.position.z = 0.2;
  goal_pose.pose.orientation.w = 1.0;

  // Set the start state to the current state
  move_group_arm.setStartStateToCurrentState();

  // Define the path constraints using the interim pose
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "interim";
  moveit_msgs::msg::PositionConstraint position_constraint;
  position_constraint.header.frame_id = interim_pose.header.frame_id;
  position_constraint.link_name = "crane_x7_gripper_base_link"; // Replace with your robot's end-effector link name
  position_constraint.constraint_region.primitives.resize(1);
  position_constraint.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  position_constraint.constraint_region.primitives[0].dimensions.resize(1);
  position_constraint.constraint_region.primitives[0].dimensions[0] = 0.01; // Radius of the sphere
  position_constraint.constraint_region.primitive_poses.push_back(interim_pose.pose);
  position_constraint.weight = 1.0;
  path_constraints.position_constraints.push_back(position_constraint);
  move_group_arm.setPathConstraints(path_constraints);

  // Plan and execute the CIRC motion
  move_group_arm.setPoseTarget(goal_pose, "crane_x7_gripper_base_link");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  success = (move_group_arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    RCLCPP_INFO(move_group_arm_node->get_logger(), "Path computed successfully. Executing the plan.");
    move_group_arm.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(move_group_arm_node->get_logger(), "Path planning failed.");
  }

  /*
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);
  */

  //// SRDFに定義されている"home"の姿勢にする
  //move_group_arm.setNamedTarget("home");
  //move_group_arm.move();

  // ハンドを開く
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}
