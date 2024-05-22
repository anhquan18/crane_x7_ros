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

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_path");

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
  move_group_arm.setMaxVelocityScalingFactor(0.9);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();

  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // ハンドを開く
  gripper_joint_values[0] = angles::from_degrees(90);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  q.setRPY(0, angles::from_degrees(180), 0);
  target_pose.orientation = tf2::toMsg(q);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  /*-------------------------物体を掴むまで-------------------------*/

  //中間位置
  target_pose.position.x = 0.15;
  target_pose.position.y = 0.00;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //物体の真上まで移動
  target_pose.position.x = 0.15;
  target_pose.position.y = -0.15;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //物体を掴みに行く
  target_pose.position.x = 0.15;
  target_pose.position.y = -0.15;
  target_pose.position.z = 0.1;
  waypoints.push_back(target_pose);

  //軌道生成
  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);
  waypoints.clear();
  
  // ハンドを閉じる
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //物体を持ち上げる
  target_pose.position.x = 0.15;
  target_pose.position.y = -0.15;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //-------------------------物体を点1まで移動-------------------------//
  //物体の真上まで移動
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //物体を掴みに行く
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.1;
  waypoints.push_back(target_pose);

  //軌道生成
  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);
  waypoints.clear();

  // ハンドを開く
  gripper_joint_values[0] = 90;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // ハンドを閉じる
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //物体を持ち上げる
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //-------------------------物体を点2まで移動-------------------------//
  //物体の真上まで移動
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //物体を掴みに行く
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0.1;
  waypoints.push_back(target_pose);

  //軌道生成
  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);
  waypoints.clear();

  // ハンドを開く
  gripper_joint_values[0] = 90;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // ハンドを閉じる
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //物体を持ち上げる
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //-------------------------物体を点3まで移動-------------------------//
  //物体の真上まで移動
  target_pose.position.x = 0.15;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  //物体を掴みに行く
  target_pose.position.x = 0.15;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.1;
  waypoints.push_back(target_pose);

  //軌道生成
  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);
  waypoints.clear();

  // ハンドを開く
  gripper_joint_values[0] = 90;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //物体を持ち上げる
  target_pose.position.x = 0.15;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.25;
  waypoints.push_back(target_pose);

  move_group_arm.computeCartesianPath(
    waypoints, eef_step, jump_threshold,
    trajectory);
  move_group_arm.execute(trajectory);

  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // ハンドを閉じる
  gripper_joint_values[0] = 0;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}
