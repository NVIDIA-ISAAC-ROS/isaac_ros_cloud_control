// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__GR00T_POLICY_ACTION_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__GR00T_POLICY_ACTION_HANDLER_HPP_

#include <memory>
#include <string>

#include "vda5050_action_handler/vda5050_action_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "isaac_ros_cloud_control_interface/action/humanoid_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vda5050_msgs/msg/action.hpp"

namespace isaac_ros
{
namespace mission_client
{

using HumanoidTask = isaac_ros_cloud_control_interface::action::HumanoidTask;
using HumanoidTaskGoalHandle = rclcpp_action::ClientGoalHandle<HumanoidTask>;

class Gr00tPolicyActionHandler : public Vda5050ActionHandlerBase
{
public:
  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;
  void Cancel(const std::string & action_id) override;

private:
  // Action client for communicating with GR00T policy server
  rclcpp_action::Client<HumanoidTask>::SharedPtr action_client_;

  // Callback group for action client
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  HumanoidTaskGoalHandle::SharedPtr current_goal_handle_;
  std::string current_action_id_;

  // Action server name/topic
  std::string action_server_name_;

  // Default timeout for actions
  double default_timeout_;

  // Helper methods
  HumanoidTask::Goal buildHumanoidTaskGoal(const vda5050_msgs::msg::Action & vda5050_action);
  std::string extractTaskCategory(const vda5050_msgs::msg::Action & vda5050_action);
  std::string extractTaskId(const vda5050_msgs::msg::Action & vda5050_action);
  std::string extractLanguageInstruction(const vda5050_msgs::msg::Action & vda5050_action);
  double extractTimeout(const vda5050_msgs::msg::Action & vda5050_action);
  geometry_msgs::msg::PoseStamped extractLocomotionTarget(
    const vda5050_msgs::msg::Action & vda5050_action);
  std::string extractParameters(const vda5050_msgs::msg::Action & vda5050_action);
};

}  // namespace mission_client
}  // namespace isaac_ros

#endif  // VDA5050_ACTION_HANDLER_PLUGINS__GR00T_POLICY_ACTION_HANDLER_HPP_
