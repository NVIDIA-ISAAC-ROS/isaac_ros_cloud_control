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

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__DOCKING_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__DOCKING_HANDLER_HPP_

#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vda5050_action_handler/vda5050_action_handler.hpp"

namespace isaac_ros
{
namespace mission_client
{
class DockingHandler : public Vda5050ActionHandlerBase
{
public:
  using DockAction =
    nav2_msgs::action::DockRobot;
  using GoalHandleDockAction = rclcpp_action::ClientGoalHandle<DockAction>;

  using UndockAction =
    nav2_msgs::action::UndockRobot;
  using GoalHandleUndockAction = rclcpp_action::ClientGoalHandle<UndockAction>;

  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;

private:
  void ExecuteDock(const vda5050_msgs::msg::Action & vda5050_action);
  void ExecuteUndock(const vda5050_msgs::msg::Action & vda5050_action);

  rclcpp_action::Client<DockAction>::SharedPtr dock_client_;
  rclcpp_action::Client<UndockAction>::SharedPtr undock_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr switch_client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr docking_lifecycle_manager_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr localization_client_;
  // Executor for service reqeust callbacks
  rclcpp::executors::SingleThreadedExecutor executor_;
  // Callback group for service reqeust callbacks
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  bool use_switch_;
};
}  // namespace mission_client
}  // namespace isaac_ros
#endif  // VDA5050_ACTION_HANDLER_PLUGINS__DOCKING_HANDLER_HPP_
