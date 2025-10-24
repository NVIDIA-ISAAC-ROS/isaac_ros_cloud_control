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

#include <algorithm>
#include <regex>
#include <sstream>
#include <unordered_map>

#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vda5050_action_handler_plugins/docking_handler.hpp"
#include "vda5050_msgs/msg/action_state.hpp"

namespace isaac_ros
{
namespace mission_client
{
namespace
{
constexpr char kDockType[] = "dock_type";
constexpr char kDockPose[] = "dock_pose";
constexpr char kTriggerGlobalLocalization[] = "trigger_global_localization";
constexpr int kTimeout = 5;

std::vector<std::string> split(const std::string & s, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Check the format of dock_pose parameter
bool isValidDockPose(const std::string & str)
{
  std::regex pattern(R"([-+]?\d*\.?\d+,\s*[-+]?\d*\.?\d+,\s*[-+]?\d*\.?\d+)");
  return std::regex_match(str, pattern);
}

// Convert string to lowercase and return true is the lowercase string is 'true'
bool stringToBool(const std::string & s)
{
  std::string lower_str = s;
  std::transform(
    lower_str.begin(), lower_str.end(), lower_str.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return lower_str == "true" || lower_str == "1";
}
}  // namespace

void DockingHandler::Initialize(
  isaac_ros::mission_client::Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  RCLCPP_INFO(rclcpp::get_logger("DockingHandler"), "Initializing DockingHandler");
  client_node_ = client_node;

  callback_group_ =
    client_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(callback_group_, client_node_->get_node_base_interface());

  dock_client_ = rclcpp_action::create_client<DockAction>(client_node_, "dock_robot");
  undock_client_ = rclcpp_action::create_client<UndockAction>(client_node_, "undock_robot");
  switch_client_ = client_node_->create_client<std_srvs::srv::SetBool>(
    "switch", rclcpp::ServicesQoS(), callback_group_);
  docking_lifecycle_manager_client_ =
    client_node_->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
      "lifecycle_manager_docking/manage_nodes",
      rclcpp::ServicesQoS(),
      callback_group_);

  use_switch_ = config["use_switch"].as<bool>();
}

void DockingHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  RCLCPP_INFO(rclcpp::get_logger("DockingHandler"), "Handling action: %s",
        vda5050_action.action_type.c_str());
  if (vda5050_action.action_type == "dock_robot") {
    ExecuteDock(vda5050_action);
  } else if (vda5050_action.action_type == "undock_robot") {
    ExecuteUndock(vda5050_action);
  }
}

void DockingHandler::ExecuteDock(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  // Enable docking server
  auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::RESUME;
  auto server_result = docking_lifecycle_manager_client_->async_send_request(server_request);
  if (executor_.spin_until_future_complete(server_result, std::chrono::seconds(kTimeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Failed to start docking server");
    RCLCPP_ERROR(client_node_->get_logger(), "Failed to start docking server");
    return;
  }

  if (use_switch_) {
  // Enable switch
    auto switch_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    switch_request->data = true;
    auto switch_result = switch_client_->async_send_request(switch_request);
    if (executor_.spin_until_future_complete(switch_result, std::chrono::seconds(kTimeout)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Failed to enable switch");
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to enable switch");
      return;
    }
  }

  // Prepare dock_robot goal and callbacks
  auto send_goal_options =
    rclcpp_action::Client<DockAction>::SendGoalOptions();
  // Prepare dock_robot parameters and callbacks
  send_goal_options.goal_response_callback =
    [this, vda5050_action](
    const rclcpp_action::ClientGoalHandle<DockAction>::SharedPtr &
    goal) {
      client_node_->ActionResponseCallback<DockAction>(goal, vda5050_action);
    };
  send_goal_options.result_callback =
    [this, vda5050_action](const GoalHandleDockAction::WrappedResult & result) {
      std::string result_description = "";
      switch (result.result->error_code) {
        case DockAction::Result::UNKNOWN:
          result_description = "Unknown";
          break;
        case DockAction::Result::DOCK_NOT_IN_DB:
          result_description = "Dock not in DB";
          break;
        case DockAction::Result::DOCK_NOT_VALID:
          result_description = "Dock not valid";
          break;
        case DockAction::Result::FAILED_TO_STAGE:
          result_description = "Failed to stage";
          break;
        case DockAction::Result::FAILED_TO_DETECT_DOCK:
          result_description = "Failed to detect dock";
          break;
        case DockAction::Result::FAILED_TO_CONTROL:
          result_description = "Failed to control";
          break;
        case DockAction::Result::FAILED_TO_CHARGE:
          result_description = "Failed to charge";
          break;
        default:
          break;
      }
      client_node_->ActionResultCallback<GoalHandleDockAction::WrappedResult>(vda5050_action,
          result, result.result->success, result_description);

      // Disable switch
      if (use_switch_) {
        auto switch_request = std::make_shared<std_srvs::srv::SetBool::Request>();
        switch_request->data = false;
        switch_client_->async_send_request(switch_request);
      }

      // Disable docking server
      auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
      docking_lifecycle_manager_client_->async_send_request(server_request);
    };
  auto goal_msg = DockAction::Goal();

  std::unordered_map<std::string, std::string> action_parameters_map;
  for (const auto & action_param : vda5050_action.action_parameters) {
    action_parameters_map[action_param.key] = action_param.value;
  }

  goal_msg.use_dock_id = false;
  double yaw;
  try {
    goal_msg.dock_type = action_parameters_map.at(kDockType);
    goal_msg.dock_pose = geometry_msgs::msg::PoseStamped();
    if (!isValidDockPose(action_parameters_map.at(kDockPose))) {
      throw std::invalid_argument("Failed to parse 'dock_pose'.");
    }
    std::vector<std::string> dock_pose_strings = split(action_parameters_map[kDockPose], ',');
    goal_msg.dock_pose.pose.position.x = std::stod(dock_pose_strings[0]);
    goal_msg.dock_pose.pose.position.y = std::stod(dock_pose_strings[1]);
    yaw = std::stod(dock_pose_strings[2]);
  } catch (std::invalid_argument & e) {
    RCLCPP_ERROR(client_node_->get_logger(), e.what());
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, e.what());
    return;
  } catch (std::out_of_range & e) {
    RCLCPP_ERROR(client_node_->get_logger(), e.what());
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, e.what());
    return;
  } catch (...) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Invalid dock_robot parameters");
    return;
  }
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, yaw);
  goal_msg.dock_pose.pose.orientation = tf2::toMsg(orientation);
  goal_msg.dock_pose.header.stamp = rclcpp::Clock().now();
  goal_msg.dock_pose.header.frame_id = "map";
  goal_msg.navigate_to_staging_pose = true;

  if (dock_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
    dock_client_->async_send_goal(goal_msg, send_goal_options);
  } else {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Failed to connect to dock server");
    // Disable switch
    if (use_switch_) {
      auto switch_request = std::make_shared<std_srvs::srv::SetBool::Request>();
      switch_request->data = false;
      switch_client_->async_send_request(switch_request);
    }

    // Disable docking server
    auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
    docking_lifecycle_manager_client_->async_send_request(server_request);
    return;
  }
}

void DockingHandler::ExecuteUndock(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  std::unordered_map<std::string, std::string> action_parameters_map;
  for (const auto & action_param : vda5050_action.action_parameters) {
    action_parameters_map[action_param.key] = action_param.value;
  }
  bool trigger_global_localization = true;
  if (action_parameters_map.find(kTriggerGlobalLocalization) != action_parameters_map.end()) {
    trigger_global_localization = stringToBool(
      action_parameters_map[kTriggerGlobalLocalization]);
  }

  // Enable docking server
  auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
  server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::RESUME;
  auto server_result = docking_lifecycle_manager_client_->async_send_request(server_request);
  if (executor_.spin_until_future_complete(server_result, std::chrono::seconds(kTimeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Failed to start docking server");
    return;
  }

  auto send_goal_options =
    rclcpp_action::Client<UndockAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, vda5050_action](
    const rclcpp_action::ClientGoalHandle<UndockAction>::SharedPtr &
    goal) {client_node_->ActionResponseCallback<UndockAction>(goal, vda5050_action);};
  send_goal_options.result_callback =
    [this, vda5050_action,
      trigger_global_localization](const GoalHandleUndockAction::WrappedResult & result) {
      // Disable docking server
      auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
      docking_lifecycle_manager_client_->async_send_request(server_request);
      // Parse error code
      std::string result_description = "";
      switch (result.result->error_code) {
        case UndockAction::Result::UNKNOWN:
          result_description = "Unknown";
          break;
        case UndockAction::Result::DOCK_NOT_VALID:
          result_description = "Dock not valid";
          break;
        case UndockAction::Result::FAILED_TO_CONTROL:
          result_description = "Failed to control";
          break;
        default:
          break;
      }
      client_node_->ActionResultCallback<GoalHandleUndockAction::WrappedResult>(vda5050_action,
          result, result.result->success, result_description);
      if (trigger_global_localization) {
        auto trigger_localization_request = std::make_shared<std_srvs::srv::Empty::Request>();
        localization_client_->async_send_request(trigger_localization_request);
      }
    };
  auto goal_msg = UndockAction::Goal();
  if (undock_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
    undock_client_->async_send_goal(goal_msg, send_goal_options);
  } else {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Failed to connect to undock server");
    // Disable docking server
    auto server_request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    server_request->command = nav2_msgs::srv::ManageLifecycleNodes_Request::PAUSE;
    docking_lifecycle_manager_client_->async_send_request(server_request);
    return;
  }
}

}
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::DockingHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
