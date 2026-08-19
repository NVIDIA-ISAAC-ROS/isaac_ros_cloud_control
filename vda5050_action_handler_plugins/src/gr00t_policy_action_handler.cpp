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

#include "vda5050_action_handler_plugins/gr00t_policy_action_handler.hpp"

#include <chrono>
#include <functional>
#include <regex>
#include <sstream>

#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vda5050_msgs/msg/action_state.hpp"

namespace isaac_ros
{
namespace mission_client
{

void Gr00tPolicyActionHandler::Initialize(
  Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  client_node_ = client_node;
  RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
        "Initializing Gr00tPolicyActionHandler");

  // Create callback group for action client
  callback_group_ =
    client_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Read configuration parameters
  action_server_name_ = config["action_server_name"] ?
    config["action_server_name"].as<std::string>() : "humanoid_task";
  default_timeout_ = config["default_timeout"] ?
    config["default_timeout"].as<double>() : 10.0;

  // Create action client
  action_client_ = rclcpp_action::create_client<HumanoidTask>(
    client_node_, action_server_name_, callback_group_);

  // Wait for action server to be available
  std::thread([this]() {
      if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Action server %s not available after waiting", action_server_name_.c_str());
      } else {
        RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Connected to action server %s", action_server_name_.c_str());
      }
  }).detach();
}

void Gr00tPolicyActionHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
    "Executing GR00T policy action: %s", vda5050_action.action_id.c_str());

  // Check if another action is already running
  if (current_goal_handle_ != nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
      "Rejecting action %s - another action %s is already running",
      vda5050_action.action_id.c_str(), current_action_id_.c_str());
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
      "Another action is already running");
    return;
  }

  // Update VDA5050 action state to RUNNING
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);

  // Build HumanoidTask goal from VDA5050 action
  auto goal_msg = buildHumanoidTaskGoal(vda5050_action);

  // Prepare send goal options using client node's template callbacks
  auto send_goal_options = rclcpp_action::Client<HumanoidTask>::SendGoalOptions();

  // Goal response callback using client node's template method
  send_goal_options.goal_response_callback =
    [this, vda5050_action](const HumanoidTaskGoalHandle::SharedPtr & goal_handle) {
      client_node_->ActionResponseCallback<HumanoidTask>(goal_handle, vda5050_action);

      // Store the goal handle for potential cancellation
      if (goal_handle) {
        current_goal_handle_ = goal_handle;
        current_action_id_ = vda5050_action.action_id;
        RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Action %s accepted by policy server", vda5050_action.action_id.c_str());
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Action %s rejected by policy server", vda5050_action.action_id.c_str());
        client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Action rejected by policy server");
      }
    };

  // Feedback callback (optional - can keep simple logging)
  send_goal_options.feedback_callback =
    [this, vda5050_action](const HumanoidTaskGoalHandle::SharedPtr,
    const std::shared_ptr<const HumanoidTask::Feedback> feedback) {
      RCLCPP_DEBUG(rclcpp::get_logger("Gr00tPolicyActionHandler"),
        "Received feedback for action %s: status=%d, time=%.1f",
        vda5050_action.action_id.c_str(), feedback->status, feedback->current_execution_time);
    };

  // Result callback using client node's template method
  send_goal_options.result_callback =
    [this,
      vda5050_action](const rclcpp_action::ClientGoalHandle<HumanoidTask>::WrappedResult & result) {
      // Clear current action tracking
      current_goal_handle_ = nullptr;
      current_action_id_.clear();

      RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
        "Action %s completed with result: %s",
        vda5050_action.action_id.c_str(),
        result.result->did_succeed ? "SUCCESS" : "FAILED");

      // Use client node's template callback for consistent result handling
      client_node_->ActionResultCallback(vda5050_action, result, result.result->did_succeed,
          result.result->message);
    };

  // Send goal
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void Gr00tPolicyActionHandler::Cancel(const std::string & action_id)
{
  RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
    "Cancelling action: %s", action_id.c_str());

  // Check if the requested action is the currently running one
  if (current_action_id_ == action_id && current_goal_handle_ != nullptr) {
    RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
      "Sending cancel request to policy server for action: %s", action_id.c_str());

    // Send cancel request
    auto cancel_future = action_client_->async_cancel_goal(current_goal_handle_);

    RCLCPP_INFO(rclcpp::get_logger("Gr00tPolicyActionHandler"),
      "Cancel request sent for action: %s", action_id.c_str());

    // Clear current action tracking
    current_goal_handle_ = nullptr;
    current_action_id_.clear();

    client_node_->UpdateActionStateById(action_id, vda5050_msgs::msg::ActionState::FAILED);
  } else if (current_action_id_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
      "No active action to cancel for action_id: %s", action_id.c_str());
    client_node_->UpdateActionStateById(action_id, vda5050_msgs::msg::ActionState::FAILED);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
      "Cannot cancel action %s - different action %s is currently running",
      action_id.c_str(), current_action_id_.c_str());
    client_node_->UpdateActionStateById(action_id, vda5050_msgs::msg::ActionState::FAILED);
  }
}

HumanoidTask::Goal Gr00tPolicyActionHandler::buildHumanoidTaskGoal(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  HumanoidTask::Goal goal;

  // Extract task information from VDA5050 action parameters
  goal.task_category = extractTaskCategory(vda5050_action);
  goal.task_id = extractTaskId(vda5050_action);
  goal.language_instruction = extractLanguageInstruction(vda5050_action);
  goal.timeout = extractTimeout(vda5050_action);
  goal.parameters = extractParameters(vda5050_action);

  // Extract locomotion target if provided
  goal.locomotion_pose = extractLocomotionTarget(vda5050_action);

  return goal;
}

std::string Gr00tPolicyActionHandler::extractTaskCategory(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key == "task_category") {
      return param.value;
    }
  }
  // Default to manipulation if not specified
  return "manipulation";
}

std::string Gr00tPolicyActionHandler::extractTaskId(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key == "task_id") {
      return param.value;
    }
  }
  // Use action type as task ID if not specified
  return vda5050_action.action_type;
}

std::string Gr00tPolicyActionHandler::extractLanguageInstruction(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key == "language_instruction" || param.key == "instruction") {
      return param.value;
    }
  }
  // Default instruction
  return "Perform the requested task";
}

double Gr00tPolicyActionHandler::extractTimeout(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key == "timeout") {
      try {
        return std::stod(param.value);
      } catch (const std::exception &) {
        RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Invalid timeout value: %s", param.value.c_str());
      }
    }
  }
  return default_timeout_;
}

geometry_msgs::msg::PoseStamped Gr00tPolicyActionHandler::extractLocomotionTarget(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  geometry_msgs::msg::PoseStamped locomotion_pose;

  // Set default empty locomotion target
  locomotion_pose.header.frame_id = "";
  locomotion_pose.header.stamp = client_node_->get_clock()->now();
  locomotion_pose.pose.position.x = 0.0;
  locomotion_pose.pose.position.y = 0.0;
  locomotion_pose.pose.position.z = 0.0;
  locomotion_pose.pose.orientation.x = 0.0;
  locomotion_pose.pose.orientation.y = 0.0;
  locomotion_pose.pose.orientation.z = 0.0;
  locomotion_pose.pose.orientation.w = 1.0;

  // Look for locomotion target parameters
  std::string locomotion_pose_str;
  std::string frame_id = "map";  // Default frame

  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key == "locomotion_pose") {
      locomotion_pose_str = param.value;
    } else if (param.key == "locomotion_frame_id") {
      frame_id = param.value;
    }
  }

  // If locomotion pose is provided, parse it
  if (!locomotion_pose_str.empty()) {
    try {
      // Support two formats:
      // 1. "x,y,yaw" format (like docking handler)
      // 2. "x,y,z,qx,qy,qz,qw" format (full pose)

      std::regex comma_pattern("\\s*,\\s*");
      std::vector<std::string> tokens;
      std::sregex_token_iterator iter(locomotion_pose_str.begin(), locomotion_pose_str.end(),
        comma_pattern, -1);
      std::sregex_token_iterator end;

      for (; iter != end; ++iter) {
        std::string token = *iter;
        if (!token.empty()) {
          tokens.push_back(token);
        }
      }

      if (tokens.size() == 3) {
        // Format: "x,y,yaw" - convert yaw to quaternion
        locomotion_pose.pose.position.x = std::stod(tokens[0]);
        locomotion_pose.pose.position.y = std::stod(tokens[1]);
        locomotion_pose.pose.position.z = 0.0;

        double yaw = std::stod(tokens[2]);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        locomotion_pose.pose.orientation = tf2::toMsg(q);

      } else if (tokens.size() == 7) {
        // Format: "x,y,z,qx,qy,qz,qw" - full pose
        locomotion_pose.pose.position.x = std::stod(tokens[0]);
        locomotion_pose.pose.position.y = std::stod(tokens[1]);
        locomotion_pose.pose.position.z = std::stod(tokens[2]);
        locomotion_pose.pose.orientation.x = std::stod(tokens[3]);
        locomotion_pose.pose.orientation.y = std::stod(tokens[4]);
        locomotion_pose.pose.orientation.z = std::stod(tokens[5]);
        locomotion_pose.pose.orientation.w = std::stod(tokens[6]);

      } else {
        RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
          "Invalid locomotion_pose format: %s. Expected 'x,y,yaw' or 'x,y,z,qx,qy,qz,qw'",
          locomotion_pose_str.c_str());
        return locomotion_pose;  // Return default empty target
      }

      // Set frame_id and timestamp for valid pose
      locomotion_pose.header.frame_id = frame_id;
      locomotion_pose.header.stamp = client_node_->get_clock()->now();
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("Gr00tPolicyActionHandler"),
        "Failed to parse locomotion_pose '%s': %s", locomotion_pose_str.c_str(), e.what());
    }
  }

  return locomotion_pose;
}

std::string Gr00tPolicyActionHandler::extractParameters(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  std::string json_params = "{";
  bool first = true;

  for (const auto & param : vda5050_action.action_parameters) {
    if (param.key != "task_category" && param.key != "task_id" &&
      param.key != "language_instruction" && param.key != "instruction" &&
      param.key != "timeout" &&
      param.key != "locomotion_pose" &&
      param.key != "locomotion_frame_id")
    {
      if (!first) {
        json_params += ",";
      }
      json_params += "\"" + param.key + "\":\"" + param.value + "\"";
      first = false;
    }
  }
  json_params += "}";

  return json_params;
}

}  // namespace mission_client
}  // namespace isaac_ros

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::Gr00tPolicyActionHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
