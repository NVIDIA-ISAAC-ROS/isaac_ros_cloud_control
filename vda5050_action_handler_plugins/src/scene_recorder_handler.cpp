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

#include <sstream>

#include "vda5050_action_handler_plugins/scene_recorder_handler.hpp"
#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace isaac_ros
{
namespace mission_client
{
constexpr int kTimeout = 10;

void SceneRecorderHandler::Initialize(
  Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  (void)config;
  client_node_ = client_node;
  start_recording_client_ = rclcpp_action::create_client<StartRecordingAction>(
    client_node_, "start_recording");
  stop_recording_client_ = rclcpp_action::create_client<StopRecordingAction>(
    client_node_, "stop_recording");
}

void SceneRecorderHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  std::unordered_map<std::string, std::string> action_parameters;
  for (const auto & param : vda5050_action.action_parameters) {
    action_parameters[param.key] = param.value;
  }
  std::string action_type = vda5050_action.action_type;
  if (action_type == "start_recording") {
    auto send_goal_options =
      rclcpp_action::Client<StartRecordingAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, vda5050_action](
      const rclcpp_action::ClientGoalHandle<StartRecordingAction>::SharedPtr &
      goal) {client_node_->ActionResponseCallback<StartRecordingAction>(goal, vda5050_action);};
    send_goal_options.result_callback =
      [this, vda5050_action](const GoalHandleStartRecordingAction::WrappedResult & result) {
        client_node_->ActionResultCallback<GoalHandleStartRecordingAction::WrappedResult>(
            vda5050_action,
            result, true, "");
      };
    auto goal_msg = StartRecordingAction::Goal();
    try {
      goal_msg.path = action_parameters["path"];
      std::vector<std::string> topics;
      std::string topic;
      std::istringstream topicStream(action_parameters["topics"]);
      while (std::getline(topicStream, topic, ',')) {
        topics.push_back(topic);
      }
      goal_msg.topics = topics;
      goal_msg.time = std::stoi(action_parameters["time"]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to parse action parameters: %s", e.what());
      client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
            "Failed to parse action parameters");
      return;
    }
    if (start_recording_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
      start_recording_client_->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to connect to start recording server");
      client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
            "Failed to connect to start recording server");
    }
  } else if (action_type == "stop_recording") {
    auto send_goal_options =
      rclcpp_action::Client<StopRecordingAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, vda5050_action](
      const rclcpp_action::ClientGoalHandle<StopRecordingAction>::SharedPtr &
      goal) {client_node_->ActionResponseCallback<StopRecordingAction>(goal, vda5050_action);};
    send_goal_options.result_callback =
      [this, vda5050_action](const GoalHandleStopRecordingAction::WrappedResult & result) {
        client_node_->ActionResultCallback<GoalHandleStopRecordingAction::WrappedResult>(
            vda5050_action,
            result, true, "");
      };
    auto goal_msg = StopRecordingAction::Goal();
    if (stop_recording_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
      stop_recording_client_->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to connect to stop recording server");
      client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
            "Failed to connect to stop recording server");
    }
  }
}

}  // namespace mission_client
}  // namespace isaac_ros


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::SceneRecorderHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
