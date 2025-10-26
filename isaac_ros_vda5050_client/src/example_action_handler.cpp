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

#include "isaac_ros_vda5050_client/example_action_handler.hpp"
#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace isaac_ros
{
namespace mission_client
{
void ExampleActionHandler::Initialize(
  Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  (void)config;
  client_node_ = client_node;
  RCLCPP_INFO(rclcpp::get_logger("ExampleActionHandler"), "Initializing ExampleActionHandler");
}

void ExampleActionHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(
    vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  std::thread([this, vda5050_action]() {
      std::unordered_map<std::string, std::string> action_parameters;
      for (const auto & param : vda5050_action.action_parameters) {
        action_parameters[param.key] = param.value;
      }
      int period = 1;
      if (action_parameters.find("period") != action_parameters.end()) {
        period = std::stoi(action_parameters["period"]);
      }
      std::this_thread::sleep_for(std::chrono::seconds(period));

      if (action_parameters["success"] == "true") {
        client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FINISHED);
        RCLCPP_INFO(rclcpp::get_logger("ExampleActionHandler"), "Example action %s finished",
            vda5050_action.action_id.c_str());
      } else {
        if (action_parameters["error_level"] == "FATAL") {
          auto error =
          client_node_->CreateError(
            isaac_ros::mission_client::Vda5050ClientNode::ErrorLevel::FATAL,
            "Example action failed",
            {{"action_id", vda5050_action.action_id}}
          );
          client_node_->AddError(error);
        }

        client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FAILED);
        RCLCPP_INFO(rclcpp::get_logger("ExampleActionHandler"), "Example action %s failed",
            vda5050_action.action_id.c_str());
      }
  }).detach();
}

void ExampleActionHandler::Cancel(const std::string & action_id)
{
  client_node_->UpdateActionStateById(
    action_id, vda5050_msgs::msg::ActionState::FAILED);
}

}  // namespace mission_client
}  // namespace isaac_ros


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::ExampleActionHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
