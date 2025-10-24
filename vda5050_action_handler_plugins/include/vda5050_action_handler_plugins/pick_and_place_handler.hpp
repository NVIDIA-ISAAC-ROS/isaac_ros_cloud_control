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

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__PICK_AND_PLACE_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__PICK_AND_PLACE_HANDLER_HPP_

#include "vda5050_action_handler/vda5050_action_handler.hpp"
#include "isaac_manipulator_interfaces/action/get_objects.hpp"
#include "isaac_manipulator_interfaces/action/pick_and_place.hpp"
#include "isaac_manipulator_interfaces/msg/object_info.hpp"
#include "isaac_manipulator_interfaces/srv/clear_objects.hpp"
#include "isaac_manipulator_interfaces/action/multi_object_pick_and_place.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace isaac_ros
{
namespace mission_client
{
class PickAndPlaceHandler : public Vda5050ActionHandlerBase
{
public:
  using GetObjectsAction =
    isaac_manipulator_interfaces::action::GetObjects;
  using GoalHandleGetObjectsAction = rclcpp_action::ClientGoalHandle<GetObjectsAction>;

  using PickPlaceAction =
    isaac_manipulator_interfaces::action::PickAndPlace;
  using GoalHandlePickPlaceAction = rclcpp_action::ClientGoalHandle<PickPlaceAction>;

  using MultiObjectPickAndPlaceAction =
    isaac_manipulator_interfaces::action::MultiObjectPickAndPlace;
  using GoalHandleMultiObjectPickAndPlaceAction =
    rclcpp_action::ClientGoalHandle<MultiObjectPickAndPlaceAction>;

  using ClearObjectsService = isaac_manipulator_interfaces::srv::ClearObjects;

  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;

private:
  void ExecuteGetObjects(const vda5050_msgs::msg::Action & vda5050_action);
  void ExecutePickPlace(const vda5050_msgs::msg::Action & vda5050_action);
  void ExecuteClearObjects(const vda5050_msgs::msg::Action & vda5050_action);
  void ExecuteMultiObjectPickAndPlace(const vda5050_msgs::msg::Action & vda5050_action);
  rclcpp_action::Client<GetObjectsAction>::SharedPtr get_objects_client_;
  rclcpp_action::Client<PickPlaceAction>::SharedPtr pick_and_place_client_;
  rclcpp_action::Client<MultiObjectPickAndPlaceAction>::SharedPtr
    multi_object_pick_and_place_client_;
  rclcpp::Client<ClearObjectsService>::SharedPtr clear_objects_client_;

  // Executor for service reqeust callbacks
  rclcpp::executors::SingleThreadedExecutor executor_;
  // Callback group for service reqeust callbacks
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
}  // namespace mission_client
}  // namespace isaac_ros

#endif  // VDA5050_ACTION_HANDLER_PLUGINS__PICK_AND_PLACE_HANDLER_HPP_
