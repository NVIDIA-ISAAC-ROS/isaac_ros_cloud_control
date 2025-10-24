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

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__SCENE_RECORDER_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__SCENE_RECORDER_HANDLER_HPP_

#include "vda5050_action_handler/vda5050_action_handler.hpp"
#include "isaac_ros_scene_recorder_interface/action/start_recording.hpp"
#include "isaac_ros_scene_recorder_interface/action/stop_recording.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace isaac_ros
{
namespace mission_client
{
class SceneRecorderHandler : public Vda5050ActionHandlerBase
{
public:
  using StartRecordingAction =
    isaac_ros_scene_recorder_interface::action::StartRecording;
  using GoalHandleStartRecordingAction = rclcpp_action::ClientGoalHandle<StartRecordingAction>;
  using StopRecordingAction =
    isaac_ros_scene_recorder_interface::action::StopRecording;
  using GoalHandleStopRecordingAction = rclcpp_action::ClientGoalHandle<StopRecordingAction>;

  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;

private:
  rclcpp_action::Client<StartRecordingAction>::SharedPtr start_recording_client_;
  rclcpp_action::Client<StopRecordingAction>::SharedPtr stop_recording_client_;

};

}  // namespace mission_client
}  // namespace isaac_ros
#endif  // VDA5050_ACTION_HANDLER_PLUGINS__SCENE_RECORDER_HANDLER_HPP_
