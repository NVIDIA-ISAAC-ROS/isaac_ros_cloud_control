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

#ifndef VDA5050_ACTION_HANDLER_HPP
#define VDA5050_ACTION_HANDLER_HPP

#include "vda5050_msgs/msg/action.hpp"
#include "yaml-cpp/yaml.h"

namespace isaac_ros
{
namespace mission_client
{
class Vda5050ClientNode;
class Vda5050ActionHandlerBase
{

public:
  virtual void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) = 0;
  // Execute the action. It should update the action state to RUNNING when the action is executed
  // and set the action state to FINISHED/FAILED when the action is done.
  virtual void Execute(const vda5050_msgs::msg::Action & vda5050_action) = 0;
  // Cancel the action. Default implementation is empty, mission client will wait for the
  // action to finish when canceling the order. If the action can be interrupted, the action
  // handler should implement this function to interrupt the action and set the action state to
  // FAILED
  virtual void Cancel(const std::string & /*action_id*/) {}
  // Pause the action. It should be implemented if the action can be paused(can_be_paused is true)
  // If the action is paused, the action handler should set the action state to PAUSED
  virtual void Pause(const std::string & /*action_id*/) {}
  // Resume the action. It should be implemented if the action can be paused.
  // If the action is resumed, the action handler should set the action state to RUNNING
  virtual void Resume(const std::string & /*action_id*/) {}
  virtual ~Vda5050ActionHandlerBase() {}

  bool can_be_paused{false};

protected:
  Vda5050ActionHandlerBase() {}
  Vda5050ClientNode * client_node_;

};
}  // namespace mission_client
}  // namespace isaac_ros

#endif  // VDA5050_ACTION_HANDLER_HPP
