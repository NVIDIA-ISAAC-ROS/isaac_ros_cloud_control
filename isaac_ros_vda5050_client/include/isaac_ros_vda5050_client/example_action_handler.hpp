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

#ifndef ISAAC_ROS_VDA5050_CLIENT__EXAMPLE_ACTION_HANDLER_HPP_
#define ISAAC_ROS_VDA5050_CLIENT__EXAMPLE_ACTION_HANDLER_HPP_

#include <string>
#include "vda5050_action_handler/vda5050_action_handler.hpp"

namespace isaac_ros
{
namespace mission_client
{
class ExampleActionHandler : public Vda5050ActionHandlerBase
{
public:
  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;
  void Cancel(const std::string & action_id) override;
};

}  // namespace mission_client
}  // namespace isaac_ros
#endif  // ISAAC_ROS_VDA5050_CLIENT__EXAMPLE_ACTION_HANDLER_HPP_
