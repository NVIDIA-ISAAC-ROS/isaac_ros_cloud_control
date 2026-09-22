// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <array>
#include <string>

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include "vda5050_action_handler/vda5050_action_handler.hpp"

namespace
{

using isaac_ros::mission_client::Vda5050ActionHandlerBase;

TEST(Vda5050ActionHandlerPlugins, DeclaresEveryPlugin)
{
  pluginlib::ClassLoader<Vda5050ActionHandlerBase> loader(
    "vda5050_action_handler", "isaac_ros::mission_client::Vda5050ActionHandlerBase");
  const std::array<std::string, 6> classes = {
    "isaac_ros::mission_client::DockingHandler",
    "isaac_ros::mission_client::PickAndPlaceHandler",
    "isaac_ros::mission_client::SceneRecorderHandler",
    "isaac_ros::mission_client::AprilTagHandler",
    "isaac_ros::mission_client::MapHandler",
    "isaac_ros::mission_client::Gr00tPolicyActionHandler",
  };

  for (const auto & class_name : classes) {
    SCOPED_TRACE(class_name);
    ASSERT_TRUE(loader.isClassAvailable(class_name));
  }
}

}  // namespace
