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

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__MAP_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__MAP_HANDLER_HPP_

#include <string>

#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "vda5050_action_handler/vda5050_action_handler.hpp"

namespace isaac_ros
{
namespace mission_client
{
class MapHandler : public Vda5050ActionHandlerBase
{
public:
  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;

private:
  void ExecuteDownloadMap(const vda5050_msgs::msg::Action & vda5050_action);
  void ExecuteEnableMap(const vda5050_msgs::msg::Action & vda5050_action);
  std::string FindMapYaml(const std::string & map_dir);
  std::string FindLatestMapDir(const std::string & map_id, const std::string & map_version);
  bool DownloadUrlToFile(const std::string & url, const std::string & out_path);
  void MaybeExtractArchive(const std::string & archive_path, const std::string & target_dir);

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr amcl_global_localization_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr amcl_reinitialize_global_localization_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string map_storage_root_;
  std::string map_server_load_service_;
  std::string amcl_global_localization_service_;
  std::string amcl_reinitialize_global_localization_service_;
  double amcl_global_localization_retry_seconds_;
};
}  // namespace mission_client
}  // namespace isaac_ros

#endif  // VDA5050_ACTION_HANDLER_PLUGINS__MAP_HANDLER_HPP_
