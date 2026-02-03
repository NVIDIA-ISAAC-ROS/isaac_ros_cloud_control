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
#include <ctime>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <sys/wait.h>
#include <string>
#include <vector>

#ifdef USE_LIBCURL
#include <curl/curl.h>
#endif

#include <yaml-cpp/yaml.h>

#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "vda5050_action_handler_plugins/map_handler.hpp"

namespace fs = std::filesystem;

namespace
{
// Execute a command without invoking a shell to avoid injection risks.
// Returns true if the command exits with status 0.
bool RunCommandNoShell(const std::vector<std::string> & args)
{
  if (args.empty()) {
    return false;
  }
  // Build argv array
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (const auto & s : args) {
    argv.push_back(const_cast<char *>(s.c_str()));
  }
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    // Child
    execvp(argv[0], argv.data());
    // If execvp returns, it's an error
    _exit(127);
  }
  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// Constants to avoid magic strings and numbers
constexpr char kActionDownloadMap[] = "downloadMap";
constexpr char kActionEnableMap[] = "enableMap";
constexpr char kParamMapId[] = "mapId";
constexpr char kParamMapVersion[] = "mapVersion";
constexpr char kParamMapDownloadLink[] = "mapDownloadLink";
constexpr char kDefaultMapId[] = "map";
constexpr char kDefaultVersionPrefix[] = "v";
constexpr int kLoadMapWaitSec = 10;
constexpr int kLoadMapCallTimeoutSec = 15;
constexpr int kAmclWaitSec = 2;
}  // namespace

namespace isaac_ros
{
namespace mission_client
{

void MapHandler::Initialize(
  isaac_ros::mission_client::Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  RCLCPP_INFO(rclcpp::get_logger("MapHandler"), "Initializing MapHandler");
  client_node_ = client_node;

  callback_group_ =
    client_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  map_storage_root_ = config["map_storage_root"] ?
    config["map_storage_root"].as<std::string>() : std::string("/tmp/vda5050_maps");
  map_server_load_service_ = config["map_server_load_service"] ?
    config["map_server_load_service"].as<std::string>() : std::string("/map_server/load_map");
  amcl_global_localization_service_ = config["amcl_global_localization_service"] ?
    config["amcl_global_localization_service"].as<std::string>() :
    std::string("/amcl/global_localization");
  amcl_reinitialize_global_localization_service_ =
    config["amcl_reinitialize_global_localization_service"] ?
    config["amcl_reinitialize_global_localization_service"].as<std::string>() :
    std::string("/reinitialize_global_localization");
  amcl_global_localization_retry_seconds_ = config["amcl_global_localization_retry_seconds"] ?
    config["amcl_global_localization_retry_seconds"].as<double>() : 30.0;

  // Log where maps will be stored and ensure the directory exists early
  try {
    const std::string abs_root = fs::absolute(map_storage_root_).string();
    RCLCPP_INFO(rclcpp::get_logger("MapHandler"), "Map storage root: %s", abs_root.c_str());
    fs::create_directories(map_storage_root_);
  } catch (...) {
  }

  load_map_client_ = client_node_->create_client<nav2_msgs::srv::LoadMap>(
    map_server_load_service_, rclcpp::ServicesQoS(), callback_group_);
  amcl_global_localization_client_ = client_node_->create_client<std_srvs::srv::Empty>(
    amcl_global_localization_service_, rclcpp::ServicesQoS(), callback_group_);
  amcl_reinitialize_global_localization_client_ = client_node_->create_client<std_srvs::srv::Empty>(
    amcl_reinitialize_global_localization_service_, rclcpp::ServicesQoS(), callback_group_);
}

void MapHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  if (vda5050_action.action_type == kActionDownloadMap) {
    ExecuteDownloadMap(vda5050_action);
  } else if (vda5050_action.action_type == kActionEnableMap) {
    ExecuteEnableMap(vda5050_action);
  } else {
    // Not handled: allow other handlers to process if configured, or mark failed
    RCLCPP_WARN(rclcpp::get_logger("MapHandler"), "Unhandled action type: %s",
      vda5050_action.action_type.c_str());
  }
}

bool MapHandler::DownloadUrlToFile(const std::string & url, const std::string & out_path)
{
#ifdef USE_LIBCURL
  CURL * curl = curl_easy_init();
  if (!curl) {return false;}
  FILE * fp = fopen(out_path.c_str(), "wb");
  if (!fp) {
    curl_easy_cleanup(curl);
    curl = nullptr;
    return false;
  }
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "MapHandler/1.0");
  CURLcode res = curl_easy_perform(curl);
  fclose(fp);
  fp = nullptr;
  curl_easy_cleanup(curl);
  curl = nullptr;
  return res == CURLE_OK;
#else
  // Fallback: invoke curl directly without a shell to avoid injection.
  // Use "--" to terminate curl options before URL.
  return RunCommandNoShell({"curl", "-sSfL", "--", url, "-o", out_path});
#endif
}

void MapHandler::MaybeExtractArchive(
  const std::string & archive_path,
  const std::string & target_dir)
{
  // Very lightweight: try unzip; if not zip, try tar. Ignore failures silently.
  if (!fs::exists(archive_path)) {return;}
  // unzip: unzip -oq <archive> -d <target_dir>
  bool ok = RunCommandNoShell({"unzip", "-oq", archive_path, "-d", target_dir});
  if (!ok) {
    // tar: tar -xf <archive> -C <target_dir>
    (void)RunCommandNoShell({"tar", "-xf", "--", archive_path, "-C", target_dir});
  }
}

void MapHandler::ExecuteDownloadMap(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(
    vda5050_action, vda5050_msgs::msg::ActionState::RUNNING, "Downloading map");

  std::string map_id;
  std::string map_version;
  std::string link;
  for (const auto & p : vda5050_action.action_parameters) {
    if (p.key == kParamMapId) {map_id = p.value;}
    if (p.key == kParamMapVersion) {map_version = p.value;}
    if (p.key == kParamMapDownloadLink) {link = p.value;}
  }
  if (map_id.empty()) {map_id = kDefaultMapId;}
  if (link.empty()) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Missing mapDownloadLink");
    return;
  }

  // Prepare target directory
  const auto ts = std::to_string(static_cast<long long>(std::time(nullptr)));
  const std::string version =
    map_version.empty() ? std::string(kDefaultVersionPrefix) : map_version;
  fs::create_directories(map_storage_root_);
  const std::string target_dir = (fs::path(map_storage_root_) /
    (map_id + "_" + version + "_" + ts)).string();
  fs::create_directories(target_dir);
  const std::string archive_path = (fs::path(target_dir) / "map_payload").string();

  if (!DownloadUrlToFile(link, archive_path)) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Download failed");
    return;
  }

  MaybeExtractArchive(archive_path, target_dir);

  client_node_->UpdateActionState(
    vda5050_action, vda5050_msgs::msg::ActionState::FINISHED,
        std::string("Map downloaded: ") + target_dir);
}

std::string MapHandler::FindLatestMapDir(
  const std::string & map_id, const std::string & map_version)
{
  if (!fs::exists(map_storage_root_)) {
    return std::string();
  }
  std::vector<std::string> candidates;
  const std::string prefix = map_version.empty() ? (map_id + "_") : (map_id + "_" + map_version);
  for (auto const & entry : fs::directory_iterator(map_storage_root_)) {
    try {
      if (entry.is_directory()) {
        const auto name = entry.path().filename().string();
        if (name.rfind(prefix, 0) == 0) {
          candidates.emplace_back(entry.path().string());
        }
      }
    } catch (...) {
    }
  }
  if (candidates.empty()) {
    return std::string();
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.back();
}

std::string MapHandler::FindMapYaml(const std::string & map_dir)
{
  if (map_dir.empty() || !fs::exists(map_dir)) {
    return std::string();
  }
  std::string map_yaml;
  for (auto const & entry : fs::recursive_directory_iterator(map_dir)) {
    try {
      if (entry.is_regular_file()) {
        const auto p = entry.path();
        const auto ext = p.extension().string();
        if (ext == ".yaml" || ext == ".yml") {
          if (p.filename() == "map.yaml" || p.filename() == "map.yml") {
            return p.string();
          }
          if (map_yaml.empty()) {
            map_yaml = p.string();
          }
        }
      }
    } catch (...) {
    }
  }
  return map_yaml;
}

void MapHandler::ExecuteEnableMap(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(
    vda5050_action, vda5050_msgs::msg::ActionState::RUNNING, "Enabling map");

  // Extract parameters
  std::string map_id;
  std::string map_version;
  for (const auto & p : vda5050_action.action_parameters) {
    if (p.key == kParamMapId) {map_id = p.value;}
    if (p.key == kParamMapVersion) {map_version = p.value;}
  }
  if (map_id.empty()) {map_id = kDefaultMapId;}

  const auto map_dir = FindLatestMapDir(map_id, map_version);
  if (map_dir.empty()) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "No downloaded map found");
    return;
  }
  auto yaml_path = FindMapYaml(map_dir);
  if (yaml_path.empty()) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "No map YAML found");
    return;
  }

  // Validate that the YAML references an existing image; otherwise, fail fast.
  try {
    YAML::Node doc = YAML::LoadFile(yaml_path);
    if (!doc || !doc["image"] || !doc["image"].IsScalar()) {
      client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Map YAML missing valid image");
      return;
    }
    const std::string image_rel = doc["image"].as<std::string>();
    const fs::path image_abs = fs::path(yaml_path).parent_path() / image_rel;
    if (!fs::exists(image_abs)) {
      client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Map image file not found");
      return;
    }
  } catch (...) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "Map YAML validation failed");
    return;
  }

  // Call Nav2 LoadMap
  auto load_req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  load_req->map_url = yaml_path;

  if (!load_map_client_->wait_for_service(std::chrono::seconds(kLoadMapWaitSec))) {
    client_node_->UpdateActionState(
      vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
      "map_server LoadMap service not available");
    return;
  }
  auto load_future = load_map_client_->async_send_request(load_req);
  // Use a local executor scoped to this call to avoid storing executors in the handler.
  {
    rclcpp::executors::SingleThreadedExecutor local_executor;
    local_executor.add_callback_group(callback_group_, client_node_->get_node_base_interface());
    if (local_executor.spin_until_future_complete(load_future,
          std::chrono::seconds(kLoadMapCallTimeoutSec)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      client_node_->UpdateActionState(
        vda5050_action, vda5050_msgs::msg::ActionState::FAILED, "LoadMap call timeout");
      return;
    }
  }

  // Trigger AMCL global localization (prefer reinitialize if available)
  try {
    if (amcl_reinitialize_global_localization_client_->wait_for_service(std::chrono::seconds(
          kAmclWaitSec)))
    {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      amcl_reinitialize_global_localization_client_->async_send_request(req);
    } else if (amcl_global_localization_client_->wait_for_service(std::chrono::seconds(
          kAmclWaitSec)))
    {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      amcl_global_localization_client_->async_send_request(req);
    }
  } catch (...) {
  }

  client_node_->UpdateActionState(
    vda5050_action, vda5050_msgs::msg::ActionState::FINISHED,
    std::string("Map enabled: ") + yaml_path);
}

}  // namespace mission_client
}  // namespace isaac_ros

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::MapHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
