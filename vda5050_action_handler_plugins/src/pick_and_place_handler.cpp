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
#include <vector>
#include <unordered_map>
#include <iomanip>
#include <string>

#include "vda5050_action_handler_plugins/pick_and_place_handler.hpp"
#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "nlohmann/json.hpp"

namespace isaac_ros
{
namespace mission_client
{
namespace
{
constexpr char kGetObjectsAction[] = "get_objects";
constexpr char kPickPlaceAction[] = "pick_place";
constexpr char kClearObjects[] = "clear_objects";
constexpr char kObjectId[] = "object_id";
constexpr char kClassId[] = "class_id";
constexpr char kPlacePose[] = "place_pose";
constexpr int kTimeout = 5;
constexpr char kMultiObjectPickAndPlaceAction[] = "multi_object_pick_and_place";
constexpr char kMode[] = "mode";
constexpr char kClassIds[] = "class_ids";
constexpr char kTargetPoses[] = "target_poses";
constexpr char kMultiBin[] = "MULTI_BIN";
constexpr char kSingleBin[] = "SINGLE_BIN";
}

void PickAndPlaceHandler::Initialize(
  Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  (void)config;
  client_node_ = client_node;

  callback_group_ =
    client_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(callback_group_, client_node_->get_node_base_interface());

  get_objects_client_ = rclcpp_action::create_client<GetObjectsAction>(
    client_node_,
    kGetObjectsAction);
  pick_and_place_client_ = rclcpp_action::create_client<PickPlaceAction>(
    client_node_,
    kPickPlaceAction);
  multi_object_pick_and_place_client_ = rclcpp_action::create_client<MultiObjectPickAndPlaceAction>(
    client_node_,
    kMultiObjectPickAndPlaceAction);
  clear_objects_client_ = client_node_->create_client<ClearObjectsService>(
    kClearObjects, rclcpp::ServicesQoS(), callback_group_);
}

void PickAndPlaceHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  if (vda5050_action.action_type == kGetObjectsAction) {
    ExecuteGetObjects(vda5050_action);
  } else if (vda5050_action.action_type == kPickPlaceAction) {
    ExecutePickPlace(vda5050_action);
  } else if (vda5050_action.action_type == kClearObjects) {
    ExecuteClearObjects(vda5050_action);
  } else if (vda5050_action.action_type == kMultiObjectPickAndPlaceAction) {
    ExecuteMultiObjectPickAndPlace(vda5050_action);
  } else {
    RCLCPP_ERROR(client_node_->get_logger(), "Action type not supported: %s",
      vda5050_action.action_type.c_str());
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
      "Action type not supported");
  }
}

std::vector<std::string> split(const std::string & s, char delimiter, bool trim_whitespace = true)
{
  RCLCPP_DEBUG(rclcpp::get_logger("split"), "Splitting string: %s", s.c_str());
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    if (trim_whitespace) {
      token.erase(0, token.find_first_not_of(" \t\n\r\f\v"));
      token.erase(token.find_last_not_of(" \t\n\r\f\v") + 1);
    }
    tokens.push_back(token);
  }
  return tokens;
}

std::string jsonFromDetection2D(const vision_msgs::msg::Detection2D & detection)
{
  std::ostringstream json_result;
  if (detection.results.size() == 0) {
    return "";
  }
  json_result << std::fixed << std::setprecision(3);  // Setting precision for double values
  json_result << "  \"bbox2d\": {\n"
              << "    \"center\": {\n"
              << "      \"x\": " << detection.bbox.center.position.x << ",\n"
              << "      \"y\": " << detection.bbox.center.position.y << ",\n"
              << "      \"theta\": " << detection.bbox.center.theta << "\n"
              << "    },\n"
              << "    \"size_x\": " << detection.bbox.size_x << ",\n"
              << "    \"size_y\": " << detection.bbox.size_y << "\n"
              << "  }";
  return json_result.str();
}

std::string jsonFromDetection3D(const vision_msgs::msg::Detection3D & detection)
{
  std::ostringstream json_result;
  if (detection.results.size() == 0) {
    return "";
  }
  json_result << std::fixed << std::setprecision(3);  // Setting precision for double values
  json_result << "  \"bbox3d\": {\n"
              << "    \"center\": {\n"
              << "      \"position\": {\n"
              << "        \"x\": " << detection.bbox.center.position.x << ",\n"
              << "        \"y\": " << detection.bbox.center.position.y << ",\n"
              << "        \"z\": " << detection.bbox.center.position.z << "\n"
              << "      },\n"
              << "      \"orientation\": {\n"
              << "        \"x\": " << detection.bbox.center.orientation.x << ",\n"
              << "        \"y\": " << detection.bbox.center.orientation.y << ",\n"
              << "        \"z\": " << detection.bbox.center.orientation.z << ",\n"
              << "        \"w\": " << detection.bbox.center.orientation.w << "\n"
              << "      }\n"
              << "    },\n"
              << "    \"size_x\": " << detection.bbox.size.x << ",\n"
              << "    \"size_y\": " << detection.bbox.size.y << ",\n"
              << "    \"size_z\": " << detection.bbox.size.z << "\n"
              << "  }";
  return json_result.str();
}

std::string getClassId(const std::vector<vision_msgs::msg::ObjectHypothesisWithPose> & objects)
{
  std::string class_id = "";
  double score = -1;
  for (size_t i = 0; i < objects.size(); i++) {
    if (objects[i].hypothesis.score > score) {
      score = objects[i].hypothesis.score;
      class_id = objects[i].hypothesis.class_id;
    }
  }
  return class_id;
}

std::string jsonFromObjectInfo(const isaac_manipulator_interfaces::msg::ObjectInfo & object_info)
{
  std::ostringstream json_result;
  std::string detection_2d = jsonFromDetection2D(object_info.detection_2d);
  std::string detection_3d = jsonFromDetection3D(object_info.detection_3d);
  std::string class_id = "";
  if (object_info.detection_2d.results.size() > 0) {
    class_id = getClassId(object_info.detection_2d.results);
  } else {
    class_id = getClassId(object_info.detection_3d.results);
  }
  json_result << "{\n"
              << "  \"object_id\": \"" << std::to_string(object_info.object_id) << "\",\n"
              << "  \"class_id\": \"" << class_id << "\",\n";
  if (detection_2d.length() > 0) {
    json_result << detection_2d << ((detection_3d.length() > 0) ? ",\n" : "");
  }
  if (detection_3d.length() > 0) {
    json_result << detection_3d;
  }
  json_result << "\n}";
  return json_result.str();
}

std::string jsonFromGetObjectsResult(
  const isaac_ros::mission_client::PickAndPlaceHandler::GoalHandleGetObjectsAction::
  WrappedResult & result)
{
  std::string json_result = "[\n";
  for (size_t i = 0; i < result.result->objects.size(); i++) {
    if (i != 0) {
      json_result += ", ";
    }
    json_result += jsonFromObjectInfo(result.result->objects[i]);
  }
  json_result += "\n]";
  return json_result;
}

geometry_msgs::msg::PoseArray PoseArrayFromString(const std::string & pose_str)
{
  geometry_msgs::msg::PoseArray pose_array;
  try {
    auto json_obj = nlohmann::json::parse(pose_str);
    // Set frame_id if present
    pose_array.header.frame_id = json_obj["frame_id"].get<std::string>();
    for (const auto & pose_json : json_obj["poses"]) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = pose_json["position"]["x"].get<double>();
      pose.position.y = pose_json["position"]["y"].get<double>();
      pose.position.z = pose_json["position"]["z"].get<double>();
      pose.orientation.x = pose_json["orientation"]["x"].get<double>();
      pose.orientation.y = pose_json["orientation"]["y"].get<double>();
      pose.orientation.z = pose_json["orientation"]["z"].get<double>();
      pose.orientation.w = pose_json["orientation"]["w"].get<double>();
      pose_array.poses.push_back(pose);
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PoseArrayFromString"), "Invalid pose string: %s", e.what());
    throw std::invalid_argument("Invalid pose string: " + std::string(e.what()));
  }
  return pose_array;
}

void PickAndPlaceHandler::ExecuteGetObjects(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  auto send_goal_options =
    rclcpp_action::Client<GetObjectsAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, vda5050_action](
    const rclcpp_action::ClientGoalHandle<GetObjectsAction>::SharedPtr &
    goal) {client_node_->ActionResponseCallback<GetObjectsAction>(goal, vda5050_action);};
  send_goal_options.result_callback =
    [this, vda5050_action](const GoalHandleGetObjectsAction::WrappedResult & result) {
      std::string json_result;
      json_result = jsonFromGetObjectsResult(result);
      client_node_->ActionResultCallback<GoalHandleGetObjectsAction::WrappedResult>(vda5050_action,
          result, true, json_result);
    };
  auto goal_msg = GetObjectsAction::Goal();
  if (get_objects_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
    get_objects_client_->async_send_goal(goal_msg, send_goal_options);
  } else {
    RCLCPP_ERROR(client_node_->get_logger(), "Failed to connect to get objects server");
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Failed to connect to get objects server");
    return;
  }
}

void PickAndPlaceHandler::ExecutePickPlace(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  auto send_goal_options =
    rclcpp_action::Client<PickPlaceAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, vda5050_action](
    const rclcpp_action::ClientGoalHandle<PickPlaceAction>::SharedPtr &
    goal) {client_node_->ActionResponseCallback<PickPlaceAction>(goal, vda5050_action);};
  send_goal_options.result_callback =
    [this, vda5050_action](const GoalHandlePickPlaceAction::WrappedResult & result) {
      client_node_->ActionResultCallback<GoalHandlePickPlaceAction::WrappedResult>(vda5050_action,
          result, true, "");
    };
  auto goal_msg = PickPlaceAction::Goal();
  std::unordered_map<std::string, std::string> action_parameters_map;
  for (const auto & action_param : vda5050_action.action_parameters) {
    action_parameters_map[action_param.key] = action_param.value;
  }
  try {
    goal_msg.object_id = std::stoi(action_parameters_map[kObjectId]);
    goal_msg.class_id = action_parameters_map[kClassId];

    std::vector<std::string> place_pose = split(action_parameters_map[kPlacePose], ',');

    goal_msg.place_pose.position.x = std::stod(place_pose[0]);
    goal_msg.place_pose.position.y = std::stod(place_pose[1]);
    goal_msg.place_pose.position.z = std::stod(place_pose[2]);

    goal_msg.place_pose.orientation.x = std::stod(place_pose[3]);
    goal_msg.place_pose.orientation.y = std::stod(place_pose[4]);
    goal_msg.place_pose.orientation.z = std::stod(place_pose[5]);
    goal_msg.place_pose.orientation.w = std::stod(place_pose[6]);
  } catch (std::out_of_range & e) {
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Invalid action parameters");
    return;
  } catch (std::invalid_argument & e) {
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Invalid action parameters");
    return;
  }
  if (pick_and_place_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
    pick_and_place_client_->async_send_goal(goal_msg, send_goal_options);
  } else {
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Failed to connect to pick and place server");
    return;
  }
}

void PickAndPlaceHandler::ExecuteClearObjects(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  auto request = std::make_shared<ClearObjectsService::Request>();
  auto result = clear_objects_client_->async_send_request(request);
  if (executor_.spin_until_future_complete(result, std::chrono::seconds(kTimeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Failed to clear objects");
    RCLCPP_ERROR(client_node_->get_logger(), "Failed to clear objects");
    return;
  }
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FINISHED);
}

void PickAndPlaceHandler::ExecuteMultiObjectPickAndPlace(
  const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
  std::unordered_map<std::string, std::string> action_parameters_map;
  for (const auto & action_param : vda5050_action.action_parameters) {
    action_parameters_map[action_param.key] = action_param.value;
  }
  auto goal_msg = MultiObjectPickAndPlaceAction::Goal();
  try {
    std::string mode = action_parameters_map[kMode];
    RCLCPP_DEBUG(client_node_->get_logger(), "Mode: %s", mode.c_str());
    if (mode == kMultiBin) {
      goal_msg.mode = MultiObjectPickAndPlaceAction::Goal::MULTI_BIN;
    } else if (mode == kSingleBin) {
      goal_msg.mode = MultiObjectPickAndPlaceAction::Goal::SINGLE_BIN;
    } else {
      throw std::invalid_argument("Invalid mode");
    }
    goal_msg.class_ids = split(action_parameters_map[kClassIds], ',');
    goal_msg.target_poses = PoseArrayFromString(action_parameters_map[kTargetPoses]);
  } catch (std::exception & e) {
    RCLCPP_ERROR(client_node_->get_logger(), "Invalid action parameters: %s", e.what());
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
      "Invalid action parameters: " + std::string(e.what()));
    return;
  }
  auto send_goal_options =
    rclcpp_action::Client<MultiObjectPickAndPlaceAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this, vda5050_action](
    const rclcpp_action::ClientGoalHandle<MultiObjectPickAndPlaceAction>::SharedPtr &
    goal) {
      client_node_->ActionResponseCallback<MultiObjectPickAndPlaceAction>(goal, vda5050_action);
    };
  send_goal_options.result_callback =
    [this, vda5050_action](const GoalHandleMultiObjectPickAndPlaceAction::WrappedResult & result) {
      bool success = false;
      std::string message = "";
      switch (result.result->workflow_status) {
        case MultiObjectPickAndPlaceAction::Result::SUCCESS:
          success = true;
          break;
        case MultiObjectPickAndPlaceAction::Result::FAILED:
          success = false;
          break;
        case MultiObjectPickAndPlaceAction::Result::PARTIAL_SUCCESS:
          message = result.result->workflow_summary;
          success = false;
          break;
        case MultiObjectPickAndPlaceAction::Result::INCOMPLETE:
          message = result.result->workflow_summary;
          success = false;
          break;
        default:
          success = false;
          message = result.result->workflow_summary;
          break;
      }
      client_node_->ActionResultCallback<GoalHandleMultiObjectPickAndPlaceAction::WrappedResult>(
          vda5050_action,
          result, success, message);
    };
  RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for action server");
  if (multi_object_pick_and_place_client_->wait_for_action_server(std::chrono::seconds(kTimeout))) {
    multi_object_pick_and_place_client_->async_send_goal(goal_msg, send_goal_options);
  } else {
    RCLCPP_ERROR(client_node_->get_logger(),
          "Failed to connect to multi object pick and place server");
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FAILED,
          "Failed to connect to multi object pick and place server");
    return;
  }
}
}  // namespace mission_client
}  // namespace isaac_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::PickAndPlaceHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
