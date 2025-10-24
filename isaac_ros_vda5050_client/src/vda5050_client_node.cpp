// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include <pluginlib/class_loader.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vda5050_msgs/msg/error_reference.hpp"
#include "vda5050_msgs/msg/info.hpp"

namespace isaac_ros
{
namespace mission_client
{
namespace
{
constexpr char kValidationError[] = "validationError";
constexpr char kOrderUpdateError[] = "orderUpdateError";
constexpr double kExecuteOrderPeriod = 0.2;

const rclcpp::QoS kDefaultQoS =
  rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
}  // namespace

Vda5050ClientNode::Vda5050ClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vda5050_client", options),
  update_feedback_period_(
    declare_parameter<double>("update_feedback_period", 1.0)),
  odom_topic_(declare_parameter<std::string>("odom_topic", "odom")),
  battery_state_topic_(declare_parameter<std::string>("battery_state_topic", "battery_state")),
  robot_type_(declare_parameter<std::string>("robot_type", "CARRIER")),
  status_check_service_(
    declare_parameter<std::string>("status_check_service", "")),
  config_file_(declare_parameter<std::string>(
    "config_file", "config/client_config.yaml")),
  order_sub_(create_subscription<vda5050_msgs::msg::Order>(
      "client_commands", kDefaultQoS,
      std::bind(&Vda5050ClientNode::OrderCallback, this,
      std::placeholders::_1))),
  instant_actions_sub_(
    create_subscription<vda5050_msgs::msg::InstantActions>(
      "instant_actions_commands", kDefaultQoS,
      std::bind(&Vda5050ClientNode::InstantActionsCallback, this,
      std::placeholders::_1))),
  battery_state_sub_(create_subscription<sensor_msgs::msg::BatteryState>(
      battery_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Vda5050ClientNode::BatteryStateCallback, this,
      std::placeholders::_1))),
  odometry_sub_(create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Vda5050ClientNode::OdometryCallback, this,
      std::placeholders::_1))),
  order_valid_error_sub_(create_subscription<std_msgs::msg::String>(
      "order_valid_error", kDefaultQoS,
      std::bind(&Vda5050ClientNode::OrderValidErrorCallback, this,
      std::placeholders::_1))),
  info_sub_(create_subscription<std_msgs::msg::String>(
      "info", kDefaultQoS,
      std::bind(&Vda5050ClientNode::InfoCallback, this,
      std::placeholders::_1))),
  order_info_pub_(
    create_publisher<vda5050_msgs::msg::AGVState>("agv_state", 1)),
  factsheet_info_pub_(
    create_publisher<vda5050_msgs::msg::Factsheet>("factsheet", 1)),
  order_id_pub_(
    create_publisher<std_msgs::msg::String>("order_id", 1)),
  client_ptr_(
    rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses")),
  nav_goal_handle_(nullptr),
  state_callback_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  executor_(),
  robot_state_timer_(create_wall_timer(
      std::chrono::duration<double>(update_feedback_period_),
      std::bind(&Vda5050ClientNode::StateTimerCallback, this),
      state_callback_group_)),
  execute_order_timer_(create_wall_timer(
      std::chrono::duration<double>(kExecuteOrderPeriod),
      std::bind(&Vda5050ClientNode::ExecuteOrderCallback, this))),
  order_id_timer_(create_wall_timer(
      std::chrono::duration<double>(1.0),
      std::bind(&Vda5050ClientNode::OrderIdCallback, this))),
  current_order_(nullptr),
  agv_state_(std::make_shared<vda5050_msgs::msg::AGVState>()),
  factsheet_(std::make_shared<vda5050_msgs::msg::Factsheet>()),
  num_actions_(0),
  cancel_action_(nullptr),
  reached_waypoint_(false),
  pause_order_(false),
  pause_order_action_id_(""),
  current_node_(0),
  next_stop_(0),
  current_node_action_(0),
  current_action_state_(0),
  action_handler_loader_("vda5050_action_handler",
    "isaac_ros::mission_client::Vda5050ActionHandlerBase")
{
  agv_state_->operating_mode = vda5050_msgs::msg::AGVState().AUTOMATIC;
  agv_state_->safety_state.e_stop = vda5050_msgs::msg::SafetyState().NONE;
  agv_state_->safety_state.field_violation = false;
  // Initialize factsheet
  factsheet_->type_specification.agv_class = robot_type_;
  factsheet_->physical_parameters.speed_max = 1;
  if (robot_type_ == "MANIPULATOR" || robot_type_ == "CONVEYOR") {
    factsheet_->physical_parameters.speed_max = 0;
  }
  const std::unordered_set<std::string> valid_robot_types = {"MANIPULATOR", "CARRIER", "FORKLIFT",
    "CONVEYOR", "TUGGER", "HUMANOID"};
  if (valid_robot_types.count(robot_type_) == 0) {
    RCLCPP_WARN(get_logger(), "Unknown robot type: %s", robot_type_.c_str());
  }

  RCLCPP_INFO(get_logger(), "Loading action handlers from config file: %s", config_file_.c_str());
  const YAML::Node config = YAML::LoadFile(config_file_);
  for (const auto & action_handler : config["action_handlers"]) {
    const std::string action_handler_name = action_handler.as<std::string>();
    const std::string plugin = config[action_handler_name]["plugin"].as<std::string>();
    try {
      auto plugin_instance = action_handler_loader_.createSharedInstance(plugin);
      RCLCPP_INFO(get_logger(), "Loading action handler: %s", action_handler_name.c_str());
      plugin_instance->Initialize(this, config[action_handler_name]);
      for (const auto & action_type : config[action_handler_name]["action_types"]) {
        const std::string action_type_name = action_type.as<std::string>();
        action_handler_map_[action_type_name] = plugin_instance;
      }
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_WARN(get_logger(),
        "Skipping action handler '%s' (plugin '%s') due to load error: %s",
        action_handler_name.c_str(), plugin.c_str(), e.what());
      continue;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "Skipping action handler '%s' (plugin '%s') due to exception: %s",
        action_handler_name.c_str(), plugin.c_str(), e.what());
      continue;
    }
  }
  // Callback group for services
  service_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  executor_.add_callback_group(service_callback_group_, this->get_node_base_interface());

  // tf_buffer and listener to get current robot positon
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (status_check_service_ != "") {
    status_check_client_ = create_client<lifecycle_msgs::srv::GetState>(
      status_check_service_, rclcpp::ServicesQoS(), service_callback_group_);
  }
  client_state_ = ClientState::IDLE;
  RCLCPP_INFO(get_logger(), "Vda5050ClientNode initialized");
}

std::string CreateISO8601Timestamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto itt = std::chrono::system_clock::to_time_t(now);
  std::ostringstream ss;
  struct tm buf;
  gmtime_r(&itt, &buf);
  ss << std::put_time(&buf, "%FT%TZ");
  return ss.str();
}

std::vector<vda5050_msgs::msg::ErrorReference> Vda5050ClientNode::CreateErrorReferenceList(
  const std::vector<std::pair<std::string, std::string>> & error_refs)
{
  std::vector<vda5050_msgs::msg::ErrorReference> error_references;
  for (const auto & error_ref : error_refs) {
    error_references.push_back(CreateErrorReference(error_ref.first, error_ref.second));
  }
  return error_references;
}

vda5050_msgs::msg::ErrorReference Vda5050ClientNode::CreateErrorReference(
  const std::string & reference_key, const std::string & reference_value)
{
  auto error_reference = vda5050_msgs::msg::ErrorReference();
  error_reference.reference_key = reference_key;
  error_reference.reference_value = reference_value;
  return error_reference;
}

vda5050_msgs::msg::Error Vda5050ClientNode::CreateError(
  ErrorLevel level, const std::string & error_msg,
  const std::vector<std::pair<std::string, std::string>> & error_refs,
  const std::string & error_type)
{
  auto error = vda5050_msgs::msg::Error();
  switch (level) {
    case ErrorLevel::WARNING:
      error.error_level = error.WARNING;
      break;
    case ErrorLevel::FATAL:
      error.error_level = error.FATAL;
      break;
    default:
      error.error_level = error.FATAL;
      break;
  }
  error.error_description = error_msg;
  error.error_references = CreateErrorReferenceList(error_refs);
  error.error_type = error_type;
  return error;
}

void Vda5050ClientNode::AddError(const vda5050_msgs::msg::Error & error)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  agv_state_->errors.push_back(error);
}

void Vda5050ClientNode::ExecuteOrderCallback()
{
  if (cancel_action_) {
    CancelOrder();
  }
  if (pause_action_) {
    PauseOrder();
  }
  // For a navigatable robot, it has to get position initialized
  // Get robot position
  if (factsheet_->physical_parameters.speed_max > 0) {
    try {
      // Find the latest map_T_base_link transform
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        "map", "base_link",
        tf2::TimePointZero);
      agv_state_->agv_position.x = t.transform.translation.x;
      agv_state_->agv_position.y = t.transform.translation.y;
      // Calculate robot orientation
      tf2::Quaternion quaternion;
      tf2::fromMsg(t.transform.rotation, quaternion);
      tf2::Matrix3x3 matrix(quaternion);
      double roll, pitch, yaw;
      matrix.getEulerYPR(yaw, pitch, roll);
      agv_state_->agv_position.theta = yaw;
      agv_state_->agv_position.position_initialized = true;
      RCLCPP_INFO_ONCE(
        this->get_logger(), "Robot position initialized");
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 120000,
        "Could not get robot position: %s", ex.what());
      return;
    }
  }
  if (client_state_ == ClientState::RUNNING) {
    RCLCPP_DEBUG(get_logger(), "Executing order");
    // Stop executing order if there is a fatal error
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (agv_state_->errors.size() > 0 &&
      agv_state_->errors[0].error_level == vda5050_msgs::msg::Error::FATAL)
    {
      RCLCPP_ERROR(get_logger(), "Fatal error detected, stopping order execution");
      // Set client state to IDLE if no running actions
      for (size_t i = 0; i < num_actions_; i++) {
        if (agv_state_->action_states[i].action_status == VDAActionState::RUNNING ||
          agv_state_->action_states[i].action_status == VDAActionState::INITIALIZING)
        {
          return;
        }
      }
      RCLCPP_INFO(get_logger(), "Order %s is completed with fatal error. Mission Client is IDLE.",
            current_order_->order_id.c_str());
      client_state_ = ClientState::IDLE;
      return;
    }
    lock.unlock();
    // Check if the robot has reached the current node
    if (reached_waypoint_) {
      // Go through the actions in the current node
      bool stop_driving = false;
      bool has_running_action = false;
      for (size_t action_idx = 0; action_idx < current_order_->nodes[current_node_].actions.size();
        action_idx++)
      {
        std::string action_id =
          current_order_->nodes[current_node_].actions[action_idx].action_id;
        std::string action_status = GetActionState(action_id);
        std::string blocking_type =
          current_order_->nodes[current_node_].actions[action_idx].blocking_type;
        // If blocking type is empty, it is regarded as HARD blocking
        bool running_or_initializing = action_status == VDAActionState::RUNNING ||
          action_status == VDAActionState::INITIALIZING;
        if (blocking_type.empty()) {
          blocking_type = vda5050_msgs::msg::Action::HARD;
        }
        if (action_status == VDAActionState::FINISHED || action_status == VDAActionState::FAILED) {
          continue;
        } else if (running_or_initializing) {
          has_running_action = true;
          if (blocking_type == vda5050_msgs::msg::Action::HARD) {
            // Hard blocking action is running. Wait for it to finish.
            return;
          }
          if (blocking_type == vda5050_msgs::msg::Action::SOFT) {
            stop_driving = true;
          }
        } else if (action_status == VDAActionState::WAITING) {
          if (blocking_type == vda5050_msgs::msg::Action::HARD) {
            // Hard blocking action is waiting. Execute it if there is no running action.
            if (!has_running_action) {
              ExecuteAction(
                current_order_->nodes[current_node_].actions[action_idx]);
            }
            // Wait for the hard blocking action to finish
            return;
          } else {
            if (blocking_type == vda5050_msgs::msg::Action::SOFT) {
              stop_driving = true;
            }
            // Execute soft/none blocking action
            ExecuteAction(
              current_order_->nodes[current_node_].actions[action_idx]);
            has_running_action = true;
          }
        }
      }
      // SOFT or HARD blocking action is running, stop driving
      if (stop_driving) {
        RCLCPP_DEBUG(get_logger(), "SOFT or HARD blocking action is running, stop driving");
        return;
      }
      // SOFT and HARD actions are all done, go to next node
      next_stop_++;
      // Check if the order is completed.
      if (next_stop_ >= current_order_->nodes.size()) {
        RCLCPP_INFO(
          get_logger(), "Order %s is completed.", current_order_->order_id.c_str());
        PublishRobotState();
        client_state_ = ClientState::IDLE;
      } else {
        reached_waypoint_ = false;
        NavigateThroughPoses();
      }
    }
  }
}

void Vda5050ClientNode::PublishRobotState()
{
  agv_state_->timestamp = CreateISO8601Timestamp();
  order_info_pub_->publish(*agv_state_);
}

void Vda5050ClientNode::StateTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  PublishRobotState();
}

void Vda5050ClientNode::OrderIdCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (current_order_) {
    auto order_id_msg = std_msgs::msg::String();
    order_id_msg.data = current_order_->order_id;
    order_id_pub_->publish(order_id_msg);
  }
}

void Vda5050ClientNode::NavigateThroughPoses()
{
  RCLCPP_DEBUG(get_logger(), "Navigating through poses");
  if (status_check_service_ != "" && current_node_ == 0) {
    try {
      auto status_check_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto result = SendServiceRequest<lifecycle_msgs::srv::GetState>(
        status_check_client_, status_check_request);
      if (result->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(get_logger(), "The status check node is not active");
        // Reset states to retry in next cycle
        next_stop_ = current_node_;
        reached_waypoint_ = true;
        return;
      } else {
        RCLCPP_DEBUG(get_logger(), "The status check node is active");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to check status: %s", e.what());
      // Reset states to retry in next cycle
      next_stop_ = current_node_;
      reached_waypoint_ = true;
      return;
    }
  }
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Navigation server not available");
    return;
  }
  if (!current_order_) {
    RCLCPP_ERROR(get_logger(), "Navigation was called when no order exists.");
  }
  if (next_stop_ >= current_order_->nodes.size()) {
    RCLCPP_INFO(get_logger(), "Navigation completed");
    return;
  }
  auto goal_msg = NavThroughPoses::Goal();
  for (size_t i = current_node_ + 1; i < current_order_->nodes.size(); i++) {
    auto pose_stamped = geometry_msgs::msg::PoseStamped();

    pose_stamped.pose.position.x =
      current_order_->nodes[i].node_position.x;
    pose_stamped.pose.position.y =
      current_order_->nodes[i].node_position.y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.header.frame_id = "map";
    // Convert theta into a quaternion for goal pose's orientation
    tf2::Quaternion orientation;
    orientation.setRPY(
      0, 0,
      current_order_->nodes[i].node_position.theta);
    pose_stamped.pose.orientation = tf2::toMsg(orientation);
    pose_stamped.header.stamp = rclcpp::Clock().now();
    goal_msg.poses.push_back(pose_stamped);
    if (current_order_->nodes[i].actions.size() > 0 ||
      i == current_order_->nodes.size() - 1 ||
      current_order_->nodes[i].node_position.allowed_deviation_x_y == 0)
    {
      next_stop_ = i;
      break;
    }
  }
  auto send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(
    &Vda5050ClientNode::NavGoalResponseCallback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(
    &Vda5050ClientNode::NavFeedbackCallback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(
    &Vda5050ClientNode::NavResultCallback, this,
    std::placeholders::_1);
  RCLCPP_INFO(
    get_logger(), "Sending goal for (x: %f, y: %f, t: %f)",
    current_order_->nodes[next_stop_].node_position.x,
    current_order_->nodes[next_stop_].node_position.y,
    current_order_->nodes[next_stop_].node_position.theta);
  agv_state_->driving = true;
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void Vda5050ClientNode::ExecuteAction(const vda5050_msgs::msg::Action & vda5050_action)
{
  RCLCPP_DEBUG(
    get_logger(), "Executing action: %s", vda5050_action.action_id.c_str());
  UpdateActionState(vda5050_action, VDAActionState::RUNNING);
  if (vda5050_action.action_type == "pause_order") {
    // Set the pause_order to running untill stopTeleop action is triggered
    pause_order_ = true;
    UpdateActionStateById(vda5050_action.action_id, VDAActionState::RUNNING);
    pause_order_action_id_ = vda5050_action.action_id;
    return;
  }
  auto handler = action_handler_map_.find(vda5050_action.action_type);
  if (handler != action_handler_map_.end()) {
    handler->second->Execute(vda5050_action);
  } else {
    RCLCPP_ERROR(get_logger(), "Action handler not found for action type: %s",
          vda5050_action.action_type.c_str());
    UpdateActionState(vda5050_action, VDAActionState::FAILED, "Action handler not found");
  }
}

void Vda5050ClientNode::InitAGVState()
{
  RCLCPP_DEBUG(this->get_logger(), "Initialization order information");
  std::lock_guard<std::mutex> lock(state_mutex_);
  size_t num_last_order_actions = num_actions_;
  num_actions_ = 0;
  for (const auto & vda5050_node : current_order_->nodes) {
    num_actions_ += vda5050_node.actions.size();
  }
  agv_state_->operating_mode = vda5050_msgs::msg::AGVState().AUTOMATIC;
  agv_state_->safety_state.e_stop = vda5050_msgs::msg::SafetyState().NONE;
  agv_state_->safety_state.field_violation = false;
  agv_state_->order_id = current_order_->order_id;
  agv_state_->last_node_id = current_order_->nodes[0].node_id;
  agv_state_->driving = false;
  agv_state_->informations.clear();
  agv_state_->errors.clear();
  agv_state_->node_states.clear();
  agv_state_->edge_states.clear();
  agv_state_->paused = false;
  reached_waypoint_ = true;

  // clear action states from last order
  if (num_last_order_actions > 0 && agv_state_->action_states.size() >= num_last_order_actions) {
    agv_state_->action_states.erase(
      agv_state_->action_states.begin(),
      agv_state_->action_states.begin() + num_last_order_actions);
  }
  // clear completed instant actions
  agv_state_->action_states.erase(
    std::remove_if(
      agv_state_->action_states.begin(),
      agv_state_->action_states.end(),
      [](const VDAActionState & state) {
        return state.action_status == VDAActionState::FINISHED ||
               state.action_status == VDAActionState::FAILED;
      }),
    agv_state_->action_states.end());

  size_t action_index = 0;
  for (const auto & vda5050_node : current_order_->nodes) {
    auto node_state = vda5050_msgs::msg::NodeState();
    node_state.node_id = vda5050_node.node_id;
    node_state.sequence_id = vda5050_node.sequence_id;
    node_state.node_description = vda5050_node.node_description;
    node_state.released = vda5050_node.released;
    agv_state_->node_states.push_back(node_state);
    for (const auto & vda5050_action : vda5050_node.actions) {
      auto actionState = VDAActionState();
      actionState.action_id = vda5050_action.action_id;
      actionState.action_type = vda5050_action.action_type;
      actionState.action_description = vda5050_action.action_description;
      actionState.action_status =
        VDAActionState::WAITING;
      agv_state_->action_states.insert(agv_state_->action_states.begin() + action_index,
            actionState);
      action_index++;
    }
  }
  agv_state_->node_states.erase(
    agv_state_->node_states.begin());
  reached_waypoint_ = true;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Obtained %ld node states and %ld action states",
    agv_state_->node_states.size(),
    agv_state_->action_states.size());

  client_state_ = ClientState::RUNNING;
}

void Vda5050ClientNode::OrderCallback(const vda5050_msgs::msg::Order::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Order with order_id %s received",
    msg->order_id.c_str());
  if (!CanAcceptOrder()) {
    if (msg->order_id != current_order_->order_id) {
      RCLCPP_INFO(
        get_logger(), "One order is running. Order %s is ignored",
        msg->order_id.c_str());
      vda5050_msgs::msg::Error error = CreateError(
        ErrorLevel::WARNING, "An order is running",
        {{"orderId", msg->order_id}},
        kOrderUpdateError);
      AddError(error);
    }
  } else if (!msg->nodes.empty()) {
    current_order_ = msg;
    current_node_ = 0;
    next_stop_ = 0;
    current_node_action_ = 0, current_action_state_ = 0;
    InitAGVState();
  }
}

void Vda5050ClientNode::BatteryStateCallback(
  const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  agv_state_->battery_state.battery_charge = msg->percentage * 100;
  agv_state_->battery_state.battery_voltage = msg->voltage;
  // POWER_SUPPLY_STATUS_CHARGING = 1
  agv_state_->battery_state.charging = (msg->power_supply_status == 1) ? true : false;

  // battery_health and reach are currently not supported
  agv_state_->battery_state.battery_health = 0;
  agv_state_->battery_state.reach = 0;
}

void Vda5050ClientNode::OdometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  agv_state_->velocity.vx = msg->twist.twist.linear.x;
  agv_state_->velocity.vy = msg->twist.twist.linear.y;
  agv_state_->velocity.omega = msg->twist.twist.angular.z;
}

void Vda5050ClientNode::InstantActionsCallback(
  const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Instant actions with header_id %d received",
    msg->header_id);
  std::unique_lock<std::mutex> lock(state_mutex_);

  for (const vda5050_msgs::msg::Action & action : msg->instant_actions) {
    // Get action state from current state
    auto it = std::find_if(
      agv_state_->action_states.begin(),
      agv_state_->action_states.end(),
      [&action](const VDAActionState & action_state) {
        return action_state.action_id == action.action_id;
      });
    if (it != agv_state_->action_states.end()) {
      continue;
    }
    RCLCPP_INFO(
      this->get_logger(), "Processing action %s of type %s.",
      action.action_id.c_str(), action.action_type.c_str());
    // Add action to action_states
    auto action_state = VDAActionState();
    action_state.action_id = action.action_id;
    action_state.action_description = action.action_description;
    action_state.action_status = VDAActionState::WAITING;
    action_state.action_type = action.action_type;
    agv_state_->action_states.push_back(action_state);
    lock.unlock();

    if (action.action_type == "cancelOrder") {
      if (cancel_action_ != nullptr) {
        RCLCPP_INFO(get_logger(), "Cancel order action is already running");
        UpdateActionState(action, VDAActionState::FAILED);
      }
      cancel_action_ = std::make_shared<vda5050_msgs::msg::Action>(action);
    } else if (action.action_type == "startTeleop" || action.action_type == "stopTeleop") {
      TeleopActionHandler(action);
    } else if (action.action_type == "factsheetRequest") {
      FactsheetRequestHandler(action);
    } else if (action.action_type == "startPause") {
      pause_action_ = std::make_shared<vda5050_msgs::msg::Action>(action);
      PauseOrder();
    } else if (action.action_type == "stopPause") {
      ResumeOrder(action);
    } else {
      auto handler = action_handler_map_.find(action.action_type);
      if (handler != action_handler_map_.end()) {
        handler->second->Execute(action);
      } else {
        RCLCPP_ERROR(get_logger(), "Action handler not found for action type: %s",
          action.action_type.c_str());
      }
    }
  }
}

void Vda5050ClientNode::TeleopActionHandler(const vda5050_msgs::msg::Action & teleop_action)
{
  UpdateActionState(teleop_action, VDAActionState::RUNNING);
  if (teleop_action.action_type == "startTeleop") {
    // Cancel any running navigation goal
    pause_order_ = true;
    client_state_ = ClientState::PAUSED;
    if (nav_goal_handle_) {
      client_ptr_->async_cancel_goal(nav_goal_handle_);
      RCLCPP_INFO(
        this->get_logger(), "Start teleop: cancelling current mission node.");
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Finished teleop. Resume order.");
    pause_order_ = false;
    client_state_ = ClientState::RUNNING;
    if (pause_order_action_id_ != "") {
      UpdateActionStateById(pause_order_action_id_, VDAActionState::FINISHED);
      pause_order_action_id_ = "";
    }
    // Reset states to resume navigation
    next_stop_ = current_node_;
    reached_waypoint_ = true;
  }
  UpdateActionState(teleop_action, VDAActionState::FINISHED);
}

void Vda5050ClientNode::FactsheetRequestHandler(
  const vda5050_msgs::msg::Action & factsheet_request)
{
  UpdateActionState(factsheet_request, VDAActionState::RUNNING);

  if (factsheet_request.action_type == "factsheetRequest") {
    // publish the factsheet over the "factsheet" topic
    PublishRobotFactsheet();
  }

  UpdateActionState(factsheet_request, VDAActionState::FINISHED);
}

void Vda5050ClientNode::PublishRobotFactsheet()
{
  factsheet_->timestamp = CreateISO8601Timestamp();
  factsheet_info_pub_->publish(*factsheet_);
}

void Vda5050ClientNode::CancelOrder()
{
  RCLCPP_INFO(get_logger(), "Cancelling order");
  if (client_state_ == ClientState::IDLE) {
    RCLCPP_ERROR(get_logger(),
          "cancelOrder action request failed. There is no active order running.");
    // The AGV must report a "noOrderToCancel" error with the errorLevel set to
    // warning. The actionId of the instantAction must be passed as an errorReference.
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::WARNING, "There is no active order running.",
      {{"action_id", cancel_action_->action_id}},
      "noOrderToCancel");
    AddError(error);
    cancel_action_.reset();
    UpdateActionState(*cancel_action_, VDAActionState::FAILED);
    return;
  }
  auto cancel_state = GetActionState(cancel_action_->action_id);
  if (cancel_state == VDAActionState::WAITING) {
    // Set cancelOrder action state to running
    UpdateActionState(*cancel_action_, VDAActionState::RUNNING);
    // Cancel nav goal
    if (nav_goal_handle_) {
      client_ptr_->async_cancel_goal(nav_goal_handle_);
    }
  }
  // Cancel all actions in current order
  bool all_actions_canceled = true;
  for (size_t i = 0; i < num_actions_; i++) {
    const std::string action_id = agv_state_->action_states[i].action_id;
    const std::string action_state = GetActionState(action_id);
    // Set waiting actions to failed
    bool running_or_initializing = action_state == VDAActionState::RUNNING ||
      action_state == VDAActionState::INITIALIZING;
    if (action_state == VDAActionState::WAITING) {
      UpdateActionStateById(action_id, VDAActionState::FAILED);
    } else if (running_or_initializing) {
      all_actions_canceled = false;
      if (canceled_action_ids_.find(action_id) == canceled_action_ids_.end()) {
        canceled_action_ids_.insert(action_id);
        auto handler = action_handler_map_.find(GetAction(action_id).action_type);
        if (handler != action_handler_map_.end()) {
          handler->second->Cancel(action_id);
        }
      }
    }
  }
  // Once all the VDA actions and navigation goal requests have finished,
  // the cancel order will be mark as finished
  if (all_actions_canceled && nav_goal_handle_ == nullptr) {
    UpdateActionState(*cancel_action_, VDAActionState::FINISHED);
    cancel_action_.reset();
    canceled_action_ids_.clear();
    client_state_ = ClientState::IDLE;
    RCLCPP_INFO(get_logger(), "Finished cancelOrder.");
  }
}

void Vda5050ClientNode::UpdateActionStateById(
  const std::string & action_id,
  const std::string & status,
  const std::string & result_description)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = std::find_if(
    agv_state_->action_states.begin(),
    agv_state_->action_states.end(),
    [&action_id](const VDAActionState & action_state) {
      return action_state.action_id == action_id;
    });
  if (it == agv_state_->action_states.end()) {
    RCLCPP_ERROR(
      get_logger(), "Error while processing action state. Couldn't find action with id: %s",
      action_id.c_str());
  } else {
    it->action_status = status;
    RCLCPP_DEBUG(get_logger(), "Updated action state for action %s to %s",
      action_id.c_str(), status.c_str());
    it->result_description = result_description;
    PublishRobotState();
  }
}

void Vda5050ClientNode::UpdateActionState(
  const vda5050_msgs::msg::Action & action,
  const std::string & status,
  const std::string & result_description)
{
  UpdateActionStateById(action.action_id, status, result_description);
}

void Vda5050ClientNode::InfoCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (agv_state_->informations.size()) {
    auto & info = agv_state_->informations.at(0);
    info.info_type = "user_info";
    info.info_description = msg->data;
    info.info_level = info.INFO;
  }
}

void Vda5050ClientNode::NavGoalResponseCallback(
  const rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Goal was rejected by server");
    auto error = vda5050_msgs::msg::Error();
    error = CreateError(
      ErrorLevel::FATAL, "Goal rejected",
      {{"node_id", current_order_->nodes[current_node_].node_id}});
    agv_state_->driving = false;
    agv_state_->errors.push_back(error);
    nav_goal_handle_.reset();
    PublishRobotState();
    client_state_ = ClientState::IDLE;
  } else {
    nav_goal_handle_ = goal_handle;
    agv_state_->driving = true;
    RCLCPP_DEBUG(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void Vda5050ClientNode::NavFeedbackCallback(
  GoalHandleNavThroughPoses::SharedPtr,
  const NavThroughPoses::Feedback::ConstSharedPtr feedback)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_node_ = next_stop_ - feedback->number_of_poses_remaining;
  agv_state_->last_node_id =
    current_order_->nodes[current_node_].node_id;
  agv_state_->last_node_sequence_id =
    current_order_->nodes[current_node_].sequence_id;
  agv_state_->driving = true;
  while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1) {
    agv_state_->node_states.erase(
      agv_state_->node_states.begin());
  }
}

void Vda5050ClientNode::NavResultCallback(
  const GoalHandleNavThroughPoses::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto error = vda5050_msgs::msg::Error();
  nav_goal_handle_.reset();
  agv_state_->driving = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        get_logger(), "Nav goal was aborted at node_id %s",
        agv_state_->last_node_id.c_str());
      current_node_ = 1;
      error = CreateError(
        ErrorLevel::FATAL, "Nav goal aborted",
        {{"node_id", current_order_->nodes[current_node_].node_id}});
      agv_state_->errors.push_back(error);
      PublishRobotState();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(
        get_logger(), "Nav goal was canceled at node_id %s",
        agv_state_->last_node_id.c_str());
      // Report error if the order is not paused and cancel action is null
      if (pause_order_ != true && cancel_action_ == nullptr && pause_action_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "Navigation goal canceled unexpectedly");
        error = CreateError(
          ErrorLevel::FATAL, "Navigation goal canceled",
          {{"node_id", current_order_->nodes[current_node_].node_id}});
        agv_state_->errors.push_back(error);
      }
      nav_goal_handle_.reset();
      PublishRobotState();
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }
  // Logic if navigation was successful
  RCLCPP_INFO(get_logger(), "Reached order node: %ld", next_stop_);
  current_node_ = next_stop_;
  while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1) {
    agv_state_->node_states.erase(
      agv_state_->node_states.begin());
  }
  agv_state_->last_node_id =
    current_order_->nodes[current_node_].node_id;
  agv_state_->last_node_sequence_id =
    current_order_->nodes[current_node_].sequence_id;
  reached_waypoint_ = true;
}

void Vda5050ClientNode::OrderValidErrorCallback(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  vda5050_msgs::msg::Error error = CreateError(
    ErrorLevel::WARNING, "Malformed order",
    {{"Error", msg->data}},
    kValidationError);
  agv_state_->errors.push_back(error);
}

template<typename ServiceT>
typename ServiceT::Response::SharedPtr Vda5050ClientNode::SendServiceRequest(
  typename rclcpp::Client<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request)
{
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(get_logger(), "Service is not available.");
    throw std::runtime_error("Service is not available.");
  }
  auto result = client->async_send_request(request);
  if (executor_.spin_until_future_complete(result) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Failed to get service result.");
  }
  return result.get();
}

std::string Vda5050ClientNode::GetActionState(const std::string & action_id)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & action_state : agv_state_->action_states) {
    if (action_state.action_id == action_id) {
      return action_state.action_status;
    }
  }
  return "UNKNOWN";
}

const vda5050_msgs::msg::Action & Vda5050ClientNode::GetAction(const std::string & action_id)
{
  auto it = std::find_if(
    current_order_->nodes[current_node_].actions.begin(),
    current_order_->nodes[current_node_].actions.end(),
    [action_id](const vda5050_msgs::msg::Action & action) {
      return action.action_id == action_id;
    });
  if (it == current_order_->nodes[current_node_].actions.end()) {
    throw std::runtime_error("Action not found");
  }
  return *it;
}


bool Vda5050ClientNode::CanAcceptOrder()
{
  if (client_state_ == ClientState::IDLE) {
    return true;
  } else if (client_state_ == ClientState::PAUSED) {
    return false;
  } else if (client_state_ == ClientState::RUNNING) {
    // Check if there is a running action
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < num_actions_; i++) {
      if (agv_state_->action_states[i].action_status == VDAActionState::RUNNING ||
        agv_state_->action_states[i].action_status == VDAActionState::INITIALIZING)
      {
        return false;
      }
    }
    // No action is running.
    // Check if there is a fatal error
    if (agv_state_->errors.size() > 0 &&
      agv_state_->errors[0].error_level == vda5050_msgs::msg::Error::FATAL)
    {
      // Return false if there is a running action
      RCLCPP_INFO(get_logger(), "Order %s is completed with fatal error. Mission Client is IDLE.",
            current_order_->order_id.c_str());
      client_state_ = ClientState::IDLE;
      return true;
    }
    // Check if the robot is at the last node and no waiting action
    if (current_node_ == current_order_->nodes.size() - 1) {
      for (size_t i = 0; i < num_actions_; i++) {
        if (agv_state_->action_states[i].action_status == VDAActionState::WAITING) {
          return false;
        }
      }
      return true;
    }
    return false;
  }
  return false;
}

void Vda5050ClientNode::PauseOrder()
{
  RCLCPP_INFO(get_logger(), "Pausing order");
  static std::set<std::string> paused_action_ids;
  if (client_state_ == ClientState::IDLE) {
    RCLCPP_ERROR(get_logger(), "No active order to pause");
    UpdateActionState(*pause_action_, VDAActionState::FAILED, "No active order to pause");
    pause_action_.reset();
    return;
  } else if (client_state_ == ClientState::PAUSED) {
    RCLCPP_ERROR(get_logger(), "Order is already paused");
    UpdateActionState(*pause_action_, VDAActionState::FINISHED);
    pause_action_.reset();
    return;
  }
  const std::string pause_action_state = GetActionState(pause_action_->action_id);
  if (pause_action_state == VDAActionState::WAITING) {
    UpdateActionState(*pause_action_, VDAActionState::RUNNING);
    if (nav_goal_handle_) {
      client_ptr_->async_cancel_goal(nav_goal_handle_);
    }
  }
  // pause all actions in current order
  bool all_actions_paused = true;
  for (size_t i = 0; i < num_actions_; i++) {
    const std::string action_id = agv_state_->action_states[i].action_id;
    const std::string action_state = GetActionState(action_id);
    if (action_state == VDAActionState::RUNNING ||
      action_state == VDAActionState::INITIALIZING)
    {
      all_actions_paused = false;
      if (paused_action_ids.find(action_id) == paused_action_ids.end()) {
        paused_action_ids.insert(action_id);
        auto handler = action_handler_map_[GetAction(action_id).action_type];
        handler->Pause(action_id);
      }
    }
  }
  if (all_actions_paused && nav_goal_handle_ == nullptr) {
    UpdateActionState(*pause_action_, VDAActionState::FINISHED);
    pause_action_.reset();
    paused_action_ids.clear();
    client_state_ = ClientState::PAUSED;
    RCLCPP_INFO(get_logger(), "Finished pauseOrder.");
  }
}

void Vda5050ClientNode::ResumeOrder(const vda5050_msgs::msg::Action & action)
{
  RCLCPP_INFO(get_logger(), "Resuming order");
  if (client_state_ != ClientState::PAUSED) {
    RCLCPP_ERROR(get_logger(), "Order is not paused");
    UpdateActionState(action, VDAActionState::FAILED, "Order is not paused");
    return;
  }
  UpdateActionState(action, VDAActionState::RUNNING);
  for (size_t i = 0; i < num_actions_; i++) {
    const std::string action_id = agv_state_->action_states[i].action_id;
    const std::string action_state = GetActionState(action_id);
    if (action_state == VDAActionState::PAUSED) {
      UpdateActionStateById(agv_state_->action_states[i].action_id, VDAActionState::RUNNING);
      const std::string action_type = GetAction(action_id).action_type;
      auto handler = action_handler_map_[action_type];
      handler->Resume(action_id);
    }
  }
  next_stop_ = current_node_;
  reached_waypoint_ = true;
  client_state_ = ClientState::RUNNING;
  UpdateActionState(action, VDAActionState::FINISHED);
}

Vda5050ClientNode::~Vda5050ClientNode() = default;

}  // namespace mission_client
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  isaac_ros::mission_client::Vda5050ClientNode)
