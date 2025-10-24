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

/**
 * This repository implements data types and logic specified in the VDA5050
 * protocol, which is specified here
 * https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md
 */

#ifndef ISAAC_ROS_VDA5050_CLIENT__VDA5050_CLIENT_NODE_HPP_
#define ISAAC_ROS_VDA5050_CLIENT__VDA5050_CLIENT_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "pluginlib/class_loader.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/order.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/agv_state.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "vda5050_msgs/msg/node_state.hpp"
#include "vda5050_msgs/msg/node.hpp"
#include "vda5050_msgs/msg/edge_state.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"
#include "vda5050_action_handler/vda5050_action_handler.hpp"

namespace isaac_ros
{
namespace mission_client
{
class Vda5050ActionHandlerBase;
class Vda5050ClientNode : public rclcpp::Node
{
public:
  using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavThroughPoses = rclcpp_action::ClientGoalHandle<NavThroughPoses>;
  using NavClient = rclcpp_action::Client<NavThroughPoses>;
  using NavCancelResponse = NavClient::CancelResponse;

  using VDAActionState = vda5050_msgs::msg::ActionState;
  enum ErrorLevel { WARNING, FATAL };
  enum ClientState
  {
    IDLE,
    RUNNING,
    PAUSED
  };

  explicit Vda5050ClientNode(const rclcpp::NodeOptions & options);

  // Update action status and description based on the action id
  void UpdateActionState(
    const vda5050_msgs::msg::Action & action,
    const std::string & status,
    const std::string & result_description = "");

  // Update action state based on the action id
  void UpdateActionStateById(
    const std::string & action_id,
    const std::string & status,
    const std::string & result_description = "");

  template<typename ActionType>
  void ActionResponseCallback(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr & goal,
    const vda5050_msgs::msg::Action & action)
  {
    if (!goal) {
      RCLCPP_ERROR(get_logger(), "Action %s was rejected", action.action_id.c_str());
      UpdateActionState(action, VDAActionState::FAILED, "Action was rejected");
    } else {
      RCLCPP_INFO(get_logger(), "Action %s was accepted", action.action_id.c_str());
      UpdateActionState(action, VDAActionState::RUNNING, "Action was accepted");
    }
    PublishRobotState();
  }

  template<typename ResultType>
  void ActionResultCallback(
    const vda5050_msgs::msg::Action & action,
    const ResultType & result,
    const bool & success,
    const std::string & description)
  {
    RCLCPP_INFO(get_logger(), "Action %s result: %s", action.action_id.c_str(),
          description.c_str());
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (success) {
          RCLCPP_INFO(get_logger(), "Action %s was succeeded", action.action_id.c_str());
          UpdateActionState(action, VDAActionState::FINISHED, description);
        } else {
          RCLCPP_ERROR(get_logger(), "Action %s was failed", action.action_id.c_str());
          UpdateActionState(action, VDAActionState::FAILED, description);
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Action %s was aborted", action.action_id.c_str());
        UpdateActionState(action, VDAActionState::FAILED, "Action is aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Action %s was canceled", action.action_id.c_str());
        UpdateActionState(action, VDAActionState::FAILED, "Action is canceled");
        break;
      default:
        break;
    }
    PublishRobotState();
  }

  std::vector<vda5050_msgs::msg::ErrorReference> CreateErrorReferenceList(
    const std::vector<std::pair<std::string, std::string>> & error_refs);

  // Create an error message
  vda5050_msgs::msg::Error CreateError(
    ErrorLevel level, const std::string & error_msg,
    const std::vector<std::pair<std::string, std::string>> & error_refs,
    const std::string & error_type = "");
  // Add error to the agv_state_
  void AddError(const vda5050_msgs::msg::Error & error);

  ~Vda5050ClientNode();

private:
  // Publish a vda5050_msgs/AGVState based on the current state of the robot
  void PublishRobotState();
  // Publish a vda5050_msgs/Factsheet based on the robot's sepcifications
  void PublishRobotFactsheet();
  // Timer callback function to publish a vda5050_msgs/AGVState message
  void StateTimerCallback();
  // The callback function when the node receives a vda5050_msgs/Order message and processes it
  void OrderCallback(const vda5050_msgs::msg::Order::ConstSharedPtr msg);
  // Execute order callback
  void ExecuteOrderCallback();
  // Function that creates the NavigateThroughPoses goal message for Nav2 and sends that goal
  // asynchronously
  void NavigateThroughPoses();
  // Vda5050 action handler: check actions in the current node and send requests to trigger
  // different servers based on the action type
  void ExecuteAction(const vda5050_msgs::msg::Action & vda5050_action);
  // Initialization the order state once received a new order
  void InitAGVState();
  // The callback function when the node receives a sensor_msgs/BatteryState message and processes
  // it into a VDA5050 BatteryState message
  void BatteryStateCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  // The callback function when the node receives a nav_msgs/Odometry message and appends it's
  // velotity to the status message's velocity that gets published
  void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  // The callback function when the node receives a std_msgs/String info message and appends it to
  // the status message that gets published
  void InfoCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  // Timer callback function to publish a std_msgs/String message containing the order_id
  void OrderIdCallback();
  // Goal response callback for NavigateThroughPoses goal message
  void NavGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr & goal);
  // Feedback callback for NavigateThroughPoses goal message
  void NavFeedbackCallback(
    GoalHandleNavThroughPoses::SharedPtr,
    const NavThroughPoses::Feedback::ConstSharedPtr);
  // Result callback for NavigateThroughPoses goal message
  void NavResultCallback(const GoalHandleNavThroughPoses::WrappedResult & result);

  void CancelOrder();
  void InstantActionsCallback(const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg);
  // Handle teleop instant actions
  void TeleopActionHandler(const vda5050_msgs::msg::Action & teleop_action);
  // Handle factsheet instant actions
  void FactsheetRequestHandler(const vda5050_msgs::msg::Action & factsheet_request);
  // The callback function when the node receives an order error message.
  void OrderValidErrorCallback(const std_msgs::msg::String::ConstSharedPtr msg);
  // Sync service request. Return request result
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr SendServiceRequest(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request);

  std::string GetActionState(const std::string & action_id);
  const vda5050_msgs::msg::Action & GetAction(const std::string & action_id);

  vda5050_msgs::msg::ErrorReference CreateErrorReference(
    const std::string & reference_key, const std::string & reference_value);

  bool CanAcceptOrder();

  void PauseOrder();
  void ResumeOrder(const vda5050_msgs::msg::Action & action);

  // Node parameters
  // The period to update the robot state and mission status messages (in seconds)
  double update_feedback_period_{};
  // The topic to get robot velocity
  std::string odom_topic_;
  // The topic to get robot battery state
  std::string battery_state_topic_;
  std::string robot_type_;
  std::string status_check_service_;
  std::string config_file_;

  // Subscribers
  rclcpp::Subscription<vda5050_msgs::msg::Order>::SharedPtr order_sub_;
  rclcpp::Subscription<vda5050_msgs::msg::InstantActions>::SharedPtr
    instant_actions_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_valid_error_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr info_sub_;

  // Publishers
  rclcpp::Publisher<vda5050_msgs::msg::AGVState>::SharedPtr order_info_pub_;
  rclcpp::Publisher<vda5050_msgs::msg::Factsheet>::SharedPtr factsheet_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_id_pub_;

  // service clients
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr status_check_client_;

  // Action clients
  rclcpp_action::Client<NavThroughPoses>::SharedPtr client_ptr_;
  GoalHandleNavThroughPoses::SharedPtr nav_goal_handle_;

  // Callback group for state timer callback
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  // Executor for service reqeust callbacks
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Timer to call PublishRobotState periodically
  rclcpp::TimerBase::SharedPtr robot_state_timer_;
  // Timer to execute order
  rclcpp::TimerBase::SharedPtr execute_order_timer_;
  // Timer to publish order_id to JsonInfoGenerator
  rclcpp::TimerBase::SharedPtr order_id_timer_;
  // Mutex to protect private member variables when they are read or written to
  std::mutex state_mutex_;

  vda5050_msgs::msg::Order::ConstSharedPtr current_order_;
  // Order information for feedback of the mission
  vda5050_msgs::msg::AGVState::SharedPtr agv_state_;
  // Factsheet information for robot
  vda5050_msgs::msg::Factsheet::SharedPtr factsheet_;
  // Number of actions in the current order
  size_t num_actions_;
  // Cancel action
  vda5050_msgs::msg::Action::SharedPtr cancel_action_;
  // Pause action
  vda5050_msgs::msg::Action::SharedPtr pause_action_;
  // Action ids that action handler Cancel() is called
  // This is used to avoid calling Cancel() multiple times for the same action
  std::set<std::string> canceled_action_ids_;
  // Reached current waypoint flag
  bool reached_waypoint_;
  // Pause order
  bool pause_order_;
  std::string pause_order_action_id_ = "";
  // Current node the robot is working on
  size_t current_node_{};
  // Then last node of navigate_through_poses action
  size_t next_stop_{};
  // Current action the robot is working on
  size_t current_node_action_{};
  // Current action state to update
  size_t current_action_state_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  ClientState client_state_;
  std::unordered_map<std::string, std::shared_ptr<Vda5050ActionHandlerBase>> action_handler_map_;
  pluginlib::ClassLoader<Vda5050ActionHandlerBase> action_handler_loader_;
};

}  // namespace mission_client
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VDA5050_CLIENT__VDA5050_CLIENT_NODE_HPP_
