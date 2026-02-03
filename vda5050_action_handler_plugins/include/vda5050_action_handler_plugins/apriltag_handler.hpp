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

#ifndef VDA5050_ACTION_HANDLER_PLUGINS__APRILTAG_HANDLER_HPP_
#define VDA5050_ACTION_HANDLER_PLUGINS__APRILTAG_HANDLER_HPP_

#include "vda5050_action_handler/vda5050_action_handler.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include <map>

namespace isaac_ros
{
namespace mission_client
{

/**
 * @brief VDA5050 action handler plugin for AprilTag detection
 *
 * Handles the 'get_apriltags' action type by collecting AprilTag detections
 * over a configurable timeout period and returning unique detections as JSON.
 *
 * Input: VDA5050 action with type 'get_apriltags'
 * Output: JSON array of detected AprilTags with pose and metadata
 */
class AprilTagHandler : public Vda5050ActionHandlerBase
{
public:
  /**
   * @brief Initialize the AprilTag handler plugin
   * @param client_node Pointer to the VDA5050 client node
   * @param config YAML configuration node
   */
  void Initialize(
    Vda5050ClientNode * client_node,
    const YAML::Node & config) override;

  /**
   * @brief Execute AprilTag detection action
   * @param vda5050_action The VDA5050 action to execute
   */
  void Execute(const vda5050_msgs::msg::Action & vda5050_action) override;

private:
  // AprilTag detection parameters
  std::string apriltag_detections_topic_;
  double apriltag_detection_buffer_time_;  // Time to collect detections (e.g. 1.0s)
  double apriltag_action_timeout_;         // Total action timeout (e.g. 5.0s)
  std::string apriltag_target_frame_;
  std::string apriltag_transform_mode_;
  double apriltag_transform_timeout_;
  std::string apriltag_collection_strategy_;
  bool verbose_;

  // Subscriber for AprilTag detections
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    apriltag_detections_sub_;

  // Cache for latest AprilTag detections
  isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr latest_apriltag_detections_;
  std::mutex apriltag_detections_mutex_;
  rclcpp::Time last_apriltag_detection_time_;

  // TF2 for coordinate transformations
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Collection window state
  struct AprilTagCollectionState
  {
    bool is_collecting = false;
    std::string action_id;
    size_t action_state_idx = 0;
    rclcpp::Time collection_start_time;
    rclcpp::Time action_start_time;
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr best_detections;
    rclcpp::TimerBase::SharedPtr collection_timer;
    rclcpp::TimerBase::SharedPtr action_timeout_timer;
  } apriltag_collection_state_;
  std::mutex apriltag_collection_mutex_;

  // Unique detection collection by tag ID (for buffering flickery detections)
  std::map<int, isaac_ros_apriltag_interfaces::msg::AprilTagDetection> unique_apriltag_detections_;

  // AprilTag action handler and utilities
  void executeGetAprilTags(const vda5050_msgs::msg::Action & vda5050_action);
  void aprilTagDetectionCallback(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg);
  void aprilTagCollectionTimerCallback();
  void aprilTagActionTimeoutCallback();
  void finalizeAprilTagCollection();
  void addDetectionsToUniqueCollection(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr detections);
  std::string jsonFromAprilTagDetections(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr detections);

  // Helper function to transform poses to target frame
  bool transformPoseToTargetFrame(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out);
};

}  // namespace mission_client
}  // namespace isaac_ros

#endif  // VDA5050_ACTION_HANDLER_PLUGINS__APRILTAG_HANDLER_HPP_
