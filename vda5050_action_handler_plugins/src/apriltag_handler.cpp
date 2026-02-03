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
#include <iomanip>
#include <algorithm>

#include "vda5050_action_handler_plugins/apriltag_handler.hpp"
#include "isaac_ros_vda5050_client/vda5050_client_node.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

namespace isaac_ros
{
namespace mission_client
{

namespace
{
constexpr char kGetAprilTagsAction[] = "get_apriltags";
constexpr double kDefaultDetectionBufferTime = 1.0;  // Time to collect detections
constexpr double kDefaultActionTimeout = 5.0;        // Total action timeout
constexpr double kDefaultTransformTimeout = 1.0;
}  // namespace

void AprilTagHandler::Initialize(
  Vda5050ClientNode * client_node,
  const YAML::Node & config)
{
  RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"), "Initializing AprilTagHandler");
  client_node_ = client_node;

  // Read configuration parameters
  apriltag_detections_topic_ =
    config["apriltag_detections_topic"].as<std::string>("tag_detections");
  apriltag_detection_buffer_time_ =
    config["apriltag_detection_buffer_time"].as<double>(kDefaultDetectionBufferTime);
  apriltag_action_timeout_ = config["apriltag_action_timeout"].as<double>(kDefaultActionTimeout);
  apriltag_target_frame_ = config["apriltag_target_frame"].as<std::string>("");
  apriltag_transform_mode_ = config["apriltag_transform_mode"].as<std::string>("static");
  apriltag_transform_timeout_ =
    config["apriltag_transform_timeout"].as<double>(kDefaultTransformTimeout);
  apriltag_collection_strategy_ =
    config["apriltag_collection_strategy"].as<std::string>("most_recent");
  verbose_ = config["verbose"].as<bool>(false);

  // Initialize TF2 for coordinate transformations
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(client_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create AprilTag detection subscription
  apriltag_detections_sub_ = client_node_->create_subscription<
    isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
    apriltag_detections_topic_,
    rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&AprilTagHandler::aprilTagDetectionCallback, this, std::placeholders::_1));

  last_apriltag_detection_time_ = client_node_->now();

  RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
    "AprilTag handler initialized with topic: %s, buffer_time: %.1fs, action_timeout: %.1fs",
    apriltag_detections_topic_.c_str(), apriltag_detection_buffer_time_, apriltag_action_timeout_);
}

void AprilTagHandler::Execute(const vda5050_msgs::msg::Action & vda5050_action)
{
  RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"), "Handling action: %s",
    vda5050_action.action_type.c_str());

  if (vda5050_action.action_type == kGetAprilTagsAction) {
    executeGetAprilTags(vda5050_action);
  }
}

void AprilTagHandler::executeGetAprilTags(const vda5050_msgs::msg::Action & vda5050_action)
{
  client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);

  RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
    "Starting AprilTag collection buffer (%.1fs) with action timeout (%.1fs) - will collect all unique tag IDs",
    apriltag_detection_buffer_time_, apriltag_action_timeout_);

  {
    std::lock_guard<std::mutex> lock(apriltag_collection_mutex_);

    // Initialize collection state
    apriltag_collection_state_.is_collecting = true;
    apriltag_collection_state_.action_id = vda5050_action.action_id;
    apriltag_collection_state_.collection_start_time = client_node_->now();
    apriltag_collection_state_.action_start_time = client_node_->now();

    // Clear the unique tags map for this collection
    unique_apriltag_detections_.clear();

    // Check if we already have recent good detections to seed the collection
    if (latest_apriltag_detections_ &&
      (client_node_->now() - last_apriltag_detection_time_).seconds() <
      apriltag_detection_buffer_time_ &&
      latest_apriltag_detections_->detections.size() > 0)
    {

      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
            "Seeding collection with %zu cached detections",
        latest_apriltag_detections_->detections.size());

      // Add cached detections to unique collection
      addDetectionsToUniqueCollection(latest_apriltag_detections_);
    }

    RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
          "Collection buffer started - waiting for detections...");

    // Start collection timer (checks for buffer completion every 100ms)
    apriltag_collection_state_.collection_timer = client_node_->create_wall_timer(
      std::chrono::milliseconds(100),  // Check every 100ms
      std::bind(&AprilTagHandler::aprilTagCollectionTimerCallback, this));

    // Start action timeout timer (safety net - should never be reached in normal operation)
    apriltag_collection_state_.action_timeout_timer = client_node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(apriltag_action_timeout_ * 1000)),
      std::bind(&AprilTagHandler::aprilTagActionTimeoutCallback, this));
  }
}

void AprilTagHandler::aprilTagDetectionCallback(
  const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
{
  /*
   * Cache latest AprilTag detections with timestamp
   * Only process and cache when we actually have detections (filter out empty arrays)
   * Also update collection window state if actively collecting
   */

  // Filter out empty detections - only process when we have actual tags
  if (msg->detections.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"), "Ignoring empty AprilTag detection array");
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"), "Processing AprilTag detection with %zu tags",
        msg->detections.size());

  // Always update the main cache when we have real detections
  {
    std::lock_guard<std::mutex> lock(apriltag_detections_mutex_);
    latest_apriltag_detections_ = msg;
    last_apriltag_detection_time_ = client_node_->now();
  }

  // Debug logging for AprilTag detections (only when verbose mode is enabled AND actively collecting)
  bool is_actively_collecting = false;
  {
    std::lock_guard<std::mutex> collection_lock(apriltag_collection_mutex_);
    is_actively_collecting = apriltag_collection_state_.is_collecting;
  }

  if (verbose_ && is_actively_collecting) {
    RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"), "AprilTag Detection: Received %zu tags",
          msg->detections.size());

    for (size_t i = 0; i < msg->detections.size(); i++) {
      const auto & detection = msg->detections[i];
      const auto & pose = detection.pose.pose.pose;

      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
                  "  Tag[%zu]: ID=%d, Family=%s, Center=(%.1f,%.1f)px, "
                  "Pose=[x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f], "
                  "Frame=%s",
                  i, detection.id, detection.family.c_str(),
                  detection.center.x, detection.center.y,
                  pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w,
                  detection.pose.header.frame_id.c_str());
    }
  }

  // Update collection buffer if actively collecting
  {
    std::lock_guard<std::mutex> collection_lock(apriltag_collection_mutex_);

    if (apriltag_collection_state_.is_collecting) {
      RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                   "Collection active: adding %zu detections to buffer",
                   msg->detections.size());

      // Add detections to unique collection (by tag ID)
      addDetectionsToUniqueCollection(msg);

      RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                   "Collection now contains %zu unique tags",
                   unique_apriltag_detections_.size());
    }
  }
}

void AprilTagHandler::aprilTagCollectionTimerCallback()
{
  std::lock_guard<std::mutex> lock(apriltag_collection_mutex_);

  if (!apriltag_collection_state_.is_collecting) {
    return;  // Collection already finished
  }

  auto buffer_elapsed_time = (client_node_->now() -
    apriltag_collection_state_.collection_start_time).seconds();

  // Check if collection buffer period has expired
  if (buffer_elapsed_time >= apriltag_detection_buffer_time_) {
    if (unique_apriltag_detections_.size() > 0) {
      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
        "AprilTag collection buffer complete (%.1fs) - found %zu unique tags",
        buffer_elapsed_time, unique_apriltag_detections_.size());
    } else {
      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
        "AprilTag collection buffer complete (%.1fs) - no detections found, returning empty",
        buffer_elapsed_time);
    }
    finalizeAprilTagCollection();
    return;
  }

  // Continue buffering...
  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
    "AprilTag collection buffering... (%.1fs elapsed, %zu unique tags so far)",
    buffer_elapsed_time, unique_apriltag_detections_.size());
}

void AprilTagHandler::aprilTagActionTimeoutCallback()
{
  std::lock_guard<std::mutex> lock(apriltag_collection_mutex_);

  if (!apriltag_collection_state_.is_collecting) {
    return;  // Collection already finished
  }

  auto total_elapsed_time = (client_node_->now() -
    apriltag_collection_state_.action_start_time).seconds();

  RCLCPP_ERROR(rclcpp::get_logger("AprilTagHandler"),
    "AprilTag action safety timeout reached (%.1fs) - something went wrong, forcing completion",
    total_elapsed_time);

  // Force finalization with timeout
  finalizeAprilTagCollection();
}

void AprilTagHandler::addDetectionsToUniqueCollection(
  const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr detections)
{
  // Add detections to unique collection map (by tag ID)
  for (const auto & detection : detections->detections) {
    // Use tag ID as key - this will automatically handle duplicates
    // Most recent detection for each ID is kept
    unique_apriltag_detections_[detection.id] = detection;

    RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
      "Added/updated tag ID %d in unique collection", detection.id);
  }
}

void AprilTagHandler::finalizeAprilTagCollection()
{
  if (!apriltag_collection_state_.is_collecting) {
    RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"), "Collection already finalized, skipping");
    return;  // Already finalized
  }

  std::string json_result = "[]";
  size_t unique_tag_count = unique_apriltag_detections_.size();

  // We need to create a dummy action with the ID to update its state
  vda5050_msgs::msg::Action current_action;
  current_action.action_id = apriltag_collection_state_.action_id;
  current_action.action_type = kGetAprilTagsAction;

  try {
    if (unique_tag_count > 0) {
      // Create detection array from unique collection
      auto unique_detections = std::make_shared<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>();
      unique_detections->header.stamp = client_node_->now();

      // Use the frame_id from the first detection (they should all be the same camera frame)
      std::string collection_frame_id = "";
      if (!unique_apriltag_detections_.empty()) {
        const auto & first_detection = unique_apriltag_detections_.begin()->second;
        collection_frame_id = first_detection.pose.header.frame_id;

        // If individual detection frame_id is empty, use the original message header
        if (collection_frame_id.empty() && latest_apriltag_detections_) {
          collection_frame_id = latest_apriltag_detections_->header.frame_id;
          RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
            "Using original message header frame_id: '%s'", collection_frame_id.c_str());
        }
      }

      unique_detections->header.frame_id = collection_frame_id;

      // Add all unique detections to array
      for (const auto & [tag_id, detection] : unique_apriltag_detections_) {
        unique_detections->detections.push_back(detection);
      }

      json_result = jsonFromAprilTagDetections(unique_detections);

      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
        "AprilTag collection completed: found %zu unique tags", unique_tag_count);

      // Log which tag IDs were found
      std::string tag_ids = "";
      for (const auto & [tag_id, detection] : unique_apriltag_detections_) {
        if (!tag_ids.empty()) {tag_ids += ", ";}
        tag_ids += std::to_string(tag_id);
      }
      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"), "Tag IDs: %s", tag_ids.c_str());

    } else {
      RCLCPP_WARN(rclcpp::get_logger("AprilTagHandler"),
        "AprilTag collection completed: no unique tags found during %.1fs buffer",
        apriltag_detection_buffer_time_);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("AprilTagHandler"),
      "Exception during AprilTag JSON generation: %s", ex.what());
    json_result = "[]";  // Fallback to empty array

    // Set action state to FAILED if there is exception
    client_node_->UpdateActionState(current_action,
      vda5050_msgs::msg::ActionState::FAILED, "Exception during JSON generation");
    return;
  }

  // Clean up collection state safely
  apriltag_collection_state_.is_collecting = false;
  unique_apriltag_detections_.clear();  // Clear unique collection

  if (apriltag_collection_state_.collection_timer) {
    apriltag_collection_state_.collection_timer->cancel();
    apriltag_collection_state_.collection_timer = nullptr;
  }

  if (apriltag_collection_state_.action_timeout_timer) {
    apriltag_collection_state_.action_timeout_timer->cancel();
    apriltag_collection_state_.action_timeout_timer = nullptr;
  }

  // Log final JSON result if verbose
  if (verbose_) {
    RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
      "Final AprilTag JSON result: %s", json_result.c_str());
  }

  // Update action state with results
  client_node_->UpdateActionState(current_action,
    vda5050_msgs::msg::ActionState::FINISHED, json_result);
}

std::string AprilTagHandler::jsonFromAprilTagDetections(
  const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr detections)
{
  /*
   * Convert AprilTag detections to JSON format
   * Include: tag_id, family, size, pose (position + orientation), frame_id
   * If apriltag_target_frame is specified, transforms poses to that frame
   * Otherwise, uses original camera frame poses
   * This matches the format expected by mission control
   */

  // DEBUG: Log transformation configuration
  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
               "=== AprilTag JSON Generation Debug ===");
  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
               "Target frame: '%s' (empty = no transform)",
               apriltag_target_frame_.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
               "Transform mode: '%s'",
               apriltag_transform_mode_.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
               "Processing %zu detections",
               detections->detections.size());

  std::ostringstream json_result;
  json_result << std::fixed << std::setprecision(3);
  json_result << "[\n";

  bool first_valid_detection = true;  // Track first valid detection for comma handling

  for (size_t i = 0; i < detections->detections.size(); i++) {
    const auto & detection = detections->detections[i];

    // DEBUG: Log original pose
    RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                 "Tag %d original pose: [x=%.3f, y=%.3f, z=%.3f] in frame '%s' (msg header: '%s')",
                 detection.id,
                 detection.pose.pose.pose.position.x,
                 detection.pose.pose.pose.position.y,
                 detection.pose.pose.pose.position.z,
                 detection.pose.header.frame_id.c_str(),
                 detections->header.frame_id.c_str());

    // Fix frame_id: use message header if individual detection frame_id is empty
    std::string source_frame_id = detection.pose.header.frame_id;
    if (source_frame_id.empty()) {
      source_frame_id = detections->header.frame_id;
      RCLCPP_WARN(rclcpp::get_logger("AprilTagHandler"),
                  "AprilTag %d has empty frame_id, using message header frame_id: '%s'",
                  detection.id, source_frame_id.c_str());
    }

    // Validate frame_id after fixing
    if (source_frame_id.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("AprilTagHandler"),
                   "Skipping AprilTag %d: both detection and message frame_id are empty",
                   detection.id);
      continue;
    }

    // Check if transformation is requested
    geometry_msgs::msg::Pose pose;
    std::string frame_id;

    if (!apriltag_target_frame_.empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                   "Attempting transformation from '%s' to '%s'",
                   source_frame_id.c_str(),
                   apriltag_target_frame_.c_str());

      // Transform pose from source frame to target frame
      geometry_msgs::msg::PoseStamped pose_in_source_frame;
      pose_in_source_frame.header = detection.pose.header;
      pose_in_source_frame.header.frame_id = source_frame_id;  // Use fixed frame_id
      pose_in_source_frame.pose = detection.pose.pose.pose;

      geometry_msgs::msg::PoseStamped pose_in_target_frame;
      bool transform_success = transformPoseToTargetFrame(pose_in_source_frame,
            pose_in_target_frame);

      if (!transform_success) {
        RCLCPP_WARN(rclcpp::get_logger("AprilTagHandler"),
                    "Skipping AprilTag %d due to transform failure from '%s' to '%s'",
                    detection.id, source_frame_id.c_str(),
                    apriltag_target_frame_.c_str());
        continue;  // Skip this detection
      }

      // Use transformed pose
      pose = pose_in_target_frame.pose;
      frame_id = apriltag_target_frame_;

      // DEBUG: Log transformed pose
      RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                   "Tag %d transformed pose: [x=%.3f, y=%.3f, z=%.3f] in frame '%s'",
                   detection.id,
                   pose.position.x, pose.position.y, pose.position.z,
                   frame_id.c_str());
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
                   "No transformation requested - using original pose");
      // Use original pose without transformation
      pose = detection.pose.pose.pose;
      frame_id = source_frame_id;  // Use fixed frame_id
    }

    // Add comma for subsequent valid detections
    if (!first_valid_detection) {
      json_result << ",\n";
    }
    first_valid_detection = false;

    json_result << "  {\n"
                << "    \"tag_id\": " << detection.id << ",\n"
                << "    \"family\": \"" << detection.family << "\",\n"
                << "    \"center\": {\n"
                << "      \"x\": " << detection.center.x << ",\n"
                << "      \"y\": " << detection.center.y << "\n"
                << "    },\n"
                << "    \"pose\": {\n"
                << "      \"position\": {\n"
                << "        \"x\": " << pose.position.x << ",\n"
                << "        \"y\": " << pose.position.y << ",\n"
                << "        \"z\": " << pose.position.z << "\n"
                << "      },\n"
                << "      \"orientation\": {\n"
                << "        \"x\": " << pose.orientation.x << ",\n"
                << "        \"y\": " << pose.orientation.y << ",\n"
                << "        \"z\": " << pose.orientation.z << ",\n"
                << "        \"w\": " << pose.orientation.w << "\n"
                << "      }\n"
                << "    },\n"
                << "    \"frame_id\": \"" << frame_id << "\",\n"
                << "    \"timestamp\": " << detection.pose.header.stamp.sec << "."
                << std::setfill('0') << std::setw(9)
                << detection.pose.header.stamp.nanosec << "\n"
                << "  }";
  }

  json_result << "\n]";

  RCLCPP_DEBUG(rclcpp::get_logger("AprilTagHandler"),
               "=== End AprilTag JSON Debug ===");

  return json_result.str();
}

bool AprilTagHandler::transformPoseToTargetFrame(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  /*
   * Transform a pose from its original frame to the configured target frame using TF2
   * Uses either latest available transform (for static) or timestamped (for dynamic)
   * Returns true if transformation was successful, false otherwise
   */
  try {
    // Get the transform from source frame to target frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    if (apriltag_transform_mode_ == "static") {
      // Use latest available transform (works with /tf_static)
      transform_stamped = tf_buffer_->lookupTransform(
          apriltag_target_frame_,
          pose_in.header.frame_id,
          tf2::TimePointZero);
    } else {
      // Use exact timestamp (requires /tf if dynamic)
      transform_stamped = tf_buffer_->lookupTransform(
          apriltag_target_frame_,
          pose_in.header.frame_id,
          pose_in.header.stamp,
          rclcpp::Duration::from_seconds(apriltag_transform_timeout_));
    }

    // Apply the transformation
    pose_out.header.frame_id = apriltag_target_frame_;
    pose_out.header.stamp = pose_in.header.stamp;
    tf2::doTransform(pose_in, pose_out, transform_stamped);

    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger("AprilTagHandler"),
                  "Successfully transformed pose from frame '%s' to '%s'",
                  pose_in.header.frame_id.c_str(),
                  apriltag_target_frame_.c_str());
    }

    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("AprilTagHandler"),
                "TF2 transform exception when transforming from '%s' to '%s': %s",
                pose_in.header.frame_id.c_str(),
                apriltag_target_frame_.c_str(), ex.what());
    return false;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("AprilTagHandler"),
                "Exception during pose transformation: %s", ex.what());
    return false;
  }
}

}  // namespace mission_client
}  // namespace isaac_ros

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(isaac_ros::mission_client::AprilTagHandler,
  isaac_ros::mission_client::Vda5050ActionHandlerBase)
