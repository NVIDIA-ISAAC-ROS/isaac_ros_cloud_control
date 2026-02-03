# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for AprilTag detection only."""
    launch_args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for ROS nodes in this launch script'),

        # AprilTag specific arguments
        DeclareLaunchArgument(
            'apriltag_image_topic',
            default_value='/front_stereo_camera/left/image_rect_color',
            description='Image topic for AprilTag detection'
        ),
        DeclareLaunchArgument(
            'apriltag_camera_info_topic',
            default_value='/front_stereo_camera/left/camera_info',
            description='Camera info topic for AprilTag detection'
        ),
        DeclareLaunchArgument(
            'apriltag_detections_topic',
            default_value='tag_detections',
            description='Topic where AprilTag detections are published (will be namespaced)'
        ),
        DeclareLaunchArgument(
            'apriltag_size',
            default_value='0.3048',  # 12 inches in meters
            description='Size of AprilTags in meters'
        ),
        DeclareLaunchArgument(
            'max_apriltags',
            default_value='10',
            description='Maximum number of AprilTags to detect'
        ),
        DeclareLaunchArgument(
            'apriltag_family',
            default_value='tag36h11',
            description='AprilTag family to detect'
        ),
        DeclareLaunchArgument(
            'apriltag_tile_size',
            default_value='4',
            description='AprilTag tile size parameter'
        ),
    ]

    # Launch configurations
    namespace = LaunchConfiguration('namespace')

    # AprilTag detection node
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name=['apriltag_detector_', namespace],
        remappings=[
            ('image', LaunchConfiguration('apriltag_image_topic')),
            ('camera_info', LaunchConfiguration('apriltag_camera_info_topic')),
            ('tag_detections', [namespace, '/', LaunchConfiguration('apriltag_detections_topic')])
        ],
        parameters=[{
            'size': LaunchConfiguration('apriltag_size'),
            'max_tags': LaunchConfiguration('max_apriltags'),
            'tile_size': LaunchConfiguration('apriltag_tile_size'),
            'tag_family': LaunchConfiguration('apriltag_family'),
        }],
    )

    # Container for AprilTag node
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name=['apriltag_detection_container_', namespace],
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return LaunchDescription(
        launch_args + [
            apriltag_container,
        ]
    )
