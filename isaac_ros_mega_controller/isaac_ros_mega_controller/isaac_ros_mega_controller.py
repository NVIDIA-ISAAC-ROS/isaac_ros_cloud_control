# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from contextlib import nullcontext
from datetime import datetime
import json
import logging
import math
import os
import subprocess
import time
from urllib.parse import urlparse

from action_msgs.msg import GoalStatus
import boto3
from boto3.s3.transfer import TransferConfig
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header


class ROSBagRecorder:

    def __init__(self, output_directory):
        self.output_directory = output_directory

    def __enter__(self):
        MB = 1024 * 1024
        self.bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record',
             '-o', self.output_directory,
             '-a', '--use-sim-time',
             '-b', f'{500 * MB}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

    def __exit__(self, exc_type, exc_value, traceback):
        if hasattr(self, 'bag_process'):
            self.bag_process.terminate()
            self.bag_process.wait()
        if exc_type:
            print(f'Exception: {exc_value}')
        return False


class IsaacRosMegaController(Node):

    def __init__(self):
        super().__init__('isaac_ros_mega_controller')
        # Declare_parameters
        self.declare_parameter('waypoints', '')
        self.declare_parameter('nav2_check_interval', 10)
        self.cli = self.create_client(GetState, '/velocity_smoother/get_state')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Parse input file
        self.nav2_check_interval = \
            self.get_parameter('nav2_check_interval').get_parameter_value().integer_value
        waypoints = self.get_parameter('waypoints').get_parameter_value().string_value
        try:
            with open(waypoints, 'r') as fp:
                self.waypoints = json.load(fp)
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoints: {e}')
            exit(1)

        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service is available, starting loop.')

        self.s3_bucket_name = os.getenv('AWS_S3_BUCKET')
        self.ros_domain_id = os.getenv('ROS_DOMAIN_ID')
        self.enable_upload_to_s3 = False
        if self.s3_bucket_name:
            self.get_logger().info('Rosbag upload is enabled.')
            self.enable_upload_to_s3 = True
        current_time = datetime.now()
        self.timestamp_str = current_time.strftime('%Y-%m-%d_%H-%M-%S')
        self.bag_directory = f'/tmp/rosbag_{self.ros_domain_id}_{self.timestamp_str}'

    def human_readable_size(self, size):
        for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
            if size < 1024:
                return f'{size:.2f} {unit}'
            size /= 1024

    def upload_to_s3(self):
        self.get_logger().info(f'Uploading bag files to S3 bucket: {self.s3_bucket_name}...')
        s3_client = boto3.client('s3')
        parsed_uri = urlparse(self.s3_bucket_name)
        s3_bucket = parsed_uri.netloc
        parent_key = parsed_uri.path.strip('/')
        KB = 1024
        chunksize = 500*KB
        logging.basicConfig(level=logging.INFO)

        config = TransferConfig(multipart_threshold=chunksize, multipart_chunksize=chunksize)
        for root, dirs, files in os.walk(self.bag_directory):
            self.get_logger().info(f'Uploading {files}')
            for file in files:
                file_path = os.path.join(root, file)
                file_size = os.path.getsize(file_path)
                file_size = self.human_readable_size(file_size)
                s3_key = os.path.relpath(file_path, self.bag_directory)
                s3_key = f'{parent_key}/rosbag_{self.ros_domain_id}_{self.timestamp_str}/{s3_key}'
                try:
                    self.get_logger().info(
                        f'Uploading {file_path}({file_size}) to S3 as {s3_key}.')
                    s3_client.upload_file(file_path, s3_bucket, s3_key, Config=config)
                except Exception as e:
                    self.get_logger().error(f'Failed to upload {file_path}: {e}')

    def send_goal(self, pose):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available after waiting')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal...')
        return self._action_client.send_goal_async(goal_msg)

    def get_pose_stamped(self, px, py, qz, qw):
        pose = PoseStamped(
            pose=Pose(
                position=Point(x=px, y=py, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            ),
            header=Header(frame_id='map')
        )
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = IsaacRosMegaController()
    while True:
        request = GetState.Request()
        # Call the service and wait for the response
        future = node.cli.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)

        if future.result() is not None:
            if future.result().current_state.label == 'active':
                node.get_logger().info('Nav2 is up.')
                break
            else:
                node.get_logger().info('Waiting for Nav2.', throttle_duration_sec=120)
        else:
            node.get_logger().error('Failed to get the state.')
        # Sleep for a while before the next call
        time.sleep(node.nav2_check_interval)
    # Start recording
    with ROSBagRecorder(node.bag_directory) if node.enable_upload_to_s3 else nullcontext():
        poses = []
        for waypoint in node.waypoints:
            q_z = math.sin(waypoint['yaw'] / 2.0)
            q_w = math.cos(waypoint['yaw'] / 2.0)
            poses.append(
                node.get_pose_stamped(float(waypoint['x']), float(waypoint['y']), q_z, q_w))
        for pose in poses:
            send_goal_future = node.send_goal(pose)
            rclpy.spin_until_future_complete(node, send_goal_future)
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().error('Goal was rejected by the action server')
                break

            node.get_logger().info('Goal accepted, waiting for result...')

            # Now wait for the result of the action
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, get_result_future)
            result = get_result_future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                node.get_logger().error('Goal failed.')
                break
            else:
                node.get_logger().info(f'Goal {waypoint} is reached.')
    if node.enable_upload_to_s3:
        node.upload_to_s3()


if __name__ == '__main__':
    main()
