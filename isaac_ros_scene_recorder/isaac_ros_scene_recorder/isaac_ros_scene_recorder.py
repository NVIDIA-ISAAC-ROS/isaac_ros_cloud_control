#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Node to record a ROS bag with action.

Example call:
ROS 2 action send_goal recorder isaac_ros_scene_recorder_interface/action/StartRecording \
"{'path': 'data/bag_name', 'topics': '/msg1 /msg2', 'time': 30}"
ROS 2 action send_goal recorder isaac_ros_scene_recorder_interface/action/StopRecording \
"{}"

Remap the action when starting the node:
ROS 2 run isaac_ros_scene_recorder scene_recorder --ros-args --remap recorder:=new_action_name
"""
import os
import subprocess
import threading

from isaac_ros_scene_recorder_interface.action import StartRecording, StopRecording
import psutil
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


def signal_process(pid, signal_to_send):
    """Kill ROS bag recording process.

    Args:
        pid (str): process id
        signal_to_send: signal.SIGINT
    """
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        children.send_signal(signal_to_send)
    process.send_signal(signal_to_send)


class RecorderActionServer(Node):
    """ROS scene recorder action server."""

    def __init__(self):
        super().__init__('recorder_action_server')
        self._start_recording_action_server = ActionServer(
            self,
            StartRecording,
            'start_recording',
            self.start_recording_action_callback)
        self._stop_recording_action_server = ActionServer(
            self,
            StopRecording,
            'stop_recording',
            self.stop_recording_action_callback)
        self.recording = False
        self.process_pid = None
        self.recording_feedback = ''
        self.ros_CLI = ['ros2', 'bag', 'record', '--include-hidden-topics', '-o', ]
        self.time_out = 600
        self.stop_recording_timer = None

    def start_recording(self, rosbag_command):
        if self.recording:
            self.get_logger().info('ROS scene recorder has already started.')
            return
        process = subprocess.Popen(rosbag_command)
        self.process_pid = process.pid
        self.recording = True
        self.recording_feedback = 'Recording ROS scene recorder with command ' \
            + ' '.join(rosbag_command)
        self.get_logger().info(self.recording_feedback)
        self.stop_recording_timer = threading.Timer(self.time_out, self.stop_recording)
        self.stop_recording_timer.start()

    def stop_recording(self):
        if self.process_pid is not None:
            signal_process(self.process_pid, subprocess.signal.SIGINT)
            self.recording_feedback = 'Stopped ROS scene recorder'
            self.get_logger().info(self.recording_feedback)
            self.process_pid = None
        self.recording = False
        if self.stop_recording_timer and self.stop_recording_timer.is_alive():
            self.stop_recording_timer.cancel()

    def start_recording_action_callback(self, goal_handle):
        self.get_logger().info('Executing ROS scene recorder service call ...')
        # Keys: action_type, action_id, path, topics, time_out
        result = StartRecording.Result()
        path = goal_handle.request.path
        if os.path.exists(path):
            self.get_logger().warn(f'Output folder {path} already exists.')
            goal_handle.succeed()
            result.success = False
            result.result_description = 'Output folder path already exists.'
            return result
        rosbag_command = self.ros_CLI[:] + [path]
        rosbag_command.extend(goal_handle.request.topics)
        self.time_out = int(goal_handle.request.time)
        self.start_recording(rosbag_command)
        goal_handle.succeed()
        result.success = True
        result.result_description = self.recording_feedback
        return result

    def stop_recording_action_callback(self, goal_handle):
        self.get_logger().info('Executing ROS scene recorder service call ...')
        result = StopRecording.Result()
        self.stop_recording()
        goal_handle.succeed()
        result.success = True
        result.result_description = self.recording_feedback
        return result


def main(args=None):
    rclpy.init(args=args)
    recorder_action_server = RecorderActionServer()
    rclpy.spin(recorder_action_server)


if __name__ == '__main__':
    main()
