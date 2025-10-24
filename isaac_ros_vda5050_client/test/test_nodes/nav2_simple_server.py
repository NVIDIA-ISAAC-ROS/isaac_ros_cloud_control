#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import time

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class Nav2SimpleServer(Node):
    """
    Simple Nav2 Server node that responds to NavigateToPose actions.

    The server provides feedback messages in each iteration by incrementing the position.x
    of the previous feedback message until the current position.x is greater than the goal
    message's position.x.

    ROS Parameters
    ----------
    start_x_pos: The x position of the first feedback message.
    x_pos_increment: The increment of x position in each feedback iteration.
    """

    def __init__(self):
        super().__init__('nav2_simple_server')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_x_pos', 0.0),
                ('x_pos_increment', 0.5)
            ]
        )
        self._action_server = ActionServer(self, NavigateThroughPoses,
                                           'navigate_through_poses',
                                           self.callback, goal_callback=self.goal_callback,
                                           cancel_callback=self.cancel_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._completed_poses = 0
        self.tf_timer = self.create_timer(0.1, self.tf_callback)
        self.start_pos = self.get_parameter('start_x_pos').value
        self.current_pos = self.start_pos
        self.reset_srv = self.create_service(Empty, 'reset', self.reset_callback)

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting robot position')
        self.current_pos = 0.0
        return Empty.Response()

    def tf_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_pos
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def callback(self, goal_handle):
        if len(goal_handle.request.poses) == 0:
            return GoalResponse.REJECT
        self.get_logger().info('Executing goal...')

        feedback_msg = NavigateThroughPoses.Feedback()
        feedback_msg.current_pose.pose.position.x = self.start_pos
        feedback_msg.number_of_poses_remaining = len(goal_handle.request.poses)
        self._completed_poses = 0

        while feedback_msg.number_of_poses_remaining > 0:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                return NavigateThroughPoses.Result()
            target_pos = goal_handle.request.poses[self._completed_poses].pose.position.x
            if self.current_pos == target_pos:
                self._completed_poses += 1
                feedback_msg.number_of_poses_remaining -= 1
            elif self.current_pos < target_pos:
                self.current_pos += self.get_parameter('x_pos_increment').value
            else:
                self.current_pos -= self.get_parameter('x_pos_increment').value
            feedback_msg.current_pose.pose.position.x = self.current_pos
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()

        result = NavigateThroughPoses.Result()
        return result

    def goal_callback(self, goal_request):
        self.get_logger().info('received a goal')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
            ) -> rclpy.action.CancelResponse:
        self.get_logger().info('cancel requested: [{goal_id}]'.format(
            goal_id=goal_handle.goal_id))
        return rclpy.action.CancelResponse.ACCEPT


def main():
    rclpy.init()
    nav2_simple_server = Nav2SimpleServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav2_simple_server)

    try:
        executor.spin()
    finally:
        nav2_simple_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
