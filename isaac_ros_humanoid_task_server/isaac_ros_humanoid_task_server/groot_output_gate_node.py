#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""
Gate node that controls whether GR00T deploy pipeline output reaches the robot.

Subscribes to the remapped deploy pipeline output topics and republishes them
to the live control topics only when active.

A heartbeat watchdog guards against the controlling process dying with the gate
left open: while the gate is active the action server must publish to
``~/heartbeat`` faster than ``watchdog_timeout_sec``; if heartbeats stop, the
gate auto-closes so GR00T output cannot keep flowing to the robot unattended.
"""

import threading

from geometry_msgs.msg import Twist
from isaac_ros_deploy_interfaces.msg import JointCommandTrajectory
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import SetBool


class GrootOutputGateNode(Node):
    """Passthrough gate for GR00T deploy pipeline outputs."""

    def __init__(self) -> None:
        super().__init__('groot_output_gate_node')

        self.declare_parameter('joint_commands_trajectory_input_topic',
                               '/deploy/joint_commands_trajectory')
        self.declare_parameter('joint_commands_trajectory_output_topic',
                               '/joint_commands_trajectory')
        self.declare_parameter('cmd_vel_input_topic', '/deploy/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
        # Deadman: auto-close the gate if no heartbeat arrives within this window
        # while active. Set <= 0 to disable the watchdog entirely.
        self.declare_parameter('watchdog_timeout_sec', 1.0)

        jct_in = self.get_parameter('joint_commands_trajectory_input_topic').value
        jct_out = self.get_parameter('joint_commands_trajectory_output_topic').value
        cv_in = self.get_parameter('cmd_vel_input_topic').value
        cv_out = self.get_parameter('cmd_vel_output_topic').value
        self._watchdog_timeout_sec = self.get_parameter('watchdog_timeout_sec').value

        self._lock = threading.Lock()
        self._active = False
        self._last_heartbeat = None

        self._jct_pub = self.create_publisher(JointCommandTrajectory, jct_out, 10)
        self._cv_pub = self.create_publisher(Twist, cv_out, 10)

        self.create_subscription(JointCommandTrajectory, jct_in, self._on_joint_cmd_trajectory, 10)
        self.create_subscription(Twist, cv_in, self._on_cmd_vel, 10)

        self.create_service(SetBool, '~/set_active', self._handle_set_active)

        if self._watchdog_timeout_sec > 0.0:
            self._watchdog_timeout = Duration(seconds=self._watchdog_timeout_sec)
            self.create_subscription(Empty, '~/heartbeat', self._on_heartbeat, 10)
            # Tick well inside the timeout so a single missed beat does not trip it.
            self.create_timer(min(self._watchdog_timeout_sec / 2.0, 0.2), self._watchdog_tick)
        else:
            self._watchdog_timeout = None

        self.get_logger().info(
            f'GrootOutputGateNode ready (gate closed). '
            f'Joint cmd traj: {jct_in} -> {jct_out}  |  cmd_vel: {cv_in} -> {cv_out}  |  '
            f'watchdog: {"disabled" if self._watchdog_timeout is None else f"{self._watchdog_timeout_sec:.1f}s"}'  # noqa: E501
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_cmd_trajectory(self, msg: JointCommandTrajectory) -> None:
        with self._lock:
            active = self._active
        if active:
            self._jct_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            active = self._active
        if active:
            self._cv_pub.publish(msg)

    def _on_heartbeat(self, _msg: Empty) -> None:
        with self._lock:
            if self._active:
                self._last_heartbeat = self.get_clock().now()

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_tick(self) -> None:
        with self._lock:
            if not self._active or self._last_heartbeat is None:
                return
            stale = (self.get_clock().now() - self._last_heartbeat) > self._watchdog_timeout
        if stale:
            self.get_logger().warning(
                f'Watchdog: no heartbeat for >{self._watchdog_timeout_sec:.1f}s — closing gate.'
            )
            self._close_gate('watchdog timeout')

    # ------------------------------------------------------------------
    # Gate control
    # ------------------------------------------------------------------

    def _open_gate(self) -> None:
        with self._lock:
            was_active = self._active
            self._active = True
            self._last_heartbeat = self.get_clock().now()
        if not was_active:
            self.get_logger().info('Gate opened — GR00T outputs now flowing to robot.')

    def _close_gate(self, reason: str) -> None:
        with self._lock:
            was_active = self._active
            self._active = False
            self._last_heartbeat = None
        if was_active:
            # Stop base motion immediately by publishing a zero Twist. We
            # deliberately do NOT publish a joint-trajectory command here: the
            # robot holds its last commanded joint pose (freeze-in-place) rather
            # than going limp, which is the safe default for a standing humanoid.
            self._cv_pub.publish(Twist())
            self.get_logger().info(f'Gate closed ({reason}) — zero cmd_vel sent.')

    # ------------------------------------------------------------------
    # Service
    # ------------------------------------------------------------------

    def _handle_set_active(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data:
            self._open_gate()
        else:
            self._close_gate('service request')

        response.success = True
        response.message = 'active' if request.data else 'inactive'
        return response


def main() -> None:
    rclpy.init()
    node = GrootOutputGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
