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
ROS 2 action server that triggers GR00T manipulation tasks via the output gate.

Implements the HumanoidTask action interface. On goal accept it opens the
GrootOutputGateNode gate so GR00T deploy pipeline output flows to the robot,
and closes it again when the task ends (timeout / cancel / abort). While a task
runs it publishes a heartbeat to the gate so the gate's deadman watchdog knows
the controller is alive.

Runs under a MultiThreadedExecutor: the execute callback paces itself with a
blocking sleep on its own worker thread, while joint-state, cancel, and service
callbacks continue to be serviced on other threads.
"""

import json
import threading
import time
from typing import Optional

from isaac_ros_cloud_control_interface.action import HumanoidTask
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_srvs.srv import SetBool


class HumanoidTaskActionServer(Node):
    """Action server that gates GR00T output for cloud-commanded manipulation tasks."""

    def __init__(self) -> None:
        super().__init__('humanoid_task_action_server')

        self.declare_parameter('action_server_name', 'humanoid_task')
        self.declare_parameter('gate_service', 'groot_output_gate_node/set_active')
        self.declare_parameter('gate_heartbeat_topic', 'groot_output_gate_node/heartbeat')
        self.declare_parameter('feedback_rate_hz', 10.0)
        # Safety bound applied when a goal requests timeout <= 0. The action
        # interface documents "0 for no timeout", but leaving the gate open
        # forever on a physical humanoid is unsafe, so an unset timeout falls
        # back to this value. Set this <= 0 to honour a true no-timeout request.
        self.declare_parameter('default_timeout_s', 300.0)

        self._action_server_name: str = self.get_parameter('action_server_name').value
        self._gate_service_name: str = self.get_parameter('gate_service').value
        self._gate_heartbeat_topic: str = self.get_parameter('gate_heartbeat_topic').value
        self._feedback_rate_hz: float = self.get_parameter('feedback_rate_hz').value
        self._default_timeout_s: float = self.get_parameter('default_timeout_s').value

        self._lock = threading.Lock()
        self._current_goal_handle = None
        self._is_executing: bool = False
        self._start_time = None
        self._current_joint_state: Optional[JointState] = None

        self._gate_client = self.create_client(
            SetBool, self._gate_service_name,
            callback_group=ReentrantCallbackGroup(),
        )
        self._heartbeat_pub = self.create_publisher(Empty, self._gate_heartbeat_topic, 10)

        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

        self._action_server = ActionServer(
            self,
            HumanoidTask,
            self._action_server_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f'HumanoidTaskActionServer ready on "{self._action_server_name}". '
            f'Gate: {self._gate_service_name}'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            self._current_joint_state = msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _resolve_timeout(self, goal_timeout: float) -> float:
        """
        Resolve the effective timeout, applying the safety default.

        A goal_timeout > 0 is honoured as-is. A goal_timeout <= 0 means "unset";
        it falls back to default_timeout_s, and only yields an unbounded timeout
        if default_timeout_s is itself <= 0.
        """
        if goal_timeout > 0.0:
            return goal_timeout
        if self._default_timeout_s > 0.0:
            return self._default_timeout_s
        return float('inf')

    def _elapsed_since(self, start) -> float:
        return (self.get_clock().now() - start).nanoseconds / 1e9

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_cb(self, goal_request) -> int:
        self.get_logger().info(
            f'Received HumanoidTask goal — category: {goal_request.task_category}, '
            f'id: {goal_request.task_id}'
        )
        # Claim the executing slot atomically at accept time. Setting the flag
        # here (rather than in the execute callback) closes the race where a
        # second goal arrives before the first execute callback has started.
        with self._lock:
            if self._is_executing:
                self.get_logger().warning('Rejecting goal — another task is already executing.')
                return GoalResponse.REJECT
            self._is_executing = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> int:
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        timeout = self._resolve_timeout(goal.timeout)

        with self._lock:
            self._current_goal_handle = goal_handle
            self._start_time = self.get_clock().now()
            start_time = self._start_time

        self.get_logger().info(
            f'Executing task "{goal.task_id}" — instruction: "{goal.language_instruction}" '
            f'— timeout: {timeout:.1f}s'
        )

        # Hand control to GR00T. If the gate cannot be opened, fail the goal
        # rather than run a silent no-op the cloud would read as success.
        if not await self._set_gate(True):
            self._reset_state()
            feedback = HumanoidTask.Feedback()
            feedback.status = HumanoidTask.Feedback.FAILED
            feedback.current_instruction = goal.language_instruction
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            result = HumanoidTask.Result()
            result.did_succeed = False
            result.message = 'Failed to open GR00T output gate.'
            result.result_data = json.dumps({'gate_open_failed': True})
            result.execution_time = 0.0
            return result

        feedback = HumanoidTask.Feedback()
        feedback.status = HumanoidTask.Feedback.EXECUTING
        feedback.current_instruction = goal.language_instruction

        period_s = 1.0 / self._feedback_rate_hz if self._feedback_rate_hz > 0.0 else 0.1
        next_tick = time.time() + period_s
        canceled = False

        try:
            while True:
                if goal_handle.is_cancel_requested:
                    canceled = True
                    break

                elapsed = self._elapsed_since(start_time)
                if elapsed >= timeout:
                    break

                # Heartbeat keeps the gate's deadman watchdog satisfied.
                self._heartbeat_pub.publish(Empty())

                with self._lock:
                    js = self._current_joint_state
                feedback.current_execution_time = elapsed
                if js is not None:
                    feedback.current_joint_state = js
                goal_handle.publish_feedback(feedback)

                sleep_s = max(0.0, next_tick - time.time())
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                next_tick = max(next_tick + period_s, time.time())

        finally:
            await self._set_gate(False)
            with self._lock:
                elapsed = self._elapsed_since(start_time)
                js = self._current_joint_state
            self._reset_state()

        result = HumanoidTask.Result()
        result.execution_time = elapsed
        if js is not None:
            result.final_joint_state = js

        if canceled:
            feedback.status = HumanoidTask.Feedback.CANCELLED
            goal_handle.publish_feedback(feedback)
            result.did_succeed = False
            result.message = 'Task canceled.'
            result.result_data = json.dumps({'canceled': True})
            goal_handle.canceled()
            self.get_logger().info('Task canceled.')
        else:
            feedback.status = HumanoidTask.Feedback.COMPLETED
            goal_handle.publish_feedback(feedback)
            result.did_succeed = True
            result.message = f'Task completed after {elapsed:.1f}s.'
            result.result_data = json.dumps({'elapsed': elapsed})
            goal_handle.succeed()
            self.get_logger().info(f'Task "{goal.task_id}" succeeded after {elapsed:.1f}s.')

        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _reset_state(self) -> None:
        with self._lock:
            self._is_executing = False
            self._current_goal_handle = None
            self._start_time = None

    async def _set_gate(self, active: bool) -> bool:
        """Open/close the gate. Returns True on confirmed success."""
        if not self._gate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Gate service "{self._gate_service_name}" unavailable — '
                f'cannot {"open" if active else "close"} gate.'
            )
            return False
        req = SetBool.Request()
        req.data = active
        response = await self._gate_client.call_async(req)
        if response is None or not response.success:
            self.get_logger().error(
                f'Gate service failed to {"open" if active else "close"}: '
                f'{response.message if response is not None else "no response"}'
            )
            return False
        return True


def main() -> None:
    rclpy.init()
    node = HumanoidTaskActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
