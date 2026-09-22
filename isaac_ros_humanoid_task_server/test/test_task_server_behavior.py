# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Functional tests for the humanoid task server and GR00T output gate."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Twist
from isaac_ros_humanoid_task_server.groot_output_gate_node import GrootOutputGateNode
from isaac_ros_humanoid_task_server.humanoid_task_action_server import HumanoidTaskActionServer
from rclpy.action import GoalResponse


def test_resolve_timeout_applies_goal_and_safety_defaults():
    """A positive goal timeout wins; otherwise the configured safety policy applies."""
    server = SimpleNamespace(_default_timeout_s=300.0)

    assert HumanoidTaskActionServer._resolve_timeout(server, 12.5) == 12.5
    assert HumanoidTaskActionServer._resolve_timeout(server, 0.0) == 300.0

    server._default_timeout_s = 0.0
    assert HumanoidTaskActionServer._resolve_timeout(server, -1.0) == float('inf')


def test_goal_callback_claims_the_execution_slot_atomically():
    """A second cloud task must be rejected while the first task owns the gate."""
    server = SimpleNamespace(
        _is_executing=False,
        _lock=threading.Lock(),
        get_logger=Mock(return_value=Mock()),
    )
    request = SimpleNamespace(task_category='manipulation', task_id='pick')

    first = HumanoidTaskActionServer._goal_cb(server, request)
    second = HumanoidTaskActionServer._goal_cb(server, request)

    assert first == GoalResponse.ACCEPT
    assert second == GoalResponse.REJECT


def test_gate_forwards_velocity_only_while_active():
    """Commands received while the safety gate is closed must be discarded."""
    publisher = Mock()
    gate = SimpleNamespace(
        _active=False,
        _lock=threading.Lock(),
        _cv_pub=publisher,
    )
    command = Twist()
    command.linear.x = 0.75

    GrootOutputGateNode._on_cmd_vel(gate, command)
    publisher.publish.assert_not_called()

    gate._active = True
    GrootOutputGateNode._on_cmd_vel(gate, command)
    publisher.publish.assert_called_once_with(command)


def test_closing_gate_stops_base_and_resets_watchdog():
    """Closing an active gate must send zero velocity and clear heartbeat state."""
    publisher = Mock()
    logger = Mock()
    gate = SimpleNamespace(
        _active=True,
        _last_heartbeat=object(),
        _lock=threading.Lock(),
        _cv_pub=publisher,
        get_logger=Mock(return_value=logger),
    )

    GrootOutputGateNode._close_gate(gate, 'test')

    assert gate._active is False
    assert gate._last_heartbeat is None
    publisher.publish.assert_called_once()
    stop_command = publisher.publish.call_args.args[0]
    assert stop_command.linear.x == 0.0
    assert stop_command.angular.z == 0.0
