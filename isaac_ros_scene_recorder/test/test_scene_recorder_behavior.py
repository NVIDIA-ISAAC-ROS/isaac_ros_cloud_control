# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Functional tests for scene-recorder process and action behavior."""

from types import SimpleNamespace
from unittest.mock import Mock

from isaac_ros_scene_recorder import isaac_ros_scene_recorder as recorder


def test_signal_process_signals_children_before_parent(monkeypatch):
    """Stopping a recording must signal the rosbag process tree, not only its parent."""
    calls = []
    child_one = SimpleNamespace(send_signal=lambda value: calls.append(('child-1', value)))
    child_two = SimpleNamespace(send_signal=lambda value: calls.append(('child-2', value)))
    parent = SimpleNamespace(
        children=lambda recursive: [child_one, child_two],
        send_signal=lambda value: calls.append(('parent', value)),
    )
    monkeypatch.setattr(recorder.psutil, 'Process', lambda pid: parent)

    recorder.signal_process(42, 'SIGINT')

    assert calls == [
        ('child-1', 'SIGINT'),
        ('child-2', 'SIGINT'),
        ('parent', 'SIGINT'),
    ]


def test_start_recording_launches_rosbag_and_arms_timeout(monkeypatch):
    """Starting a recording must retain the process ID and arm automatic shutdown."""
    process = SimpleNamespace(pid=1234)
    popen = Mock(return_value=process)
    timer = Mock()
    timer_factory = Mock(return_value=timer)
    monkeypatch.setattr(recorder.subprocess, 'Popen', popen)
    monkeypatch.setattr(recorder.threading, 'Timer', timer_factory)
    server = SimpleNamespace(
        recording=False,
        process_pid=None,
        recording_feedback='',
        time_out=15,
        stop_recording_timer=None,
        stop_recording=Mock(),
        get_logger=Mock(return_value=Mock()),
    )
    command = ['ros2', 'bag', 'record', '-o', '/tmp/example', '/camera']

    recorder.RecorderActionServer.start_recording(server, command)

    popen.assert_called_once_with(command)
    assert server.process_pid == 1234
    assert server.recording is True
    assert ' '.join(command) in server.recording_feedback
    timer_factory.assert_called_once_with(15, server.stop_recording)
    timer.start.assert_called_once_with()


def test_stop_recording_interrupts_process_and_cancels_timer(monkeypatch):
    """Manual shutdown must interrupt rosbag and cancel the pending timeout."""
    signal_process = Mock()
    monkeypatch.setattr(recorder, 'signal_process', signal_process)
    timer = Mock()
    timer.is_alive.return_value = True
    server = SimpleNamespace(
        recording=True,
        process_pid=1234,
        recording_feedback='',
        stop_recording_timer=timer,
        get_logger=Mock(return_value=Mock()),
    )

    recorder.RecorderActionServer.stop_recording(server)

    signal_process.assert_called_once_with(1234, recorder.subprocess.signal.SIGINT)
    assert server.process_pid is None
    assert server.recording is False
    timer.cancel.assert_called_once_with()
    assert server.stop_recording_timer is None


def test_start_action_builds_expected_rosbag_command(monkeypatch):
    """The action request path, topics, and timeout must reach the process launcher."""
    monkeypatch.setattr(recorder.os.path, 'exists', lambda path: False)
    start_recording = Mock()
    server = SimpleNamespace(
        ros_CLI=['ros2', 'bag', 'record', '--include-hidden-topics', '-o'],
        time_out=600,
        recording_feedback='recording started',
        start_recording=start_recording,
        get_logger=Mock(return_value=Mock()),
    )
    goal = SimpleNamespace(
        request=SimpleNamespace(path='/tmp/scene', topics=['/camera', '/tf'], time=20),
        succeed=Mock(),
    )

    result = recorder.RecorderActionServer.start_recording_action_callback(server, goal)

    start_recording.assert_called_once_with([
        'ros2', 'bag', 'record', '--include-hidden-topics', '-o',
        '/tmp/scene', '/camera', '/tf',
    ])
    assert server.time_out == 20
    assert result.success is True
    assert result.result_description == 'recording started'
    goal.succeed.assert_called_once_with()
