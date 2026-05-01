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

"""Proof-Of-Life test for the Isaac ROS MQTT Bridge using TCP."""
import os
import pathlib
import subprocess

import base_mqtt_bridge_test
from launch_ros.actions import Node

import pytest


@pytest.mark.rostest
def generate_test_description():
    subprocess.Popen(['mosquitto'])
    mqtt_ros_node = Node(
        name='mqtt_ros_node',
        package='isaac_ros_mqtt_bridge',
        namespace=MqttBridgeTcpTest.generate_namespace(),
        executable='mqtt_ros_bridge_node',
        parameters=[{
            'ros_publisher_type': 'std_msgs/String',
            'ros_subscriber_type': 'std_msgs/String'
        }],
        remappings=[('agv_state', base_mqtt_bridge_test.TO_ROS_TOPIC)],
        output='screen'
    )

    return MqttBridgeTcpTest.generate_test_description([mqtt_ros_node])


class MqttBridgeTcpTest(base_mqtt_bridge_test.BaseMqttBridgeTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    def test_ros_to_mqtt(self):
        self.base_test_ros_to_mqtt('tcp')

    def test_mqtt_to_ros(self):
        self.base_test_mqtt_to_ros('tcp')
