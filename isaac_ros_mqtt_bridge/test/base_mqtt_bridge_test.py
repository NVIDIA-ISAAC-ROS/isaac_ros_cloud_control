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

"""Base test class for the Isaac ROS MQTT Bridge."""
import json
import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest
import paho.mqtt.client as mqtt

import rclpy

from std_msgs.msg import String

FROM_MQTT_TOPIC = 'uagv/v2/RobotCompany/carter01/state'
TO_MQTT_TOPIC = 'uagv/v2/RobotCompany/carter01/order'
FROM_ROS_TOPIC = 'bridge_pub_topic'
TO_ROS_TOPIC = 'to_ros_topic'


class BaseMqttBridgeTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    def base_test_ros_to_mqtt(self, transport, port=1883):
        """
        Test for RosToMqtt Bridge node.

        This test creates a ROS 2 String publisher and an MQTT subscriber. The string publisher
        will publish to a topic that the RosToMqtt Bridge node subscribes to, and the bridge
        node will translate that into a JSON message and publish it to MQTT, which the MQTT
        subscriber will receive.
        """
        TIMEOUT = 10

        self.generate_namespace_lookup([TO_ROS_TOPIC])

        string_pub = self.node.create_publisher(
            String, self.namespaces[TO_ROS_TOPIC], self.DEFAULT_QOS)

        received_messages = []

        mqtt_receiver = mqtt.Client('mqtt_client', transport=transport)

        def on_mqtt_connect(client, userdata, flags, rc):
            print('Mqtt Connected')

        def on_mqtt_message(client, userdata, msg):
            received_messages.append(msg)

        mqtt_receiver.on_connect = on_mqtt_connect
        mqtt_receiver.on_message = on_mqtt_message

        mqtt_receiver.connect('localhost', port)
        mqtt_receiver.subscribe(FROM_MQTT_TOPIC)
        mqtt_receiver.loop_start()

        try:
            message_contents = 'Hello world'
            ros_string = String()
            ros_string.data = message_contents

            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                string_pub.publish(ros_string)
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if received_messages:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')
            if done:
                message = json.loads(
                    str(received_messages[0].payload, 'utf-8'))
                print(message)
                self.assertTrue(message['data'] == message_contents)

        finally:
            self.node.destroy_publisher(string_pub)
            mqtt_receiver.loop_stop()

    def base_test_mqtt_to_ros(self, transport, port=1883):
        """
        Test for MqttToRos Bridge node.

        This test creates a MQTT publisher and a ROS string subscriber. The MQTT publisher
        will publish to a topic that the MqttToRos Bridge node subscribes to, and the bridge
        node will translate that into the correct type of ROS message, populate it, and publish
        it to the ROS middleware, which the ROS subscriber will receive.
        """
        TIMEOUT = 10

        self.generate_namespace_lookup([FROM_ROS_TOPIC])

        received_messages = {}

        mqtt_transmitter = mqtt.Client('mqtt_client', transport=transport)

        def on_mqtt_connect(client, userdata, flags, rc):
            print('Mqtt Connected')

        mqtt_transmitter.on_connect = on_mqtt_connect
        mqtt_transmitter.connect('localhost', port)
        mqtt_transmitter.loop_start()

        subs = self.create_logging_subscribers(
            [(FROM_ROS_TOPIC, String)], received_messages)
        try:
            message_contents = 'Hello world'
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                mqtt_message = {'data': message_contents}
                mqtt_transmitter.publish(
                    TO_MQTT_TOPIC, json.dumps(mqtt_message))
                rclpy.spin_once(self.node, timeout_sec=(0.1))
                if received_messages:
                    done = True
                    break
            self.assertTrue(done, 'Appropriate output not received')
            if done:
                # Check if the node published a ROS message that is the same type
                # as the type given to the node as a parameter
                self.assertTrue(isinstance(
                    received_messages[FROM_ROS_TOPIC], String))
                # Check if the message data matches
                self.assertTrue(
                    received_messages[FROM_ROS_TOPIC].data == message_contents)

        finally:
            self.node.destroy_subscription(subs)
            mqtt_transmitter.loop_stop()
