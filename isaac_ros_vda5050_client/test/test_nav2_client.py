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

import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from isaac_ros_mqtt_bridge.MqttBridgeUtils import convert_dict_keys
from isaac_ros_test import IsaacROSBaseTest
import launch_ros.actions

import pytest
import rclpy

from rosbridge_library.internal import message_conversion
from rosbridge_library.internal import ros_loader
from sensor_msgs.msg import BatteryState as ROSBatteryState
from std_srvs.srv import Empty
from vda5050_msgs.msg import (
    Action, ActionState, AGVState, BatteryState, InstantActions, Node, Order)

ORDER_TOPIC = 'orders'
INSTANT_ORDER_TOPIC = 'instant_orders'
ORDER_INFO_TOPIC = 'agv_state'
BATTERY_STATE_TOPIC = 'battery_state'
RESET_TOPIC = 'reset'
START_X_POS = 0.0
STARTUP_TIME = 3
TIMEOUT = 10


@pytest.mark.rostest
def generate_test_description():

    nav2_simple_server = launch_ros.actions.Node(
        name='nav2_server',
        package='isaac_ros_vda5050_client',
        namespace=Nav2ClientTest.generate_namespace(),
        executable='nav2_simple_server.py',
        parameters=[{
            'start_x_pos': START_X_POS
        }],
        output='screen'
    )

    nav2_client = launch_ros.actions.Node(
        name='nav2_client',
        package='isaac_ros_vda5050_client',
        namespace=Nav2ClientTest.generate_namespace(),
        executable='vda5050_client',
        parameters=[{
            'config_file': os.path.join(
                get_package_share_directory('isaac_ros_vda5050_client'), 'config', 'config.yaml'),
            'update_feedback_period': 0.3
        }],
        arguments=['--ros-args', '--log-level', 'isaac_ros_test.nav2_client:=debug'],
        remappings=[('client_commands', ORDER_TOPIC),
                    ('instant_actions_commands', INSTANT_ORDER_TOPIC)],
        output='screen'
    )

    return Nav2ClientTest.generate_test_description([
        nav2_simple_server, nav2_client])


class Nav2ClientTest(IsaacROSBaseTest):
    received_msgs = []

    def create_order_from_json(self, json_file_path):
        """
        Create an order from a JSON file.

        Parameters
        ----------
        json_file_path : str
            Path under isaac_ros_vda5050_client/test_orders/

        Returns
        -------
        Order
            The vda5050_msgs/Order message

        """
        base_path = get_package_share_directory('isaac_ros_vda5050_client')
        json_file_path_absolute = os.path.join(base_path, 'test_orders', json_file_path)
        with open(json_file_path_absolute, 'r') as file:
            order_data = json.load(file)
        converted_message_dict = convert_dict_keys(
            order_data, 'camel_to_snake')
        ros_msg = ros_loader.get_message_instance('vda5050_msgs/Order')
        message_conversion.populate_instance(
            converted_message_dict, ros_msg)
        return ros_msg

    def create_order(self, order_id: str, node_positions, node_actions):
        """
        Create order for VDA5050 client node.

        Parameters
        ----------
        order_id : str
            The order_id used for creating this order
        node_positions : List[tuple[float]]
            The positions that the robot should reach. The first element of the tuple
            should represent the x-coordinate, and the second element should represent
            the y-coordinate.
        node_actions : List[List[vda5050_msgs/Action]]
            The actions to be performed at each node.

        Returns
        -------
        Order
            The vda5050_msgs/Order message

        """
        order_msg = Order()
        order_msg.order_id = order_id
        nodes = []
        for i, node_pos in enumerate(node_positions):
            node = Node()
            node.node_id = str(i)
            node.node_position.x = node_pos[0]
            node.node_position.y = node_pos[1]
            node.node_position.map_id = 'map'
            node.actions = node_actions[i]
            nodes.append(node)
        order_msg.nodes = nodes
        return order_msg

    def create_agv_state(self, order_id, last_node_id, position_initialized,
                         position_x=0.0, position_y=0.0):
        """
        Create AGVState message.

        Parameters
        ----------
        order_id : str
            The order_id for the created AGVState
        last_node_id : str
            The last_node_id for the created AGVState
        position_initialized : bool
            The value of position_initialized in agv_position
        position_x : float
            The value of agv_position.x
        position_y : float
            The value of agv_position.y

        Returns
        -------
        AGVState
            The vda5050_msgs/AGVState message

        """
        agv_state = AGVState()
        agv_state.order_id = order_id
        agv_state.agv_position.x = position_x
        agv_state.agv_position.y = position_y
        agv_state.agv_position.position_initialized = position_initialized
        agv_state.last_node_id = last_node_id
        return agv_state

    def verify_results(self, received_messages, expected_last_msg):
        """
        Verify the messages received from the client node.

        Parameters
        ----------
        received_messages : List[AGVState]
            A list of AGVState messages from the client node.
        expected_last_msg : AGVState
            The expected last received message.

        """
        try:
            last_order_id = received_messages[ORDER_INFO_TOPIC][-1].order_id
            self.assertEqual(last_order_id, expected_last_msg.order_id, 'Order ID does not match')
            last_pos = received_messages[ORDER_INFO_TOPIC][-1].agv_position
            self.assertEqual(last_pos, expected_last_msg.agv_position, 'End pose does not match')
            self.assertEqual(received_messages[ORDER_INFO_TOPIC][-1].action_states,
                             expected_last_msg.action_states,
                             'Action state does not match')
            last_node_id = received_messages[ORDER_INFO_TOPIC][-1].last_node_id
            self.assertEqual(last_node_id, expected_last_msg.last_node_id,
                             'Last node ID does not match')
        except Exception as e:
            self.node.get_logger().info('last_order_state:')
            json_msg = message_conversion.extract_values(received_messages[ORDER_INFO_TOPIC][-1])
            self.node.get_logger().info(json.dumps(json_msg, indent=2, default=str))
            self.node.get_logger().info('expected_last_msg:')
            json_msg = message_conversion.extract_values(expected_last_msg)
            self.node.get_logger().info(json.dumps(json_msg, indent=2, default=str))
            raise e

    def create_battery_state(self, percentage, voltage, power_supply_status):
        """
        Create BatteryState message.

        Parameters
        ----------
        percentage : float
            Charge percentage on 0 to 1 range
        voltage : float
            Voltage in Volts
        power_supply_status : int
            The charging status as reported. Values defined below.

            POWER_SUPPLY_STATUS_UNKNOWN = 0
            POWER_SUPPLY_STATUS_CHARGING = 1
            POWER_SUPPLY_STATUS_DISCHARGING = 2
            POWER_SUPPLY_STATUS_NOT_CHARGING = 3
            POWER_SUPPLY_STATUS_FULL = 4

        Returns
        -------
        BatteryState
            The sensor_msgs/BatteryState message

        """
        battery_state = ROSBatteryState()
        battery_state.percentage = percentage
        battery_state.voltage = voltage
        battery_state.power_supply_status = power_supply_status
        return battery_state

    def verify_battery_state(self, received_messages, expected_battery_state):
        """
        Verify the battery state messages received from the client node.

        Parameters
        ----------
        received_messages : List[AGVState]
            A list of AGVState messages from the client node.
        expected_battery_state: AGVState
            The expected battery state message.

        """
        battery_state = received_messages[ORDER_INFO_TOPIC][-1].battery_state
        self.assertEqual(battery_state.battery_charge, expected_battery_state.battery_charge)
        self.assertEqual(battery_state.battery_voltage, expected_battery_state.battery_voltage)
        self.assertEqual(battery_state.charging, expected_battery_state.charging)

    def is_order_completed(self, received_messages, last_node_id):
        """Check if the order is completed."""
        if len(received_messages[ORDER_INFO_TOPIC]) == 0:
            return False
        has_unfinished_action = False
        has_running_action = False
        for action_state in received_messages[ORDER_INFO_TOPIC][-1].action_states:
            if action_state.action_status not in ['FAILED', 'FINISHED']:
                has_unfinished_action = True
            if action_state.action_status in ['INITIALIZING', 'RUNNING']:
                has_running_action = True

        # All nodes are reached and all actions are finished or failed
        if (len(received_messages[ORDER_INFO_TOPIC]) and
                received_messages[ORDER_INFO_TOPIC][-1].last_node_id ==
                last_node_id and not has_unfinished_action):
            self.node.get_logger().info(
                f'order {received_messages[ORDER_INFO_TOPIC][-1].order_id} completed')
            self.node.get_logger().info(
                'all nodes reached and all actions are finished or failed')

            return True
        # A fatal error occurred
        has_fatal_error = False
        for error in received_messages[ORDER_INFO_TOPIC][-1].errors:
            if error.error_level == 'FATAL':
                has_fatal_error = True
                break
        if has_fatal_error and not has_running_action:
            return True
        return False

    def is_action_running(self, received_messages, action_idx=0):
        if len(received_messages[ORDER_INFO_TOPIC]) > 0 and \
           len(received_messages[ORDER_INFO_TOPIC][-1].action_states) > action_idx:
            if received_messages[ORDER_INFO_TOPIC][-1].action_states[action_idx].action_status \
               != 'WAITING':
                return True
        return False

    def teleop_with_api(self, received_messages, order_pub, instant_order_pub, node_positions):
        """Test Teleop with StartTeleop/StopTeleop instant actions."""
        order_id = 'teleop_with_api'
        received_messages[ORDER_INFO_TOPIC].clear()
        node_actions = [[], []]
        teleop_order = self.create_order(order_id, node_positions, node_actions)
        # Test startTeleop instant action.
        start_teleop_instant_order = InstantActions()
        start_teleop_instant_order.instant_actions = [
            Action(action_type='startTeleop', action_id='start_teleop_0')]
        # Test stopTeleop instant action.
        stop_teleop_instant_order = InstantActions()
        stop_teleop_instant_order.instant_actions = [
            Action(action_type='stopTeleop', action_id='stop_teleop_0')]
        # Expected state.
        expected_teleop = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, node_positions[-1][0])
        expected_teleop.action_states = [
            ActionState(action_id='start_teleop_0', action_type='startTeleop',
                        action_status='FINISHED'),
            ActionState(action_id='stop_teleop_0', action_type='stopTeleop',
                        action_status='FINISHED')
        ]
        order_pub.publish(teleop_order)
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        instant_order_pub.publish(start_teleop_instant_order)
        end_time = time.time() + TIMEOUT
        i = 0
        while time.time() < end_time:
            # Simulate teleop
            if i == 3:
                instant_order_pub.publish(stop_teleop_instant_order)
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(received_messages, '1') and \
                    received_messages[ORDER_INFO_TOPIC][-1].action_states[-1].action_status \
                    == 'FINISHED':
                break
            i += 1
        self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                           'Appropriate output not received')
        self.verify_results(received_messages, expected_teleop)

    def teleop_with_mission_node(self, received_messages, order_pub,
                                 instant_order_pub, node_positions):
        """Test Teleop with pause order mission node."""
        order_id = 'teleop_with_mission_node'
        received_messages[ORDER_INFO_TOPIC].clear()
        pause_order_action = [Action(action_type='pause_order',
                              action_id='pause_order_1',
                              action_parameters=[])]
        node_actions = [[], pause_order_action]
        teleop_order = self.create_order(order_id, node_positions, node_actions)
        stop_teleop_instant_order = InstantActions()
        stop_teleop_instant_order.instant_actions = [
            Action(action_type='stopTeleop', action_id='stop_teleop')]
        # Expected state
        expected_teleop = self.create_agv_state(
            order_id, str(len(node_positions) - 1), True, node_positions[-1][0])
        expected_teleop.action_states = [
            ActionState(action_id='pause_order_1', action_type='pause_order',
                        action_status='FINISHED'),
            ActionState(action_id='stop_teleop', action_type='stopTeleop',
                        action_status='FINISHED')
        ]
        order_pub.publish(teleop_order)
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        end_time = time.time() + TIMEOUT
        stop_order_published = False
        while time.time() < end_time:
            if self.is_action_running(received_messages, 0) and not stop_order_published:
                stop_order_published = True
                instant_order_pub.publish(stop_teleop_instant_order)
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(received_messages, '1'):
                break
        # spin twice for the order to complete
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        rclpy.spin_once(self.node, timeout_sec=(0.1))
        self.assertGreater(len(received_messages[ORDER_INFO_TOPIC]), 0,
                           'Appropriate output not received')
        self.verify_results(received_messages, expected_teleop)

    def setUp(self):
        """Set up before each test."""
        super().setUp()
        self.received_messages = {}
        self.generate_namespace_lookup([ORDER_TOPIC, INSTANT_ORDER_TOPIC, ORDER_INFO_TOPIC,
                                        BATTERY_STATE_TOPIC, RESET_TOPIC])
        self.order_pub = self.node.create_publisher(
            Order, self.namespaces[ORDER_TOPIC], self.DEFAULT_QOS)
        self.subs = self.create_logging_subscribers(
            [(ORDER_INFO_TOPIC, AGVState)], self.received_messages,
            accept_multiple_messages=True)
        self.instant_order_pub = self.node.create_publisher(
            InstantActions, self.namespaces[INSTANT_ORDER_TOPIC], self.DEFAULT_QOS)
        self.battery_state_pub = self.node.create_publisher(
            ROSBatteryState, self.namespaces[BATTERY_STATE_TOPIC], self.DEFAULT_QOS)
        self.reset_client = self.node.create_client(Empty, self.namespaces[RESET_TOPIC])
        # Wait for the order_pub to be subscribed
        while self.order_pub.get_subscription_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=(0.5))
            self.node.get_logger().info('Waiting for subscription')

    def tearDown(self):
        """Tear down after each test."""
        self.node.destroy_subscription(self.subs)
        self.node.destroy_publisher(self.order_pub)
        self.node.destroy_publisher(self.instant_order_pub)
        self.node.destroy_publisher(self.battery_state_pub)
        self.node.destroy_client(self.reset_client)
        super().tearDown()

    def reset_test_context(self):
        self.received_messages[ORDER_INFO_TOPIC].clear()

    def simple_order_test(self):
        """
        Test simple order.

        Test a simple order with 3 nodes and actions at the first and last nodes.
        """
        self.reset_test_context()
        self.node.get_logger().info('\n======simple_order_test======')
        order = self.create_order_from_json('simple_order.json')
        order_id = order.order_id
        last_node_id = order.nodes[-1].node_id
        end_x = order.nodes[-1].node_position.x
        self.order_pub.publish(order)
        self.node.get_logger().info('order published')

        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        self.assertGreater(
            len(self.received_messages[ORDER_INFO_TOPIC]), 0, 'Appropriate output not received')
        expected_last_msg = self.create_agv_state(
            order_id, last_node_id, True, end_x)
        expected_last_msg.action_states.append(
            ActionState(action_id='0', action_type='example_action', action_status='FINISHED')
        )
        expected_last_msg.action_states.append(
            ActionState(action_id='1', action_type='example_action', action_status='FINISHED')
        )
        self.verify_results(self.received_messages, expected_last_msg)

    def order_with_fatal_error_test(self):
        """Test order with action that will fail."""
        self.reset_test_context()
        self.node.get_logger().info('\n======order_with_fatal_error_test======')
        order = self.create_order_from_json('order_with_action_failure.json')
        self.order_pub.publish(order)

        order_id = order.order_id
        last_node_id = order.nodes[0].node_id
        end_x = 0.0
        self.order_pub.publish(order)
        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        self.assertGreater(
            len(self.received_messages[ORDER_INFO_TOPIC]), 0, 'Appropriate output not received')
        expected_last_msg = self.create_agv_state(
            order_id, last_node_id, True, end_x)
        expected_last_msg.action_states.append(
            ActionState(action_id='0', action_type='example_action', action_status='FAILED')
        )
        expected_last_msg.action_states.append(
            ActionState(action_id='1', action_type='example_action', action_status='WAITING')
        )
        self.verify_results(self.received_messages, expected_last_msg)

    def teleop_test(self):
        self.reset_test_context()
        self.node.get_logger().info('\n======teleop_test======')
        self.teleop_with_mission_node(
            self.received_messages,
            self.order_pub,
            self.instant_order_pub,
            [(2.0, 0.0), (1.0, 0.0)])

        # Test Teleop with API calls
        self.teleop_with_api(
            self.received_messages,
            self.order_pub, self.instant_order_pub,
            [(1.0, 0.0), (0.0, 0.0)])

    def cancel_order_test(self):
        """Test cancelOrder instant action."""
        self.reset_test_context()
        self.node.get_logger().info('\n======cancel_order_test======')
        order = self.create_order_from_json('simple_order.json')
        order_id = order.order_id
        last_node_id = order.nodes[0].node_id
        cancel_order_instant_order = InstantActions()
        cancel_order_instant_order.instant_actions = [
            Action(action_type='cancelOrder', action_id='cancel_order')]
        self.order_pub.publish(order)

        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            # Wait until the robot starts moving
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0 and \
                    self.received_messages[ORDER_INFO_TOPIC][-1].order_id == order.order_id and \
                    self.received_messages[ORDER_INFO_TOPIC][-1].agv_position.x > 0.0:
                self.instant_order_pub.publish(cancel_order_instant_order)
                break
        # Test cancelOrder instant action.
        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            # Wait until the cancelOrder action is finished
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                state = self.received_messages[ORDER_INFO_TOPIC][-1].action_states[-1]
                if state.action_id == 'cancel_order' and state.action_status == 'FINISHED':
                    break
        self.assertGreater(len(self.received_messages[ORDER_INFO_TOPIC]), 0,
                           'Appropriate output not received')
        end_x = self.received_messages[ORDER_INFO_TOPIC][-1].agv_position.x
        expected_last_msg = self.create_agv_state(
            order_id, last_node_id, True, end_x)
        expected_last_msg.action_states = [
            ActionState(action_id='0', action_type='example_action', action_status='FINISHED'),
            ActionState(action_id='1', action_type='example_action', action_status='FAILED'),
            ActionState(
                action_id='cancel_order', action_type='cancelOrder', action_status='FINISHED')
        ]
        self.verify_results(self.received_messages, expected_last_msg)

    def battery_state_test(self):
        self.reset_test_context()
        self.node.get_logger().info('\n======battery_state_test======')
        battery_state_messages = [
            self.create_battery_state(0.25, 12.0, 1),
            self.create_battery_state(1.0, 5.0, 3)
        ]

        # Expected battery messages
        expected_battery_state = [
            BatteryState(battery_charge=25.0, battery_voltage=12.0, charging=True),
            BatteryState(battery_charge=100.0, battery_voltage=5.0, charging=False)
        ]
        self.battery_state_pub.publish(battery_state_messages[0])
        self.received_messages[ORDER_INFO_TOPIC].clear()
        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if (len(self.received_messages[ORDER_INFO_TOPIC]) > 1):
                break
        self.verify_battery_state(self.received_messages, expected_battery_state[0])

        self.received_messages[ORDER_INFO_TOPIC].clear()
        self.battery_state_pub.publish(battery_state_messages[1])
        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if (len(self.received_messages[ORDER_INFO_TOPIC]) > 1):
                break
        self.verify_battery_state(self.received_messages, expected_battery_state[1])

    def none_blocking_action_test(self):
        """Test order with none blocking action."""
        self.reset_test_context()
        self.node.get_logger().info('\n======none_blocking_action_test======')
        order = self.create_order_from_json('none_blocking_action.json')
        last_node_id = order.nodes[-1].node_id
        self.order_pub.publish(order)
        end_time = time.time() + TIMEOUT
        running_while_driving = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[0].action_status == 'RUNNING' and \
                        last_msg.agv_position.x < 2.0:
                    running_while_driving = True
                    break
        # Action 0 should be running.
        self.assertTrue(running_while_driving)

        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        self.assertEqual(
            self.received_messages[ORDER_INFO_TOPIC][-1].action_states[0].action_status,
            'FINISHED')

    def soft_blocking_action_test(self):
        """Test order with soft blocking action."""
        self.reset_test_context()
        self.node.get_logger().info('\n======soft_blocking_action_test======')
        order = self.create_order_from_json('soft_blocking_action.json')
        last_node_id = order.nodes[-1].node_id
        self.order_pub.publish(order)
        end_time = time.time() + TIMEOUT
        running_while_driving = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[0].action_status == 'RUNNING' and \
                        last_msg.driving is True:
                    running_while_driving = True
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        # There should be no action that is running while driving
        self.assertFalse(running_while_driving)

    def parallel_action_test(self):
        """
        Test order with parallel actions.

        The first node has 5 actions. The blocking type of the actions are:
        - SOFT
        - SOFT
        - HARD
        - NONE
        - NONE
        The client should execute the first 2 actions in parallel.
        The third action should be executed alone after the first 2 actions are finished.
        The last 2 actions should be executed in parallel after the third action is finished
        without blocking driving.

        """
        self.reset_test_context()
        self.node.get_logger().info('\n======parallel_action_test======')
        order = self.create_order_from_json('parallel_action.json')
        last_node_id = order.nodes[-1].node_id
        self.order_pub.publish(order)
        end_time = time.time() + TIMEOUT
        first_2_actions_running = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[0].action_status == 'RUNNING' and \
                        last_msg.action_states[1].action_status == 'RUNNING':
                    first_2_actions_running = True
                    break
        self.assertTrue(first_2_actions_running)

        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[2].action_status == 'RUNNING':
                    break
        self.assertEqual(last_msg.action_states[0].action_status, 'FINISHED')
        self.assertEqual(last_msg.action_states[1].action_status, 'FINISHED')
        self.assertEqual(last_msg.action_states[3].action_status, 'WAITING')
        self.assertEqual(last_msg.action_states[4].action_status, 'WAITING')

        end_time = time.time() + TIMEOUT
        last_2_actions_running_while_driving = False
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[3].action_status == 'RUNNING' and \
                        last_msg.action_states[4].action_status == 'RUNNING' and \
                        last_msg.driving is True:
                    last_2_actions_running_while_driving = True
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        self.assertTrue(last_2_actions_running_while_driving)

    def pause_order_test(self):
        """Test pauseOrder instant action."""
        self.reset_test_context()
        self.node.get_logger().info('\n======pause_order_test======')
        order = self.create_order_from_json('simple_order.json')
        order_id = order.order_id
        last_node_id = order.nodes[-1].node_id
        pause_order_instant_order = InstantActions()
        pause_order_instant_order.instant_actions = [
            Action(action_type='startPause', action_id='pause')]
        self.order_pub.publish(order)

        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[0].action_status == 'RUNNING':
                    self.instant_order_pub.publish(pause_order_instant_order)
                    break
        self.instant_order_pub.publish(pause_order_instant_order)
        end_time = time.time() + TIMEOUT
        # Wait until the pause action is finished
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if len(self.received_messages[ORDER_INFO_TOPIC]) > 0:
                last_msg = self.received_messages[ORDER_INFO_TOPIC][-1]
                if last_msg.order_id == order.order_id and \
                        last_msg.action_states[-1].action_status == 'FINISHED':
                    break

        # Verify the robot is not driving
        self.assertFalse(self.received_messages[ORDER_INFO_TOPIC][-1].driving)

        # Publish resume order instant action
        resume_order_instant_order = InstantActions()
        resume_order_instant_order.instant_actions = [
            Action(action_type='stopPause', action_id='resume')]
        self.instant_order_pub.publish(resume_order_instant_order)

        # Wait until the order is completed
        end_time = time.time() + TIMEOUT
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=(0.1))
            if self.is_order_completed(self.received_messages, last_node_id):
                break
        end_x = 0.0
        expected_last_msg = self.create_agv_state(
            order_id, last_node_id, True, end_x)
        expected_last_msg.action_states = [
            ActionState(action_id='0', action_type='example_action', action_status='FINISHED'),
            ActionState(action_id='1', action_type='example_action', action_status='FINISHED'),
            ActionState(action_id='pause', action_type='startPause', action_status='FINISHED'),
            ActionState(action_id='resume', action_type='stopPause', action_status='FINISHED')
        ]
        self.verify_results(self.received_messages, expected_last_msg)

    def test_nav2_client(self):
        try:
            self.simple_order_test()
            self.order_with_fatal_error_test()
            self.teleop_test()
            self.cancel_order_test()
            self.battery_state_test()
            self.none_blocking_action_test()
            self.soft_blocking_action_test()
            self.parallel_action_test()
            self.pause_order_test()
        finally:
            pass
