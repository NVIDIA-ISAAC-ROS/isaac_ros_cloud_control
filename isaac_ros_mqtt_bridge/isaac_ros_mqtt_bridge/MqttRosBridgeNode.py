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
import socket
import ssl
import threading
import time

from isaac_ros_mqtt_bridge.MqttBridgeUtils import ConnectionMessage
from isaac_ros_mqtt_bridge.MqttBridgeUtils import convert_dict_keys
from isaac_ros_mqtt_bridge.MqttBridgeUtils import State
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal import ros_loader
from std_msgs.msg import String


class MqttRosBridgeNode(Node):
    """Unified bridge node for MQTT<->ROS VDA5050 topics."""

    def __init__(self, name='mqtt_ros_bridge_node'):
        """Construct the MqttRosBridgeNode."""
        super().__init__(name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('interface_name', 'uagv'),
                ('major_version', 'v2'),
                ('manufacturer', 'RobotCompany'),
                ('serial_number', 'carter01'),
                ('mqtt_host_name', 'localhost'),
                ('mqtt_port', 1883),
                ('mqtt_transport', 'tcp'),
                ('mqtt_ws_path', ''),
                ('mqtt_keep_alive', 60),
                ('mqtt_username', ''),
                ('mqtt_password', ''),
                ('mqtt_tls_enabled', False),
                ('mqtt_ca_cert', ''),
                ('mqtt_client_cert', ''),
                ('mqtt_client_key', ''),
                ('mqtt_tls_insecure', False),
                ('mqtt_tls_version', 'tlsv1.2'),
                ('mqtt_debug_logs', False),
                ('reconnect_period', 5),
                ('retry_forever', False),
                ('num_retries', 10),
                ('ros_queue_size', 10),
                ('convert_case', True),
                ('ros_subscriber_type', 'vda5050_msgs/AGVState'),
                ('ros_publisher_type', 'vda5050_msgs/Order')
            ]
        )

        # Initialize MQTT client
        client_id = self.get_parameter('serial_number').value
        self.get_logger().info(f'MQTT Client ID: {client_id}')
        self.get_logger().info(f'MQTT Host: {self.get_parameter("mqtt_host_name").value}')

        self.mqtt_client = mqtt.Client(
            client_id=client_id,
            protocol=mqtt.MQTTv311,
            transport=self.get_parameter('mqtt_transport').value)
        self._mqtt_loop_started = False
        self._mqtt_ready = False
        self._reconnect_lock = threading.Lock()
        self._reconnecting = False

        self._setup_mqtt_connection()
        if self.get_parameter('mqtt_tls_enabled').value:
            self._setup_tls()

        # Store common parameters
        self.manufacturer = self.get_parameter('manufacturer').value
        self.serial_number = self.get_parameter('serial_number').value
        self.interface_name = self.get_parameter('interface_name').value
        self.major_version = self.get_parameter('major_version').value
        self.mqtt_topic_prefix = (
            f'{self.interface_name}/{self.major_version}/{self.manufacturer}/{self.serial_number}'
        )

        # Set up MQTT callbacks
        self._setup_mqtt_callbacks()

        # Initialize ROS publishers
        self._setup_ros_publishers()

        # Initialize ROS subscribers
        self._setup_ros_subscribers()

        # Connect to MQTT broker
        self._connect_mqtt_broker()

    def _setup_tls(self):
        """Configure TLS settings for MQTT client."""
        try:
            ca_certs = self.get_parameter('mqtt_ca_cert').value
            cert_file = self.get_parameter('mqtt_client_cert').value
            key_file = self.get_parameter('mqtt_client_key').value

            self.get_logger().info('Setting up TLS with:')
            self.get_logger().info(f'CA Certificate: {ca_certs}')
            self.get_logger().info(f'Client Certificate: {cert_file}')
            self.get_logger().info(f'Client Key: {key_file}')

            tls_version_map = {
                'tlsv1.2': ssl.TLSVersion.TLSv1_2,
                'tlsv1.3': ssl.TLSVersion.TLSv1_3,
            }
            tls_version_str = self.get_parameter('mqtt_tls_version').value
            min_tls_version = tls_version_map.get(tls_version_str)
            if min_tls_version is None:
                self.get_logger().warning(
                    f'Unknown or unsupported mqtt_tls_version "{tls_version_str}"'
                    ', defaulting to TLS 1.2')
                min_tls_version = ssl.TLSVersion.TLSv1_2

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            ssl_context.minimum_version = min_tls_version
            ssl_context.load_cert_chain(certfile=cert_file, keyfile=key_file)
            ssl_context.load_verify_locations(ca_certs)

            if self.get_parameter('mqtt_tls_insecure').value:
                ssl_context.check_hostname = False
                ssl_context.verify_mode = ssl.CERT_NONE

            self.mqtt_client.tls_set_context(ssl_context)
            self.mqtt_client.tls_insecure_set(
                self.get_parameter('mqtt_tls_insecure').value)

        except Exception as e:
            self.get_logger().error(f'TLS setup failed: {str(e)}')
            raise

    def _connection_state_payload(self, state: State) -> str:
        """Build a VDA5050 connection state JSON payload."""
        msg = ConnectionMessage(self.manufacturer, self.serial_number, state)
        return str(msg)

    def _publish_connection_state(self, state: State) -> None:
        """Publish a VDA5050 connection state message to MQTT."""
        topic = f'{self.mqtt_topic_prefix}/connection'
        payload = self._connection_state_payload(state)
        self.get_logger().info(f'Publishing connection state {state.name} to {topic}')
        self.mqtt_client.publish(topic, payload, qos=1, retain=False)

    def _setup_mqtt_connection(self):
        """Configure MQTT connection parameters."""
        if self.get_parameter('mqtt_transport').value == 'websockets' and \
                self.get_parameter('mqtt_ws_path').value != '':
            self.mqtt_client.ws_set_options(path=self.get_parameter('mqtt_ws_path').value)

        if self.get_parameter('mqtt_username').value != '' and \
                self.get_parameter('mqtt_password').value != '':
            self.mqtt_client.username_pw_set(
                username=self.get_parameter('mqtt_username').value,
                password=self.get_parameter('mqtt_password').value)

    def _setup_mqtt_callbacks(self):
        """Set up MQTT client callbacks."""
        def on_connect(client, userdata, flags, rc):
            self._on_mqtt_connect(client, userdata, flags, rc)

        def on_disconnect(client, userdata, rc):
            self._mqtt_ready = False
            self.get_logger().error(
                f'MQTT Disconnected - Result Code: {rc} '
                f'({mqtt.error_string(rc)})')
            if rc == 0:
                self.get_logger().info('Clean disconnect')
            else:
                self.get_logger().error('Unexpected disconnect')

                if not rclpy.ok():
                    return
                with self._reconnect_lock:
                    if self._reconnecting:
                        self.get_logger().info(
                            'Reconnection already in progress, skipping.')
                        return
                    self._reconnecting = True
                self.get_logger().info('Scheduling reconnection on a new thread...')
                reconnect_thread = threading.Thread(
                    target=self._reconnect_worker, daemon=True)
                reconnect_thread.start()

        def on_message(client, userdata, msg):
            try:
                self.get_logger().debug(f'From {msg.topic}: {str(msg.payload)}')
                self._handle_mqtt_message(msg)
            except Exception as e:
                self.get_logger().error(f'Error handling MQTT message: {str(e)}')
                error_msg = String()
                error_msg.data = repr(e)
                self.error_publisher.publish(error_msg)

        def on_subscribe(client, userdata, mid, granted_qos):
            self.get_logger().info(
                f'MQTT subscribe ack: mid={mid}, granted_qos={granted_qos}')

        def on_log(client, userdata, level, buf):
            self.get_logger().info(f'MQTT debug log: level={level} {buf}')

        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_disconnect = on_disconnect
        self.mqtt_client.on_message = on_message
        self.mqtt_client.on_subscribe = on_subscribe
        if self.get_parameter('mqtt_debug_logs').value:
            self.mqtt_client.on_log = on_log

    def _setup_ros_publishers(self):
        """Initialize ROS publishers."""
        queue_size = self.get_parameter('ros_queue_size').value

        self.order_publisher = self.create_publisher(
            ros_loader.get_message_class(self.get_parameter('ros_publisher_type').value),
            'client_commands', queue_size)

        self.instant_actions_publisher = self.create_publisher(
            ros_loader.get_message_class('vda5050_msgs/InstantActions'),
            'instant_actions_commands', queue_size)

        self.error_publisher = self.create_publisher(
            String, 'order_valid_error', queue_size)

    def _setup_ros_subscribers(self):
        """Initialize ROS subscribers."""
        queue_size = self.get_parameter('ros_queue_size').value
        ros_msg_type = self.get_parameter('ros_subscriber_type').value

        self.create_subscription(
            ros_loader.get_message_class(ros_msg_type),
            'agv_state',
            self._handle_state_message,
            queue_size)

        self.create_subscription(
            ros_loader.get_message_class('vda5050_msgs/Factsheet'),
            'factsheet',
            self._handle_factsheet_message,
            queue_size)

    def _connect_mqtt_broker(self):
        """Establish connection to MQTT broker with retry logic."""
        max_retries = self.get_parameter('num_retries').value
        retries = 0
        retry_forever = self.get_parameter('retry_forever').value

        host = self.get_parameter('mqtt_host_name').value
        port = self.get_parameter('mqtt_port').value
        keep_alive = self.get_parameter('mqtt_keep_alive').value

        will_topic = f'{self.mqtt_topic_prefix}/connection'
        will_payload = self._connection_state_payload(State.CONNECTIONBROKEN)
        self.mqtt_client.will_set(will_topic, will_payload, qos=1, retain=False)

        base_delay = self.get_parameter('reconnect_period').value
        max_delay = 60

        while retries < max_retries or retry_forever:
            try:
                connect_result = self.mqtt_client.connect(host, port, keep_alive)
                self.get_logger().info(
                    f'MQTT connect() returned {connect_result}; '
                    f'prefix={self.mqtt_topic_prefix}')
                if not self._mqtt_loop_started:
                    self.mqtt_client.loop_start()
                    self._mqtt_loop_started = True

                for _ in range(10):
                    if self.mqtt_client.is_connected():
                        return True
                    time.sleep(1)

                raise ConnectionError('Connection timed out waiting for broker')

            except ConnectionRefusedError as e:
                self.get_logger().error(f'Connection Error: {e}. Please check mqtt_host_name.')
            except socket.timeout as e:
                self.get_logger().error(
                    f'Connection Error: {e}. Please verify host is reachable.')
            except socket.gaierror as e:
                self.get_logger().error(
                    f'Connection Error: {e}. Could not resolve mqtt_host_name.')
            except Exception as e:
                self.get_logger().error(f'Connection attempt failed: {str(e)}')

            try:
                if self._mqtt_loop_started:
                    self.mqtt_client.loop_stop()
                    self._mqtt_loop_started = False
            except Exception:
                pass
            delay = min(base_delay * (2 ** retries), max_delay)
            self.get_logger().info(
                f'Retrying in {delay}s (attempt {retries + 1}/{max_retries})...')
            time.sleep(delay)
            retries += 1

        self.get_logger().error('Failed to connect to MQTT broker, ending retries.')
        return False

    def _reconnect_worker(self):
        """Run _connect_mqtt_broker and clear the reconnecting flag when done."""
        try:
            self._connect_mqtt_broker()
        finally:
            with self._reconnect_lock:
                self._reconnecting = False

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        """
        Handle MQTT connection and setup.

        Args:
            client: MQTT client instance
            userdata: User data supplied to client
            flags: Response flags from broker
            rc: Return code from connection attempt
        """
        if rc != 0:
            connack_reasons = {
                1: 'incorrect protocol version',
                2: 'invalid client identifier',
                3: 'server unavailable',
                4: 'bad username or password',
                5: 'not authorized',
            }
            reason = connack_reasons.get(rc, 'unknown')
            self.get_logger().error(f'Connection failed with code {rc} ({reason})')
            return

        self._mqtt_ready = True
        topics = [
            f'{self.mqtt_topic_prefix}/order',
            f'{self.mqtt_topic_prefix}/instantActions',
        ]
        self.get_logger().info(f'MQTT connected. Subscribing to topics: {topics}')

        for topic in topics:
            sub_result, sub_mid = self.mqtt_client.subscribe(topic)
            self.get_logger().info(
                f'MQTT subscribe() topic={topic}, result={sub_result}, mid={sub_mid}')

        self._publish_connection_state(State.ONLINE)

    def _create_default_trajectory(self):
        """Create a default trajectory dictionary."""
        return {
            'degree': 1,
            'knot_vector': [],
            'control_points': []
        }

    def _set_default_values(self, message_dict):
        """Set default values for null fields in the order message."""
        # Set default values for edges
        if 'edges' in message_dict:
            edge_defaults = {
                'maxSpeed': 1.0,
                'maxHeight': 2.0,
                'minHeight': 0.0,
                'orientation': 0.0,
                'direction': 'NONE',
                'maxRotationSpeed': 0.5,
                'length': 0.0,
                'trajectory': self._create_default_trajectory()
            }

            for edge in message_dict['edges']:
                for key, default_value in edge_defaults.items():
                    if key not in edge or edge[key] is None:
                        edge[key] = default_value
                        if key == 'trajectory':
                            self.get_logger().debug('Setting default trajectory for edge')

        return message_dict

    def _handle_mqtt_message(self, msg):
        """Handle incoming MQTT messages."""
        try:
            message_dict = json.loads(str(msg.payload, 'utf-8'))

            if msg.topic.endswith('order'):
                message_dict = self._set_default_values(message_dict)
                if self.get_parameter('convert_case').value:
                    message_dict = convert_dict_keys(message_dict, 'camel_to_snake')
                ros_msg = ros_loader.get_message_instance(
                    self.get_parameter('ros_publisher_type').value)
                message_conversion.populate_instance(message_dict, ros_msg)
                self.order_publisher.publish(ros_msg)

            elif msg.topic.endswith('instantActions'):
                if self.get_parameter('convert_case').value:
                    message_dict = convert_dict_keys(message_dict, 'camel_to_snake')
                ros_msg = ros_loader.get_message_instance('vda5050_msgs/InstantActions')
                message_conversion.populate_instance(message_dict, ros_msg)
                self.instant_actions_publisher.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e).__name__}')
            error_msg = String()
            error_msg.data = repr(e)
            self.error_publisher.publish(error_msg)

    def _handle_state_message(self, msg):
        """Handle ROS state messages."""
        try:
            if not self._mqtt_ready:
                return

            extracted = message_conversion.extract_values(msg)
            self.last_state_msg = extracted

            if self.get_parameter('convert_case').value:
                extracted = convert_dict_keys(extracted, 'snake_to_dromedary')

            mqtt_topic = f'{self.mqtt_topic_prefix}/state'
            mqtt_payload = json.dumps(extracted)
            self.mqtt_client.publish(mqtt_topic, mqtt_payload)

            self._publish_visualization()

        except Exception as e:
            self.get_logger().error(f'Error processing state message: {str(e)}')

    def _handle_factsheet_message(self, msg):
        """Handle ROS factsheet messages."""
        try:
            if not self._mqtt_ready:
                return

            extracted = message_conversion.extract_values(msg)
            if self.get_parameter('convert_case').value:
                extracted = convert_dict_keys(extracted, 'snake_to_dromedary')

            self.mqtt_client.publish(
                f'{self.mqtt_topic_prefix}/factsheet',
                json.dumps(extracted))

        except Exception as e:
            self.get_logger().error(f'Error processing factsheet message: {str(e)}')

    def disconnect(self):
        """Clean up MQTT connection."""
        try:
            if self._mqtt_ready:
                self._publish_connection_state(State.OFFLINE)
            if self._mqtt_loop_started:
                self.mqtt_client.loop_stop()
                self._mqtt_loop_started = False
            self.mqtt_client.disconnect()

        except Exception as e:
            self.get_logger().error(f'Error during disconnect: {str(e)}')

    def _publish_visualization(self):
        """Publish visualization message."""
        try:
            if not self._mqtt_ready:
                return

            vis_msg = ros_loader.get_message_instance('vda5050_msgs/Visualization')
            vis_msg.manufacturer = self.get_parameter('manufacturer').value
            vis_msg.serial_number = self.get_parameter('serial_number').value

            if hasattr(self, 'last_state_msg'):
                state_msg = self.last_state_msg
                vis_msg.version = state_msg['version']
                vis_msg.header_id = state_msg['header_id']
                vis_msg.time_stamp = state_msg.get('timestamp', state_msg.get('time_stamp', ''))

                agv_pos = ros_loader.get_message_instance('vda5050_msgs/AGVPosition')
                agv_pos.position_initialized = state_msg['agv_position']['position_initialized']
                agv_pos.localization_score = state_msg['agv_position']['localization_score']
                agv_pos.deviation_range = state_msg['agv_position']['deviation_range']
                agv_pos.x = state_msg['agv_position']['x']
                agv_pos.y = state_msg['agv_position']['y']
                agv_pos.theta = state_msg['agv_position']['theta']
                agv_pos.map_id = state_msg['agv_position']['map_id']
                agv_pos.map_description = state_msg['agv_position']['map_description']
                vis_msg.agv_position = agv_pos

                vel = ros_loader.get_message_instance('vda5050_msgs/Velocity')
                vel.vx = state_msg['velocity']['vx']
                vel.vy = state_msg['velocity']['vy']
                vel.omega = state_msg['velocity']['omega']
                vis_msg.velocity = vel

            extracted = message_conversion.extract_values(vis_msg)
            if self.get_parameter('convert_case').value:
                extracted = convert_dict_keys(extracted, 'snake_to_dromedary')

            mqtt_topic = f'{self.mqtt_topic_prefix}/visualization'
            mqtt_payload = json.dumps(extracted)
            self.mqtt_client.publish(mqtt_topic, mqtt_payload)

        except Exception as e:
            self.get_logger().error(f'Error publishing visualization message: {str(e)}')


def main(args=None):
    """Execute the MqttRosBridgeNode."""
    rclpy.init(args=args)
    node = MqttRosBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cleaning up...')
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
