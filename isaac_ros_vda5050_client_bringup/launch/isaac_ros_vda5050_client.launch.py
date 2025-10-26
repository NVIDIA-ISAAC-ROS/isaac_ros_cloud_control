# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for VDA5050 Client node."""
    launch_args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for ROS nodes in this launch script'),
        DeclareLaunchArgument('mqtt_host_name', default_value='localhost',
                              description='MQTT broker IP to connect to'),
        DeclareLaunchArgument('mqtt_transport', default_value='tcp',
                              description='Protocol for MQTT messages to be sent (pass either tcp'
                                          'or websockets)'),
        DeclareLaunchArgument('mqtt_username', default_value='',
                              description='Username to authenticate to MQTT broker'),
        DeclareLaunchArgument('mqtt_password', default_value='',
                              description='Password to authenticate to MQTT broker'),
        DeclareLaunchArgument('interface_name', default_value='uagv',
                              description='Name of the used interface. Used to construct mqtt '
                                          'topic names.'),
        DeclareLaunchArgument('major_version', default_value='v2',
                              description='VDA5050 major version.'),
        DeclareLaunchArgument('manufacturer', default_value='RobotCompany',
                              description='Manufacturer of the AGV.'),
        DeclareLaunchArgument('serial_number', default_value='carter01',
                              description='Unique AGV Serial Number'),
        DeclareLaunchArgument('ros_subscriber_type', default_value='vda5050_msgs/AGVState',
                              description='ROS message type to convert to outgoing MQTT message'),
        DeclareLaunchArgument('ros_publisher_type', default_value='vda5050_msgs/Order',
                              description='ROS message type to convert received MQTT message to'),
        DeclareLaunchArgument('mqtt_port', default_value='1883',
                              description='The port of the MQTT message broker to connect to'),
        DeclareLaunchArgument('retry_forever', default_value='true',
                              description='Retry forever to connect to MQTT message broker if'
                                          'connection is not established'),
        DeclareLaunchArgument('reconnect_period', default_value='5',
                              description='The period of time to wait before retrying to connect'
                                          'to MQTT message broker (in seconds)'),
        DeclareLaunchArgument('num_retries', default_value='10',
                              description='Number of reconnection retries to MQTT message broker'
                                          'before giving up'),
        DeclareLaunchArgument('ros_recorder', default_value='false',
                              description='Launch ROS scene recorder if true'),
        DeclareLaunchArgument('odom_topic', default_value='/chassis/odom',
                              description='The topic that publishes nav_msgs/Odometry message.'),
        DeclareLaunchArgument('robot_type', default_value='CARRIER',
                              description='The type of robot we are inspecting. Valid values are '
                                          '[FORKLIFT, CONVEYOR, TUGGER, CARRIER, MANIPULATOR, '
                                          'HUMANOID]'),
        DeclareLaunchArgument('battery_state_topic', default_value='/chassis/battery_state',
                              description='The topic that publishes battery state message.'),
        DeclareLaunchArgument('launch_ws_bridge', default_value='False',
                              description='Launch rosbridge websocket if set to True.'),
        DeclareLaunchArgument('status_check_service', default_value='/velocity_smoother/get_state',
                              description='The service to check to verify NAV2 is ready'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(
            get_package_share_directory(
                'isaac_ros_vda5050_client_bringup'), 'config', 'vda5050_client_params.yaml'),
                description='The path to the client config file'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='The log level to use for the client node')
    ]

    namespace = LaunchConfiguration('namespace')
    mqtt_host_name = LaunchConfiguration('mqtt_host_name')
    mqtt_transport = LaunchConfiguration('mqtt_transport')
    mqtt_username = LaunchConfiguration('mqtt_username')
    mqtt_password = LaunchConfiguration('mqtt_password')
    ros_subscriber_type = LaunchConfiguration('ros_subscriber_type')
    ros_publisher_type = LaunchConfiguration('ros_publisher_type')
    mqtt_port = LaunchConfiguration('mqtt_port')
    retry_forever = LaunchConfiguration('retry_forever')
    reconnect_period = LaunchConfiguration('reconnect_period')
    num_retries = LaunchConfiguration('num_retries')
    ros_recorder = LaunchConfiguration('ros_recorder')
    interface_name = LaunchConfiguration('interface_name')
    major_version = LaunchConfiguration('major_version')
    manufacturer = LaunchConfiguration('manufacturer')
    serial_number = LaunchConfiguration('serial_number')
    odom_topic = LaunchConfiguration('odom_topic')
    robot_type = LaunchConfiguration('robot_type')
    battery_state_topic = LaunchConfiguration('battery_state_topic')
    status_check_service = LaunchConfiguration('status_check_service')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    client_node = Node(
        namespace=namespace,
        name='vda5050_client_node',
        package='isaac_ros_vda5050_client',
        executable='vda5050_client',
        parameters=[{
            'update_feedback_period': 1.0,
            'config_file': config_file,
            'odom_topic': odom_topic,
            'robot_type': robot_type,
            'battery_state_topic': battery_state_topic,
            'status_check_service': status_check_service
        }],
        arguments=['--ros-args', '--log-level', ['vda5050_client_node:=', log_level]],
        output='screen'
    )

    ros_mqtt_bridge_node = Node(
        name='ros_mqtt_bridge_node',
        package='isaac_ros_mqtt_bridge',
        executable='ros_to_mqtt_bridge_node',
        parameters=[{
            'mqtt_host_name': mqtt_host_name,
            'mqtt_transport': mqtt_transport,
            'mqtt_username': mqtt_username,
            'mqtt_password': mqtt_password,
            'interface_name': interface_name,
            'major_version': major_version,
            'manufacturer': manufacturer,
            'serial_number': serial_number,
            'ros_subscriber_type': ros_subscriber_type,
            'mqtt_port': mqtt_port,
            'retry_forever': retry_forever,
            'reconnect_period': reconnect_period,
            'num_retries': num_retries
        }],
        namespace=namespace,
        remappings=[('ros_sub_topic', 'agv_state')],
        output='screen'
    )

    mqtt_ros_bridge_node = Node(
        name='mqtt_ros_bridge_node',
        package='isaac_ros_mqtt_bridge',
        executable='mqtt_to_ros_bridge_node',
        parameters=[{
            'mqtt_host_name': mqtt_host_name,
            'mqtt_transport': mqtt_transport,
            'mqtt_username': mqtt_username,
            'mqtt_password': mqtt_password,
            'interface_name': interface_name,
            'major_version': major_version,
            'manufacturer': manufacturer,
            'serial_number': serial_number,
            'ros_publisher_type': ros_publisher_type,
            'mqtt_port': mqtt_port,
            'retry_forever': retry_forever,
            'reconnect_period': reconnect_period,
            'num_retries': num_retries
        }],
        namespace=namespace,
        remappings=[('bridge_pub_topic', 'client_commands')],
        output='screen'
    )

    recorder_node = Node(
        namespace=namespace,
        name='recorder_action_server',
        package='isaac_ros_scene_recorder',
        executable='scene_recorder',
        parameters=[],
        output='screen',
        condition=IfCondition(ros_recorder)
    )

    rosbridge_server_launch_dir = os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch')
    rosbridge_server_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([rosbridge_server_launch_dir,
                                    '/rosbridge_websocket_launch.xml']),
        launch_arguments={'namespace': namespace}.items(),
        condition=IfCondition(LaunchConfiguration('launch_ws_bridge'))
    )

    return LaunchDescription(launch_args +
                             [
                                 client_node,
                                 ros_mqtt_bridge_node,
                                 mqtt_ros_bridge_node,
                                 recorder_node,
                                 rosbridge_server_launch
                             ])
