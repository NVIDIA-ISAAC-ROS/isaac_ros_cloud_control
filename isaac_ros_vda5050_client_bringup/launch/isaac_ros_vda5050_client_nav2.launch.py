# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Generate launch description for VDA5050 Nav2 Client node."""
    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for ROS nodes in this launch script'),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed Nav2 bringup'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Omniverse Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'init_pose_x',
            default_value='0.0',
            description='Initial position X coordinate'),
        DeclareLaunchArgument(
            'init_pose_y',
            default_value='0.0',
            description='Initial position Y coordinate'),
        DeclareLaunchArgument(
            'init_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory(
                    'isaac_ros_vda5050_client_bringup'),
                'maps', 'carter_warehouse_navigation.yaml'
            ),
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'isaac_ros_vda5050_client_bringup'),
                'config', 'carter_navigation_params.yaml'
            ),
            description='Full path to navigation param file to load'),
        DeclareLaunchArgument(
            'info_generator_params_file',
            default_value=os.path.join(
                get_package_share_directory(
                    'isaac_ros_vda5050_client_bringup'),
                'config', 'json_info_generator_params.yaml'
            ),
            description='Full path to JSON info generator param file to load'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='False',
            description='Launch RViz if set to True'),
    ]
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose_x = LaunchConfiguration('init_pose_x', default=0.0)
    init_pose_y = LaunchConfiguration('init_pose_y', default=0.0)
    init_pose_yaw = LaunchConfiguration('init_pose_yaw', default=0.0)
    map_dir = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file',)
    info_generator_params_file = LaunchConfiguration('info_generator_params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    param_substitutions = {
        'x': init_pose_x,
        'y': init_pose_y,
        'yaw': init_pose_yaw
    }

    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    mission_client_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_vda5050_client_bringup'), 'launch')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('isaac_ros_vda5050_client_bringup'),
        'config', 'carter_navigation.rviz')

    json_info_generator_node = Node(
        name='json_info_generator_node',
        package='isaac_ros_json_info_generator',
        executable='json_info_generator_node',
        parameters=[info_generator_params_file],
        namespace=namespace,
        remappings=[('ros_sub_topic', 'agv_state')],
        output='screen'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_composition': use_composition,
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': configured_params}.items(),
    )

    mission_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([mission_client_launch_dir,
                                       '/isaac_ros_vda5050_client.launch.py']),
        launch_arguments={'namespace': namespace}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(launch_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_dir}.items(),
    )

    return LaunchDescription(launch_args +
                             [json_info_generator_node,
                              nav2_bringup_launch,
                              mission_client_launch,
                              rviz_launch,

                              ])
