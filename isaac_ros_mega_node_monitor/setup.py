# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from glob import glob
import os

from setuptools import setup

package_name = 'isaac_ros_mega_node_monitor'

setup(
    name=package_name,
    version='3.2.6',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuzho',
    maintainer_email='yuzho@nvidia.com',
    description='ROS2 service to monitor node status',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'isaac_ros_mega_node_monitor = \
                isaac_ros_mega_node_monitor.isaac_ros_mega_node_monitor:main',
        ],
    },
)
