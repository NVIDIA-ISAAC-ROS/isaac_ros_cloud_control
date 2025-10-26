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

cmake_minimum_required(VERSION 3.22.1)
project(vda5050_action_handler_plugins LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)

find_package(vda5050_action_handler REQUIRED)
find_package(pluginlib REQUIRED)
find_package(isaac_ros_vda5050_client REQUIRED)
find_package(vda5050_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(isaac_manipulator_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

set(dependencies
  rclcpp
  pluginlib
  vda5050_action_handler
  vda5050_msgs
  isaac_ros_vda5050_client
  std_srvs
  isaac_manipulator_interfaces
  isaac_ros_scene_recorder_interface
)

add_library(vda5050_action_handler_plugins
  src/docking_handler.cpp
  src/pick_and_place_handler.cpp
  src/scene_recorder_handler.cpp
)

target_link_libraries(vda5050_action_handler_plugins
  yaml-cpp
  nlohmann_json
)

target_compile_definitions(vda5050_action_handler_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(vda5050_action_handler_plugins
  ${dependencies}
)

install(TARGETS vda5050_action_handler_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  vda5050_action_handler_plugins
)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(vda5050_action_handler plugins.xml)

ament_package()
