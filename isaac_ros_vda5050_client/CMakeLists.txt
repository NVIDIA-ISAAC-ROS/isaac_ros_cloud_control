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

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_vda5050_client LANGUAGES C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

# The FindPythonInterp and FindPythonLibs modules are removed
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

execute_process(
  COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CUDA_MIN_VERSION "11.4")

# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs)
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(vda5050_client_node SHARED
  src/vda5050_client_node.cpp)
target_link_libraries(vda5050_client_node
  yaml-cpp
)
target_compile_definitions(vda5050_client_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(vda5050_client_node
  "isaac_ros::mission_client::Vda5050ClientNode")
set(node_plugins "${node_plugins}isaac_ros::mission_client::Vda5050ClientNode;$<TARGET_FILE:vda5050_client_node>\n")

ament_auto_add_executable("vda5050_client" src/vda5050_client_main.cpp)


set(example_handler_dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  pluginlib
  vda5050_action_handler
  vda5050_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  nav2_msgs
  vda5050_msgs
  tf2_geometry_msgs
  lifecycle_msgs
  sensor_msgs
)
add_library(example_action_handler SHARED
  src/example_action_handler.cpp
)
target_include_directories(example_action_handler
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(example_action_handler PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(example_action_handler
  ${example_handler_dependencies}
)

install(TARGETS vda5050_client_node example_action_handler
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  test/test_nodes/nav2_simple_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY test/test_orders
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_nav2_client.py TIMEOUT "40")
endif()

ament_export_libraries(
  example_action_handler
)
pluginlib_export_plugin_description_file(vda5050_action_handler plugin.xml)

# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE)
