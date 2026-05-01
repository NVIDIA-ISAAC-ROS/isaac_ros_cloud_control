# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# This repository implements data types and logic specified in the VDA5050
# protocol, which is specified here
# https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md

# Header ID of the message
uint32 header_id

# Timestamp in ISO8601 format (YYYY-MM-DDTHH:mm:ss.ffZ)
string time_stamp

# Version of the protocol [Major].[Minor].[Patch]
string version

# Manufacturer of the AGV
string manufacturer

# Serial number of the AGV
string serial_number

# The AGV's position
vda5050_msgs/AGVPosition agv_position

# The AGV's velocity in vehicle coordinates
vda5050_msgs/Velocity velocity 