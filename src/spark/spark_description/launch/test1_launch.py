# Copyright (c) 2022 NXROBO
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: litian.zhuang */

import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.actions import LogInfo, DeclareLaunchArgument

def generate_launch_description():
    Ros1BagPlayNode = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
        name='turtlesim_nodex',
        output='screen',
        )
    Ros2BagPlayNode = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
        name='turtlesim_nodexx',
        output='screen',
        )

    return launch.LaunchDescription([Ros1BagPlayNode,Ros2BagPlayNode])
