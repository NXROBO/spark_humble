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

    start_description_rviz = LaunchConfiguration('start_description_rviz')   
    declare_start_description_rviz = DeclareLaunchArgument(
        'start_description_rviz', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start_description_rviz')    

    rviz_config_dir = os.path.join(get_package_share_directory('spark_description'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('spark_description'), 'urdf', 'spark_340.urdf.xacro')
    urdf = to_urdf(xacro_path, {'camera_type_tel' : 'd435', 'lidar_type_tel' : 'ydlidar_g6'})
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(start_description_rviz),
        parameters=[{'use_sim_time': False}]
        )
    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )

    model_node1 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', urdf],
        name='turtlesim_nodex',
        output='screen',
        )
    lgolog=LogInfo(msg='Version 1', condition=LaunchConfigurationEquals('bag_version', 'v1')),

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',

        )
    return launch.LaunchDescription([declare_start_description_rviz, rviz_node, model_node])
