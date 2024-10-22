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

def generate_launch_description():
    available_urdf_files = [f for f in os.listdir(os.path.join(get_package_share_directory('spark_description'), 'urdf')) if f.startswith('spark')]
    params = dict([cmd_arg for cmd_arg in [cmd_arg.split(':=') for cmd_arg in sys.argv] if len(cmd_arg)==2])
    if ('model' not in params or params['model'] not in available_urdf_files):
        print ('USAGE:')
        print ('ros2 launch spark_description view_model.launch.py model:=<model>')
        print ('Available argumants for <model> are as follows:')
        print ('\n'.join(available_urdf_files))
        exit(-1)

    rviz_config_dir = os.path.join(get_package_share_directory('spark_description'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('spark_description'), 'urdf', params['model'])
    urdf = to_urdf(xacro_path, {'camera_type_tel' : 'd435', 'lidar_type_tel' : 'ydlidar_g6'})
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        node_name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )
    model_node = Node(
        node_name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',

        )
    return launch.LaunchDescription([rviz_node, model_node])
