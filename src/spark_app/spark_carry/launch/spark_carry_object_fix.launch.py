# Copyright (c) 2022 NXROBO
#
# /* Author: litian.zhuang */
# /* email: litian.zhuang@nxrobo.com */
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
# 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import Node
from launch.substitutions import PythonExpression



def generate_launch_description():
    # Get the launch directory
    swiftpro_driver_dir = get_package_share_directory('swiftpro')
    spark_carry_dir = get_package_share_directory('spark_carry')
    spark_bringup_dir = get_package_share_directory('spark_bringup')

    # Create the launch configuration variables
    serial_port = LaunchConfiguration('serial_port')
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    start_base = LaunchConfiguration('start_base')
    start_camera = LaunchConfiguration('start_camera')
    start_lidar = LaunchConfiguration('start_lidar')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')   
    dp_rgist = LaunchConfiguration('dp_rgist')   
    start_bringup_rviz = LaunchConfiguration('start_bringup_rviz')   


    declare_serial_port = DeclareLaunchArgument(
        'serial_port', 
        default_value='/dev/sparkBase',
        description='serial port name:/dev/sparkBase or /dev/ttyUSBx')
    declare_enable_arm_tel = DeclareLaunchArgument(
        'enable_arm_tel', 
        default_value='false',
        choices=['true', 'false'],
        description='Whether to run arm')
    declare_arm_type_tel = DeclareLaunchArgument(
        'arm_type_tel', 
        default_value='uarm',
        choices=['uarm', 'sagittarius_arm'],
        description='arm name')
    declare_start_base = DeclareLaunchArgument(
        'start_base', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run base')
    declare_start_camera = DeclareLaunchArgument(
        'start_camera', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run camera')
    declare_start_lidar = DeclareLaunchArgument(
        'start_lidar', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run lidar')
    declare_camera_type_tel = DeclareLaunchArgument(
        'camera_type_tel', 
        default_value='d435',
        #choices=['d435', 'astra_pro'],
        description='camera type')
    declare_lidar_type_tel = DeclareLaunchArgument(
        'lidar_type_tel', 
        default_value='ydlidar_g6',
        #choices=['ydlidar_g2', 'ydlidar_g6'],
        description='lidar type')
    declare_dp_rgist = DeclareLaunchArgument(
        'dp_rgist', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to run dp_rgist')
    declare_start_bringup_rviz = DeclareLaunchArgument(
        'start_bringup_rviz', 
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start_bringup_rviz')    
    declare_color_topic_name = DeclareLaunchArgument(
        'color_topic_name', 
        default_value=PythonExpression(["'/camera/color/image_raw' if '", camera_type_tel, "' == 'd435' else '/camera/rgb/image_raw'"])
    )

   
    # Specify the actions
    driver_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(spark_bringup_dir, 'launch',
                                                       'driver_bringup.launch.py')),
            launch_arguments={'serial_port': serial_port,
                              'enable_arm_tel': enable_arm_tel,
                              'arm_type_tel': arm_type_tel,
                              'start_base' : start_base,
                              'start_camera': start_camera,
							                'start_lidar': start_lidar,
                              'camera_type_tel' : camera_type_tel,
                              'lidar_type_tel': lidar_type_tel,
							                'dp_rgist': dp_rgist,
                              'start_bringup_rviz' : start_bringup_rviz,}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(swiftpro_driver_dir, 'launch',
                                                       'pro_control_nomoveit.launch.py')),
            launch_arguments={}.items()),
    ])

 
    s_carry_object_node = launch_ros.actions.Node(
        package='spark_carry',
        executable='s_carry_object',  
        output='screen',
        emulate_tty=True,
        remappings=[('/camera/color/image_raw', LaunchConfiguration('color_topic_name'))],

    )

    cali_send_topic_node = launch_ros.actions.Node(
        package='spark_carry',
        executable='cmd_spark_carry_start.sh',  
        output='screen',
        emulate_tty=True,
        )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_serial_port)
    ld.add_action(declare_enable_arm_tel)
    ld.add_action(declare_arm_type_tel)
    ld.add_action(declare_start_base)
    ld.add_action(declare_start_lidar)
    ld.add_action(declare_start_camera)
    ld.add_action(declare_camera_type_tel)
    ld.add_action(declare_lidar_type_tel)
    ld.add_action(declare_dp_rgist)
    ld.add_action(declare_start_bringup_rviz)
    ld.add_action(declare_color_topic_name)    
    ld.add_action(driver_group)
    ld.add_action(s_carry_object_node)
    ld.add_action(cali_send_topic_node)
    return ld
