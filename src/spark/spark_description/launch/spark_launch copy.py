# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os
import launch
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from typing import List, Optional, Text, Union

from launch import LaunchContext, SomeSubstitutionsType
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


class DeclareSparkRobotDescriptionLaunchArgument(DeclareLaunchArgument):
    """Generate a URDF of a robot through a modified DeclareLaunchArgument object."""

    def __init__(
        self,
        *,
        default_value: Optional[SomeSubstitutionsType] = Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('spark_description'),
                'urdf',
                'spark_340'
            ]), '.urdf.xacro ',
            'enable_arm_tel:=', LaunchConfiguration('enable_arm_tel'), ' ',
            'arm_type_tel:=', LaunchConfiguration('arm_type_tel'), ' ',
            'camera_type_tel:=', LaunchConfiguration('camera_type_tel'), ' ',
            'lidar_type_tel:=', LaunchConfiguration('lidar_type_tel'), ' ',
        ]),
        **kwargs
    ) -> None:
        """
        Construct the modified DeclareLaunchArgument object.
        :param default_value: The default model given to the parent DeclareLaunchArgument; if you
            want to override this value, it must follow the convention in this object's source
        """
        super().__init__(
            name='robot_description',
            default_value=default_value,
            description=(
                'URDF of the robot; this is typically generated by the xacro command.'
            ),
            choices=None,
            **kwargs
        )


def declare_spark_robot_description_launch_arguments(
    *,
    enable_arm_tel: Text = 'false',
    arm_type_tel: Text = 'uarm',
    camera_type_tel: Text = 'd435',
    lidar_type_tel: Text = 'ydlidar_g6',
) -> List[DeclareLaunchArgument]:
    """
    Return the `robot_description` DeclareLaunchArgument and its requried children.
    DeclareLaunchArgument objects:
        - `enable_arm_tel`
        - `arm_type_tel`
        - `camera_type_tel`
        - `lidar_type_tel`
    :details: Include this in your LaunchDescription by appending its output to the list of
        DeclareLaunchArguments
    """
    return [
        DeclareLaunchArgument(
            'enable_arm_tel',
            default_value='false', #TextSubstitution(text=enable_arm_tel),
            choices=('true', 'false'),
            description=(
                "name of the 'root' link on the arm; typically `base_link`, but can be changed if "
                'attaching the arm to a mobile base that already has a `base_link` frame.'
            ),
        ),
        DeclareLaunchArgument(
            'arm_type_tel',
            default_value='uarm', #TextSubstitution(text=arm_type_tel),
            description=(
                'if `true`, the default gripper is included in the `robot_description` parameter; '
                'if `false`, it is left out; set to `false` if not using the default gripper.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_type_tel',
            default_value='d435',
            description=(
                'if `true`, the AR tag mount is included in the `robot_description` parameter; if '
                '`false`, it is left out; set to `true` if using the AR tag mount in your project.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_type_tel',
            default_value='ydlidar_g6',
            description=(
                'if `true`, the gripper_bar link is included in the `robot_description` parameter;'
                ' if `false`, the gripper_bar and finger links are not loaded. Set to `false` if '
                'you have a custom gripper attachment.'
            ),
        ),
        DeclareSparkRobotDescriptionLaunchArgument(),
    ]


# def determine_use_sim_time_param(
#     context: LaunchContext,
#     hardware_type_launch_arg: LaunchConfiguration
# ) -> Union[TextSubstitution, LaunchConfiguration]:
#     """
#     Set `use_sim_time` parameter to `true` if using simulated hardware.
#     :param context: The launch context
#     :param hardware_type: The `hardware_type` LaunchConfiguration
#     :return: True if hardware is simulated, the `use_sim_time` LaunchConfiguration otherwise
#     """
#     if hardware_type_launch_arg.perform(context) in ('gz_classic'):
#         print('000000000')
#         return TextSubstitution(text='true')
#     else:
#         print('111111111111')
#         return LaunchConfiguration('use_sim_time')

#==================================================================================

def launch_setup(context, *args, **kwargs):
    enable_arm_tel = LaunchConfiguration('enable_arm_tel')
    arm_type_tel = LaunchConfiguration('arm_type_tel')
    camera_type_tel = LaunchConfiguration('camera_type_tel')
    lidar_type_tel = LaunchConfiguration('lidar_type_tel')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    # hardware_type_launch_arg = LaunchConfiguration('hardware_type')

    # # sets use_sim_time parameter to 'true' if using gazebo hardware
    # use_sim_time_param = determine_use_sim_time_param(
    #     context=context,
    #     hardware_type_launch_arg=hardware_type_launch_arg
    # )
    xacro_path = os.path.join(get_package_share_directory('spark_description'), 'urdf', 'spark_340.urdf.xacro')
    urdf = to_urdf(xacro_path, {'camera_type_tel' : 'd435', 'lidar_type_tel' : 'ydlidar_g6'})
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_launch_arg
        #    'use_sim_time': use_sim_time_param,
        }],
        # arguments = [urdf],

        output={'both': 'log'},
    )

    rviz_config_dir = os.path.join(get_package_share_directory('spark_description'), 'rviz', 'urdf.rviz')


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_dir,
        ],
        # parameters=[{
        #     'use_sim_time': use_sim_time_param,
        # }],

        output={'both': 'log'},
    )

    return [
        robot_state_publisher_node,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'enable_arm_tel',
    #         default_value='false',
    #         #choices=get_interbotix_xsarm_models(),
    #         description='model type of the Interbotix Arm such as `wx200` or `rx150`.'
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'arm_type_tel',
    #         default_value='uarm',
    #         description=(
    #             'name of the robot (typically equal to `robot_model`, but could be anything).'
    #         ),
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'camera_type_tel',
    #         default_value='d435',
    #         description='launches RViz if set to `true`.',
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'lidar_type_tel',
    #         default_value='ydlidar_g6',
    #         description='launches the joint_state_publisher node.',
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'lidar_type_teld',
    #         default_value='ydlidar_g6',
    #         description='launches the joint_state_publisher node.',
    #     )
    # )
    declared_arguments.extend(
        declare_spark_robot_description_launch_arguments(),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])