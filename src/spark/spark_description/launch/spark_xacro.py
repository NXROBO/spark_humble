

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
            default_value=TextSubstitution(text=enable_arm_tel),
            choices=('true', 'false'),
            description=(
                "Whether to enable the arm model" 
            ),
        ),
        DeclareLaunchArgument(
            'arm_type_tel',
            default_value=TextSubstitution(text=arm_type_tel),
            choices=('uarm', 'sagittarius_arm'),
            description=(
                "which arm to be used"
            ),
        ),
        DeclareLaunchArgument(
            'camera_type_tel',
            default_value=camera_type_tel,
            choices=('d435', 'astrapro'),
            description=(
                "which camera to be used"
            ),
        ),
        DeclareLaunchArgument(
            'lidar_type_tel',
            default_value=lidar_type_tel,
            choices=('ydlidar_g6', 'ydlidar_g2'),
            description=(
                "which lidar to be used"
            ),
        ),
        DeclareSparkRobotDescriptionLaunchArgument(),
    ]


def determine_use_sim_time_param(
    context: LaunchContext,
    camera_type_tel_launch_arg: LaunchConfiguration
) -> Union[TextSubstitution, LaunchConfiguration]:
    """
    Set `use_sim_time` parameter to `true` if using simulated hardware.

    :param context: The launch context
    :param camera_type_tel: The `camera_type_tel` LaunchConfiguration
    :return: True if hardware is simulated, the `use_sim_time` LaunchConfiguration otherwise
    """
    if camera_type_tel_launch_arg.perform(context) in ('d435'):
        return TextSubstitution(text='true')
    else:
        return LaunchConfiguration('use_sim_time')