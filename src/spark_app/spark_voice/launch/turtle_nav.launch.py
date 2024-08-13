from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    lang=LaunchConfiguration('language',default='zh')
    print(lang)
    return LaunchDescription([
        DeclareLaunchArgument(
        'language', 
        default_value='zh',
        choices=['zh', 'en'],
        description='Choice language'), 
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='spark_voice',
            executable='local_asr',
            name='local_asr',
            parameters=[{'language':lang}]
        ),
        Node(
            package='spark_voice',
            executable='voice_nav',
            name='voice_nav',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
            ]
        )
    ])