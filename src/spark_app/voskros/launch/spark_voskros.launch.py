from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    device=LaunchConfiguration('device',default='')
    model=LaunchConfiguration('model',default='en-us')
    grammar=LaunchConfiguration('grammar',default='')
    samplerate=LaunchConfiguration('samplerate',default='0')
    result=LaunchConfiguration('result',default='result')
    ns=LaunchConfiguration('ns',default='/stt')



    return LaunchDescription([
        DeclareLaunchArgument(
        'model', 
        default_value='en-us',
        choices=['cn', 'en-us'],
        description='Choice language'), 

        DeclareLaunchArgument(
        'device', 
        default_value='',
        description='Choice voice device'), 

        DeclareLaunchArgument(
        'grammar', 
        default_value='',
        description='grammar'), 

        DeclareLaunchArgument(
        'samplerate', 
        default_value='0',
        description='Choice samplerate'), 

        DeclareLaunchArgument(
        'result', 
        default_value='result',
        description='Choice topic txt outp topic name'), 

        DeclareLaunchArgument(
        'ns', 
        default_value='/stt',
        description='Choice namespace'), 

        Node(
            package='voskros',
            executable='vosk',
            name='vosk',
            parameters=[{'model':model,
                         'device':device,
                         'grammar':grammar,
                         'samplerate':samplerate,
                         'result':result}],
            # remappings=[],
            # namespace=nss,
            namespace=ns
        ),
    ])