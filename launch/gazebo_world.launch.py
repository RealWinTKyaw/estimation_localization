import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'world_name',
            default_value='walls_two_sided',
            description='World name'
        ),
        DeclareLaunchArgument(
            'enable_gui',
            default_value='true',
            description='Enable GUI'
        ),
        DeclareLaunchArgument(
            'enable_headless',
            default_value='false',
            description='Enable Headless'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch.substitutions.LaunchConfiguration('gazebo_ros'), '/launch/empty_world.launch.py']),
            launch_arguments={
                'world_name': launch.substitutions.LaunchConfiguration('world_name'),
                'debug': 'false',
                'paused': 'false',
                'headless': launch.substitutions.LaunchConfiguration('enable_headless'),
                'gui': launch.substitutions.LaunchConfiguration('enable_gui')
            }.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map', '100']
        )
    ])
