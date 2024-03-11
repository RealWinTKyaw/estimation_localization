import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick Device'
        ),
        DeclareLaunchArgument(
            'joystick',
            default_value='true',
            description='Enable Joystick'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='teleop.yaml',
            description='Configuration file for teleop'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': LaunchConfiguration('joy_dev')}]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            namespace='joy_teleop'
        )

    ])
