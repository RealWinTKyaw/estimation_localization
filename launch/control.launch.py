import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(
            'tf_prefix',
            default_value='none',
            description='TF Prefix'
        ),

        launch.actions.LogInfo(
            text=["Loading control.yaml..."]
        ),
        Node(
            package='rosparam',
            executable='rosparam',
            arguments=['load', launch.substitutions.LaunchConfiguration('estimation_assignment') + '/resources/control.yaml'],
            output='screen'
        ),
        Node(
            package='rosparam',
            executable='rosparam',
            arguments=[
                'param',
                'husky_velocity_controller/base_frame_id',
                launch.substitutions.LaunchConfiguration('tf_prefix') + '/base_link'
            ],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='base_controller_spawner',
            output='screen',
            arguments=['husky_joint_publisher', 'husky_velocity_controller', '--shutdown-timeout', '3'],
            respawn=True
        ),

        Node(
            package='interactive_marker_twist_server',
            executable='marker_server',
            name='twist_marker_server',
            output='screen'
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[launch.substitutions.LaunchConfiguration('estimation_assignment') + '/resources/twist_mux.yaml'],
            remappings=[('/cmd_vel_out', '/husky_velocity_controller/cmd_vel')]
        )

    ])
