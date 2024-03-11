import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'odometry_orientation_noise_std_dev',
            default_value='0.1',
            description='Odometry Orientation Noise Standard Deviation'
        ),
        DeclareLaunchArgument(
            'odometry_position_noise_std_dev',
            default_value='0.05',
            description='Odometry Position Noise Standard Deviation'
        ),

        Node(
            package='estimation_assignment',
            executable='occupancy_grid_mapper.py',
            name='occupancy_grid_mapper',
            output='screen',
            parameters=[
                {'odometry_position_noise_std_dev': launch.substitutions.LaunchConfiguration('odometry_position_noise_std_dev')},
                {'odometry_orientation_noise_std_dev': launch.substitutions.LaunchConfiguration('odometry_orientation_noise_std_dev')}
            ]
        )

    ])
