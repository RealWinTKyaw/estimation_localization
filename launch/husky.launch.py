import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'laser_enabled',
            default_value='true',
            description='Enable Laser'
        ),
        DeclareLaunchArgument(
            'ur5_enabled',
            default_value='false',
            description='Enable UR5'
        ),
        DeclareLaunchArgument(
            'kinect_enabled',
            default_value='false',
            description='Enable Kinect'
        ),
        DeclareLaunchArgument(
            'husky_name',
            default_value='husky',
            description='Husky Robot Name'
        ),
        DeclareLaunchArgument(
            'odometry_noise_std_dev',
            default_value='0.0',
            description='Odometry Noise Standard Deviation'
        ),

        GroupAction([
            Node(
                package='estimation_assignment',
                executable='ground_truth_tf_publisher.py',
                name='estimation_assignment',
                output='screen'
            ),

            GroupAction(
                namespace=launch.substitutions.LaunchConfiguration('husky_name'),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([launch.substitutions.LaunchConfiguration('estimation_assignment'), '/launch/spawn_husky.launch.py']),
                        launch_arguments={
                            'laser_enabled': launch.substitutions.LaunchConfiguration('laser_enabled'),
                            'ur5_enabled': launch.substitutions.LaunchConfiguration('ur5_enabled'),
                            'kinect_enabled': launch.substitutions.LaunchConfiguration('kinect_enabled'),
                            'x': launch.substitutions.LaunchConfiguration('xinit'),
                            'y': launch.substitutions.LaunchConfiguration('yinit'),
                            'z': launch.substitutions.LaunchConfiguration('zinit'),
                            'robot_name': launch.substitutions.LaunchConfiguration('husky_name'),
                            'tf_prefix': launch.substitutions.LaunchConfiguration('husky_name'),
                            'odometry_noise_std_dev': launch.substitutions.LaunchConfiguration('odometry_noise_std_dev')
                        }.items()
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([launch.substitutions.LaunchConfiguration('estimation_assignment'), '/launch/teleop.launch.py']),
                        launch_arguments={
                            'joy_dev': '/dev/input/js0',
                            'joystick': 'true'
                        }.items()
                    ),
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='odom_to_world',
                        output='screen',
                        arguments=['$(arg xinit)', '$(arg yinit)', '$(arg zinit)', '0', '0', '0', 'map', '$(arg husky_name)_1/odom', '100']
                    ),
                ]
            ),
        ])
    ])
