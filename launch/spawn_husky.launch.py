import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
            'x',
            default_value='0.0',
            description='X coordinate for spawning the robot'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Y coordinate for spawning the robot'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Z coordinate for spawning the robot'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='/',
            description='TF Prefix'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='/',
            description='Robot Name'
        ),
        DeclareLaunchArgument(
            'odometry_noise_std_dev',
            default_value='0.0',
            description='Odometry Noise Standard Deviation'
        ),

        Node(
            package='xacro',
            executable='xacro',
            name='xacro',
            output='screen',
            arguments=[
                launch.substitutions.LaunchConfiguration('estimation_assignment') + '/urdf/description.gazebo.xacro',
                'laser_enabled:=' + launch.substitutions.LaunchConfiguration('laser_enabled'),
                'ur5_enabled:=' + launch.substitutions.LaunchConfiguration('ur5_enabled'),
                'kinect_enabled:=' + launch.substitutions.LaunchConfiguration('kinect_enabled'),
                'namespace:=' + launch.substitutions.LaunchConfiguration('robot_name'),
                'odometry_noise_std_dev:=' + launch.substitutions.LaunchConfiguration('odometry_noise_std_dev')
            ],
            parameters=[{'robot_description': ''}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch.substitutions.LaunchConfiguration('estimation_assignment'), '/launch/control.launch.py']),
            launch_arguments={'tf_prefix': launch.substitutions.LaunchConfiguration('tf_prefix')}.items()
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            name='arm_controller_spawner',
            output='screen',
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('ur5_enabled')),
            arguments=['arm_controller', '--shutdown-timeout', '3']
        ),

        Node(
            package='rostopic',
            executable='rostopic',
            name='fake_joint_calibration',
            output='screen',
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('ur5_enabled')),
            arguments=['pub', 'calibrated', 'std_msgs/Bool', 'true']
        ),

        Node(
            package='husky_control',
            executable='stow_ur5',
            name='stow_ur5',
            output='screen',
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('ur5_enabled'))
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('kinect_enabled')),
            remappings=[('/cloud_in', '/kinect/depth/points'), ('/scan', '/kinect/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'tolerance': 1.0,
                'min_height': 0.05,
                'max_height': 1.0,
                'angle_min': -0.52,
                'angle_max': 0.52,
                'angle_increment': 0.005,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'concurrency_level': 1
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_husky_model',
            output='screen',
            arguments=[
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z'),
                '-unpause',
                '-urdf',
                '-param', 'robot_description',
                '-model', launch.substitutions.LaunchConfiguration('robot_name')
            ]
        )

    ])
