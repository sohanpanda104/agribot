from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


# Declare launch arguments for pose
x_arg = DeclareLaunchArgument('x', default_value='0.0')
y_arg = DeclareLaunchArgument('y', default_value='0.0')
z_arg = DeclareLaunchArgument('z', default_value='0.0')
roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

x = LaunchConfiguration('x')
y = LaunchConfiguration('y')
z = LaunchConfiguration('z')
roll = LaunchConfiguration('roll')
pitch = LaunchConfiguration('pitch')
yaw = LaunchConfiguration('yaw')


def generate_launch_description():
    share_dir = get_package_share_directory('arm_assembly_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'arm_assembly.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arm_assembly',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )

    arm_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen'
        )

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,

        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        joint_state_broadcaster_spawner,
        arm_controller
    ])
