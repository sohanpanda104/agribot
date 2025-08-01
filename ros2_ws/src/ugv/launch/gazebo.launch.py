from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro
import yaml


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


def load_robot_description(robot_description_path, vehicle_params_path):
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )

    return robot_description.toxml()


def generate_launch_description():
    pkg_ugv = get_package_share_directory('ugv')
    urdf_file = os.path.join(pkg_ugv, 'urdf', 'ugv.xacro')
    params_file = os.path.join(pkg_ugv, 'config', 'parameters.yaml')
    ctrl_yaml = os.path.join(pkg_ugv, 'config', 'gz_ros2_control.yaml')

    robot_description = load_robot_description(urdf_file, params_file)

    # Gazebo server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        )
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # URDF spawn node
    urdf_spawn_node = TimerAction(
        period=2.0,
        actions = [
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'ugv',
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
        ]
    )

    control_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace='',
                output='screen',
                name='controller_manager',
                parameters=[ctrl_yaml],
                remappings=[
                    ('robot_description', '/robot_description')
                ]
            )
        ]
    )


    spawn_controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['forward_velocity_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,

        gazebo_server,
        gazebo_client,

        robot_state_publisher_node,
        urdf_spawn_node,

        control_manager,
        spawn_controllers,
        vehicle_controller_node
    ])
