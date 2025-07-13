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

    # Apply the gz_ros2_control.yaml to ackermann_drive.yaml replacement
    xml_content = robot_description.toxml()
    xml_content = xml_content.replace("gz_ros2_control.yaml", "ackermann_drive.yaml")
        
    return xml_content


def generate_launch_description():
    pkg_ugv = get_package_share_directory('ugv')
    urdf_file = os.path.join(pkg_ugv, 'urdf', 'ugv.xacro')
    vehicle_params_file = os.path.join(pkg_ugv, 'config', 'parameters.yaml')
    ctrl_yaml = os.path.join(pkg_ugv, 'config', 'ackermann_drive.yaml')

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_ugv, 'config', 'nav2_params.yaml')
    bt_file = os.path.join(nav2_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    robot_description = load_robot_description(urdf_file, vehicle_params_file)

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


    ekf_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(pkg_ugv, 'config', 'ekf.yaml')]
            )
        ]
    )

    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                    'params_file': nav2_params_file,
                    'bt_xml_file': bt_file,
                    'map': os.path.join(pkg_ugv, 'maps', '5x5.yaml')
                }.items()
            )
        ]
    )

    spawn_controllers = TimerAction(
        period=6.0,
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
                arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
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

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,

        gazebo_server,
        gazebo_client,

        robot_state_publisher_node,
        urdf_spawn_node,

        control_manager,
        ekf_node,
        # nav2,
        spawn_controllers,
    ])
