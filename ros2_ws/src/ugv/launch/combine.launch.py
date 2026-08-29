import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare namespaces as arguments
    vehicle_ns_arg = DeclareLaunchArgument('vehicle_ns', default_value='vehicle')
    arm_ns_arg = DeclareLaunchArgument('arm_ns', default_value='arm')

    vehicle_ns = LaunchConfiguration('vehicle_ns')
    arm_ns = LaunchConfiguration('arm_ns')

    share_dir = get_package_share_directory('arm_assembly_description')
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
    urdf_path = os.path.join(config_dir, 'combined.urdf')

    # Include the ackermann vehicle launch
    ackermann_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ackermann_steering_vehicle'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={'description_name': vehicle_ns}.items()
    )

    # Include the arm assembly launch
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('arm_assembly_description'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={'description_name': arm_ns}.items()
    )

    combiner_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='ugv',
                executable='combine_descriptions_node',
                name='combine_descriptions_node',
                parameters=[
                    {'vehicle_ns': vehicle_ns, 'arm_ns': arm_ns}
                ],
                output='screen'
            )
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='ugv',
        parameters=[
            {
                'robot_description': Command([FindExecutable(name='xacro'), ' ', urdf_path]),
                'use_sim_time': True,
            },
        ],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='ugv',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace='ugv',
    )

    combined_description = TimerAction(
        period=1.5,  # Wait for combiner_node to finish
        actions=[
            robot_state_publisher,
            joint_state_publisher_node, # Only need one of the two jsp nodes
            # joint_state_publisher_gui_node,
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='ugv',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        vehicle_ns_arg, arm_ns_arg,

        ackermann_launch,
        arm_launch,
        combiner_node,
        combined_description,
        rviz_node,
    ])
