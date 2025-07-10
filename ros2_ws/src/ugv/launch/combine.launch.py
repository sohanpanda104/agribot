import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # Declare namespaces as arguments
    vehicle_ns_arg = DeclareLaunchArgument('vehicle_ns', default_value='vehicle')
    arm_ns_arg = DeclareLaunchArgument('arm_ns', default_value='arm')

    vehicle_ns = LaunchConfiguration('vehicle_ns')
    arm_ns = LaunchConfiguration('arm_ns')


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

    config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
    urdf_path = os.path.join(config_dir, 'combined.urdf')

    # Read the URDF content
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    ugv_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='combined_state_publisher',
        namespace='ugv',
        parameters=[{'robot_description': urdf_content}],
        output='screen'
        )

    return LaunchDescription([
        vehicle_ns_arg, arm_ns_arg,

        ackermann_launch,
        arm_launch,
        combiner_node,
        ugv_rsp
    ])
