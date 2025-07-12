from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
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
    config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
    urdf_path = os.path.join(config_dir, 'combined.urdf')
    robot_urdf = xacro.process_file(urdf_path).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='ugv',
        parameters=[
            {'robot_description': robot_urdf}
        ],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui', # non gui did not work idk why
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings=[
            ('robot_description', '/ugv/robot_description'),
            ('joint_states', '/ugv/joint_states')
        ],
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
    
    # Spawn entity with proper timing
    urdf_spawn_node = TimerAction(
        period=3.0,  # Increased delay to ensure Gazebo is ready
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace='ugv',
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
            ),
        ]
    )

    # Load controllers with proper service checking and longer delay
    spawn_controllers = TimerAction(
        period=8.0,  # Increased delay to ensure controller manager is ready
        actions=[
            # First, wait for the controller manager service to be available
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/ugv/controller_manager/list_controllers', 
                     'controller_manager_msgs/srv/ListControllers', '{}'],
                output='screen',
                shell=True
            ),
            # Load joint_state_broadcaster first (required by other controllers)
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                     'joint_state_broadcaster', '--controller-manager', '/ugv/controller_manager'],
                output='screen'
            ),
        ]
    )
    
    # Load remaining controllers after joint_state_broadcaster
    load_remaining_controllers = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                     'vehicle_controller', '--controller-manager', '/ugv/controller_manager'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                     'arm_controller', '--controller-manager', '/ugv/controller_manager'],
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,

        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        # spawn_controllers,
        # load_remaining_controllers
    ])
