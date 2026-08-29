from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Configurable launch arguments ===
    zmq_uri_arg = DeclareLaunchArgument(
        'zmq_uri',
        default_value='tcp://192.168.1.10:5555',
        description='ZMQ URI of the robot publisher'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # === Launch Configurations ===
    zmq_uri = LaunchConfiguration('zmq_uri')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === Paths ===
    turtlebot3_cartographer_launch_file = Path(
        '/opt/ros/humble/share/turtlebot3_cartographer/launch/cartographer.launch.py'
    )

    # === Nodes ===

    # ZMQ bridge node
    zmq_bridge_node = Node(
        package='groundbot',
        executable='topic_client',
        name='zmq_bridge',
        parameters=[{
            'zmq_uri': zmq_uri,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Static transform publisher: base_link -> laser
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    ),


    # Include Cartographer launch file with overridden config
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(turtlebot3_cartographer_launch_file)]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'configuration_directory': '/home/suraj/agribot/ros2_ws/src/groundbot/config',
            'configuration_basename': 'config.lua'
        }.items()
    )

    return LaunchDescription([
        zmq_uri_arg,
        use_sim_time_arg,
        zmq_bridge_node,
        static_tf_node,
        cartographer_launch,
    ])
