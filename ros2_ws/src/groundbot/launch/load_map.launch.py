from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/suraj/agribot/ros2_ws/src/groundbot/maps/on_table.yaml',
                'frame_id': 'map',
                'topic_name': 'map',
                'use_sim_time': False
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
              'use_sim_time': False,
              'autostart': True,
              'node_names': ['map_server']
            }]
        )
    ])
