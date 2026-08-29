import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('arm_assembly_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'arm_assembly.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()


    # Launch Arguments
    state_publisher_rename_arg = DeclareLaunchArgument(
        name='description_name',
        default_value='arm_assembly'
    )

    # Launch Config 
    state_publisher_rename = LaunchConfiguration('description_name')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=state_publisher_rename,
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    return LaunchDescription([
        # Launch Args
        state_publisher_rename_arg,

        # Nodes
        robot_state_publisher_node,
    ])
