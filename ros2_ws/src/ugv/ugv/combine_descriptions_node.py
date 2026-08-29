import os
import yaml
import rclpy
import xacro
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


class URDFCombiner(Node):
    def __init__(self):
        super().__init__('combine_descriptions_node')
        self.get_logger().info("Starting combine_descriptions_node")

        self.declare_parameter('vehicle_ns', 'vehicle')
        self.declare_parameter('arm_ns', 'arm')
        self.vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value
        self.arm_ns = self.get_parameter('arm_ns').get_parameter_value().string_value
        self.get_logger().info(f"Using vehicle_ns='{self.vehicle_ns}' and arm_ns='{self.arm_ns}'")

        # Get URDFs
        self.vehicle_urdf = self.get_robot_description_from(f"/{self.vehicle_ns}/robot_state_publisher")
        self.arm_urdf = self.get_robot_description_from(f"/{self.arm_ns}/robot_state_publisher")

        if self.vehicle_urdf and self.arm_urdf:
            self.combine_urdf()

    def get_robot_description_from(self, target_node_name):
        client = self.create_client(GetParameters, f"{target_node_name}/get_parameters")
        self.get_logger().info(f"Waiting for parameter service from {target_node_name}...")

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {target_node_name}/get_parameters not available.")
            return None

        request = GetParameters.Request()
        request.names = ['robot_description']

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and len(future.result().values) > 0:
            value = future.result().values[0]
            if value.type == ParameterType.PARAMETER_STRING:
                self.get_logger().info(f"Received robot_description from {target_node_name}")
                return value.string_value
            else:
                self.get_logger().error(f"robot_description is not a string from {target_node_name}")
        else:
            self.get_logger().error(f"Failed to get robot_description from {target_node_name}")
        return None

    def combine_urdf(self):
        # Parse both URDFs
        vehicle_root = ET.fromstring(self.vehicle_urdf)
        arm_root = ET.fromstring(self.arm_urdf)

        # Remove unwanted links and joints from arm URDF
        for tag in ['link', 'joint']:
            for elem in list(arm_root):  # Only check direct children
                name = elem.attrib.get('name', '')
                if elem.tag == tag and name in ['world', 'base_link', 'base_to_world', 'dcmotor_to_base']:
                    arm_root.remove(elem)

        # Add custom joint to connect dcmotor (from arm) to base_link (from vehicle)
        new_joint = ET.Element('joint', {
            'name': 'dcmotor_to_base_link',
            'type': 'fixed'
        })

        parent_elem = ET.SubElement(new_joint, 'parent')
        parent_elem.attrib['link'] = 'base_link'
        child_elem = ET.SubElement(new_joint, 'child')
        child_elem.attrib['link'] = 'dcmotor'
        origin_elem = ET.SubElement(new_joint, 'origin')

        # Use the dimensions of the vehicle to place the arm appropriately
        vehicle_dir = get_package_share_directory('gazebo_ackermann_steering_vehicle')
        vehicle_config_path = os.path.join(vehicle_dir, 'config', 'parameters.yaml')

        with open(vehicle_config_path, 'r') as f:
            data = yaml.safe_load(f)

        params = data.get('/**', {}).get('ros__parameters', {})
        body_length = params.get('body_length')
        body_width = params.get('body_width')
        body_height = params.get('body_height')
        wheel_radius = params.get('wheel_radius')

        origin_elem.attrib['xyz'] = f"{(-body_length/2) - 0.1} {(-body_width/2) - 0.05} {body_height + wheel_radius + 0.05}"
        origin_elem.attrib['rpy'] = f"0 0 -1.57079632679"

        # Handle ros2_control tags - merge both systems
        vehicle_ros2_control = vehicle_root.find('ros2_control')
        arm_ros2_control = arm_root.find('ros2_control')
        
        # Remove the arm's ros2_control from arm_root as we'll merge it
        if arm_ros2_control is not None:
            arm_root.remove(arm_ros2_control)
        
        # Update vehicle ros2_control name to be more descriptive
        if vehicle_ros2_control is not None:
            vehicle_ros2_control.attrib['name'] = 'ugv_vehicle_control'
        
        # Create a new ros2_control block for the arm with updated name
        if arm_ros2_control is not None:
            arm_ros2_control.attrib['name'] = 'ugv_arm_control'
            # Add the modified arm ros2_control to vehicle_root
            vehicle_root.append(arm_ros2_control)

        # Handle gazebo plugins
        vehicle_gazebo = vehicle_root.find('gazebo')
        arm_gazebo = arm_root.find('gazebo')
        
        if vehicle_gazebo is not None and arm_gazebo is not None:
            # Update namespaces to /ugv for vehicle plugins
            for plugin in vehicle_gazebo.findall('plugin'):
                ros_elem = plugin.find('ros')
                if ros_elem is not None:
                    namespace_elem = ros_elem.find('namespace')
                    if namespace_elem is not None:
                        namespace_elem.text = '/ugv'
                    else:
                        # Add namespace if it doesn't exist
                        namespace_elem = ET.SubElement(ros_elem, 'namespace')
                        namespace_elem.text = '/ugv'
                    
                    # Update node names to be more specific
                    node_name_elem = ros_elem.find('node_name')
                    if node_name_elem is not None:
                        if 'vehicle_gz_control' in node_name_elem.text:
                            node_name_elem.text = 'ugv_vehicle_gz_control'
                else:
                    # Add ros element with namespace if it doesn't exist
                    ros_elem = ET.SubElement(plugin, 'ros')
                    namespace_elem = ET.SubElement(ros_elem, 'namespace')
                    namespace_elem.text = '/ugv'
                    node_name_elem = ET.SubElement(ros_elem, 'node_name')
                    if plugin.get('name') == 'vehicle_control':
                        node_name_elem.text = 'ugv_vehicle_gz_control'
            
            # Update arm gazebo plugin and merge
            for plugin in arm_gazebo.findall('plugin'):
                ros_elem = plugin.find('ros')
                if ros_elem is not None:
                    namespace_elem = ros_elem.find('namespace')
                    if namespace_elem is not None:
                        namespace_elem.text = '/ugv'
                    else:
                        # Add namespace if it doesn't exist
                        namespace_elem = ET.SubElement(ros_elem, 'namespace')
                        namespace_elem.text = '/ugv'
                    
                    # Update node names to be more specific
                    node_name_elem = ros_elem.find('node_name')
                    if node_name_elem is not None:
                        if 'arm_gz_control' in node_name_elem.text:
                            node_name_elem.text = 'ugv_arm_gz_control'
                else:
                    # Add ros element with namespace if it doesn't exist
                    ros_elem = ET.SubElement(plugin, 'ros')
                    namespace_elem = ET.SubElement(ros_elem, 'namespace')
                    namespace_elem.text = '/ugv'
                    node_name_elem = ET.SubElement(ros_elem, 'node_name')
                    if plugin.get('name') == 'arm_control':
                        node_name_elem.text = 'ugv_arm_gz_control'
                
                # Add arm plugins to vehicle gazebo block
                vehicle_gazebo.append(plugin)
            
            # Remove the separate arm gazebo block
            arm_root.remove(arm_gazebo)
        
        elif arm_gazebo is not None:
            # If only arm has gazebo block, update it and add to vehicle
            for plugin in arm_gazebo.findall('plugin'):
                ros_elem = plugin.find('ros')
                if ros_elem is not None:
                    namespace_elem = ros_elem.find('namespace')
                    if namespace_elem is not None:
                        namespace_elem.text = '/ugv'
                    else:
                        namespace_elem = ET.SubElement(ros_elem, 'namespace')
                        namespace_elem.text = '/ugv'
                    
                    node_name_elem = ros_elem.find('node_name')
                    if node_name_elem is not None:
                        if 'arm_gz_control' in node_name_elem.text:
                            node_name_elem.text = 'ugv_arm_gz_control'
                else:
                    ros_elem = ET.SubElement(plugin, 'ros')
                    namespace_elem = ET.SubElement(ros_elem, 'namespace')
                    namespace_elem.text = '/ugv'
            
            vehicle_root.append(arm_gazebo)
            arm_root.remove(arm_gazebo)

        # Handle other gazebo reference tags - update namespaces
        for gazebo_ref in vehicle_root.findall(".//gazebo[@reference]"):
            for plugin in gazebo_ref.findall('plugin'):
                robot_namespace = plugin.find('robotNamespace')
                if robot_namespace is not None:
                    robot_namespace.text = '/ugv'
        
        for gazebo_ref in arm_root.findall(".//gazebo[@reference]"):
            for plugin in gazebo_ref.findall('plugin'):
                robot_namespace = plugin.find('robotNamespace')
                if robot_namespace is not None:
                    robot_namespace.text = '/ugv'

        # Add all remaining elements from arm to vehicle
        for elem in list(arm_root):
            vehicle_root.append(elem)
        
        # Add the new joint connecting arm to vehicle
        vehicle_root.append(new_joint)

        # Convert to string with proper formatting
        combined_urdf = ET.tostring(vehicle_root, encoding='unicode')
        
        # Clean up any xacro artifacts or malformed XML
        combined_urdf = combined_urdf.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')
        
        # Pretty print the XML
        try:
            from xml.dom import minidom
            dom = minidom.parseString(combined_urdf)
            combined_urdf = dom.toprettyxml(indent="  ")
            # Remove empty lines and XML declaration duplicates
            lines = [line for line in combined_urdf.split('\n') if line.strip()]
            # Remove multiple XML declarations
            xml_decl_count = 0
            clean_lines = []
            for line in lines:
                if line.strip().startswith('<?xml'):
                    xml_decl_count += 1
                    if xml_decl_count == 1:
                        clean_lines.append(line)
                else:
                    clean_lines.append(line)
            combined_urdf = '\n'.join(clean_lines)
        except Exception:
            pass # TODO

        # Save the combined URDF
        config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
        os.makedirs(config_dir, exist_ok=True)
        save_path = os.path.join(config_dir, 'combined.urdf')

        with open(save_path, "w") as f:
            f.write(combined_urdf)

        self.get_logger().info(f"Saved combined_urdf to {save_path}")
        
        # Create a combined controller configuration
        self._create_combined_controller_config()

    def _create_combined_controller_config(self):
        """Create a combined controller configuration file for both vehicle and arm"""
        
        # Load existing configurations
        vehicle_dir = get_package_share_directory('gazebo_ackermann_steering_vehicle')
        arm_dir = get_package_share_directory('arm_assembly_description')
        
        vehicle_config_path = os.path.join(vehicle_dir, 'config', 'gz_ros2_control.yaml')
        arm_config_path = os.path.join(arm_dir, 'config', 'controller.yaml')
        
        combined_config = {
            'controller_manager': {
                'ros__parameters': {
                    'update_rate': 100,
                    'use_sim_time': True,
                    'joint_state_broadcaster': {
                        'type': 'joint_state_broadcaster/JointStateBroadcaster'
                    }
                }
            }
        }
        
        # Load and merge vehicle config
        if os.path.exists(vehicle_config_path):
            try:
                with open(vehicle_config_path, 'r') as f:
                    vehicle_config = yaml.safe_load(f)
                    if vehicle_config and 'controller_manager' in vehicle_config:
                        vehicle_params = vehicle_config['controller_manager'].get('ros__parameters', {})
                        combined_config['controller_manager']['ros__parameters'].update(vehicle_params)
            except Exception as e:
                self.get_logger().warn(f"Could not load vehicle config: {e}")
        
        # Load and merge arm config
        if os.path.exists(arm_config_path):
            try:
                with open(arm_config_path, 'r') as f:
                    arm_config = yaml.safe_load(f)
                    if arm_config and 'controller_manager' in arm_config:
                        arm_params = arm_config['controller_manager'].get('ros__parameters', {})
                        combined_config['controller_manager']['ros__parameters'].update(arm_params)
            except Exception as e:
                self.get_logger().warn(f"Could not load arm config: {e}")
        
        # Save combined config
        config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
        os.makedirs(config_dir, exist_ok=True)
        combined_config_path = os.path.join(config_dir, 'combined_controllers.yaml')
        
        with open(combined_config_path, 'w') as f:
            yaml.dump(combined_config, f, default_flow_style=False)
        
        self.get_logger().info(f"Saved combined controller config to {combined_config_path}")
        
        return combined_config_path


def main(args=None):
    rclpy.init(args=args)
    node = URDFCombiner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
