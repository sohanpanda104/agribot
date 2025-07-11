import os
import yaml
import rclpy
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
            self.combine_and_print_urdfs()

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

    def combine_and_print_urdfs(self):
        # Parse both URDFs
        vehicle_root = ET.fromstring(self.vehicle_urdf)
        arm_root = ET.fromstring(self.arm_urdf)

        # Remove unwanted links and joints from arm URDF
        for tag in ['link', 'joint']:
            for elem in arm_root.findall(tag):
                name = elem.attrib.get('name', '')
                if name in ['world', 'base_link', 'base_to_world', 'dcmotor_to_base']:
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

        # Load the YAML file
        with open(vehicle_config_path, 'r') as f:
            data = yaml.safe_load(f)

        # Extract parameters
        params = data.get('/**', {}).get('ros__parameters', {})

        # Get specific values
        body_length = params.get('body_length')
        body_width = params.get('body_width')
        body_height = params.get('body_height')
        wheel_radius = params.get('wheel_radius')

        origin_elem.attrib['xyz'] = f"{(-body_length/2) - 0.1} {(-body_width/2) - 0.05} {body_height + wheel_radius + 0.05}"
        origin_elem.attrib['rpy'] = f"0 0 -1.57079632679"

        # Append the cleaned arm links and joints to the vehicle root
        for elem in arm_root:
            vehicle_root.append(elem)
        vehicle_root.append(new_joint)

        # Print combined URDF
        combined_urdf = ET.tostring(vehicle_root, encoding='unicode')

        # Optionally write to file
        config_dir = os.path.join(os.path.expanduser('~'), '.ugv')
        os.makedirs(config_dir, exist_ok=True)
        save_path = os.path.join(config_dir, 'combined.urdf')

        with open(save_path, "w") as f:
            f.write(combined_urdf)
        self.get_logger().info(f"Saved combined_urdf to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = URDFCombiner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
