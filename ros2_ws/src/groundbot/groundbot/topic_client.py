#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
import zmq
import pickle

class TopicClient(Node):
    def __init__(self):
        super().__init__('topic_client')

        self.declare_parameter('zmq_uri', 'tcp://localhost:5555')
        zmq_uri = self.get_parameter('zmq_uri').get_parameter_value().string_value

        # --- ZMQ setup ---
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(zmq_uri)          # e.g. "tcp://192.168.1.10:5555"
        self.socket.setsockopt_string(zmq.SUBSCRIBE, 'scan')
        self.socket.setsockopt_string(zmq.SUBSCRIBE, 'odom')

        # --- ROS2 publishers ---
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(f'Connected to ZMQ at {zmq_uri}, publishing /scan and /odom locally')

        # Poller allows non-blocking check for messages
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        # Timer to drive incoming checks
        self.create_timer(0.01, self.check_zmq)

    def check_zmq(self):
        """Check for any incoming ZMQ multipart messages and dispatch."""
        socks = dict(self.poller.poll(0))
        if self.socket in socks:
            topic, payload = self.socket.recv_multipart()
            data = pickle.loads(payload)
            if topic == b'scan':
                self._handle_scan(data)
            elif topic == b'odom':
                self._handle_odom(data)

    def _handle_scan(self, d: dict):
        msg = LaserScan()
        # Header
        msg.header.stamp = Time(sec=d['header']['stamp']['sec'],
                                nanosec=d['header']['stamp']['nanosec'])
        msg.header.frame_id = d['header']['frame_id']
        # Scan params
        msg.angle_min = d['angle_min']
        msg.angle_max = d['angle_max']
        msg.angle_increment = d['angle_increment']
        msg.time_increment = d['time_increment']
        msg.scan_time = d['scan_time']
        msg.range_min = d['range_min']
        msg.range_max = d['range_max']
        msg.ranges = list(d['ranges'])
        msg.intensities = list(d.get('intensities', []))
        self.scan_pub.publish(msg)
        self.get_logger().debug('Published /scan')

    def _handle_odom(self, d: dict):
        msg = Odometry()
        # Header
        msg.header.stamp = Time(sec=d['header']['stamp']['sec'],
                                nanosec=d['header']['stamp']['nanosec'])
        msg.header.frame_id = d['header']['frame_id']
        msg.child_frame_id = d['child_frame_id']

        # Pose
        p = d['pose']['pose']['position']
        o = d['pose']['pose']['orientation']
        msg.pose = PoseWithCovariance()
        msg.pose.pose.position.x = p['x']
        msg.pose.pose.position.y = p['y']
        msg.pose.pose.position.z = p['z']
        msg.pose.pose.orientation.x = o['x']
        msg.pose.pose.orientation.y = o['y']
        msg.pose.pose.orientation.z = o['z']
        msg.pose.pose.orientation.w = o['w']

        # Twist
        tl = d['twist']['twist']['linear']
        ta = d['twist']['twist']['angular']
        msg.twist = TwistWithCovariance()
        msg.twist.twist.linear.x = tl['x']
        msg.twist.twist.linear.y = tl['y']
        msg.twist.twist.linear.z = tl['z']
        msg.twist.twist.angular.x = ta['x']
        msg.twist.twist.angular.y = ta['y']
        msg.twist.twist.angular.z = ta['z']

        self.odom_pub.publish(msg)
        self.get_logger().debug('Published /odom')

        t = TransformStamped()
        t.header.stamp = Time(sec=d['header']['stamp']['sec'],
                                nanosec=d['header']['stamp']['nanosec'])
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Published odom base_link transform')

def main(args=None):
    rclpy.init(args=args)
    node = TopicClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

