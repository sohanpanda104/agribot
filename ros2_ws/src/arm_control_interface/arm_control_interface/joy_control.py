import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
import time
import threading


class ArmControlNode(Node):
    def __init__(self, stdscr):
        super().__init__('arm_control_node')
        self.stdscr = stdscr
        self.joint_names = [
            'link1_to_base',
            'servo_horn_to_servo',
            'camera_roll',
            'camera_pitch'
        ]

        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = self.current_positions.copy()

        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.target_positions[name] for name in self.joint_names]
        point.time_from_start.sec = 1

        traj.points.append(point)
        self.publisher.publish(traj)

    def control_loop(self):
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        selected_joint = 0
        step = 0.05

        while self.running:
            self.stdscr.erase()
            self.stdscr.addstr(0, 2, "Use UP/DOWN to select joint, LEFT/RIGHT to move it")
            self.stdscr.addstr(1, 2, "Press 'q' to quit")

            for i, name in enumerate(self.joint_names):
                pos = self.current_positions[name]
                target = self.target_positions[name]
                bar = "=" * int((pos + 3.14) / 6.28 * 20)  # Simple slider from -π to +π
                self.stdscr.addstr(3 + i, 2, f"[{'*' if i == selected_joint else ' '}] {name:25}: {pos:+.2f} -> {target:+.2f} | {bar}")

            key = self.stdscr.getch()

            if key == ord('q'):
                self.running = False
                break
            elif key == curses.KEY_UP:
                selected_joint = (selected_joint - 1) % len(self.joint_names)
            elif key == curses.KEY_DOWN:
                selected_joint = (selected_joint + 1) % len(self.joint_names)
            elif key == curses.KEY_RIGHT:
                name = self.joint_names[selected_joint]
                self.target_positions[name] += step
                self.publish_trajectory()
            elif key == curses.KEY_LEFT:
                name = self.joint_names[selected_joint]
                self.target_positions[name] -= step
                self.publish_trajectory()

            self.stdscr.refresh()
            time.sleep(0.05)

    def stop(self):
        self.running = False
        self.control_thread.join()

def main(stdscr):
    rclpy.init()
    node = ArmControlNode(stdscr)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

def run_with_curses():
    curses.wrapper(main)

if __name__ == '__main__':
    run_with_curses
