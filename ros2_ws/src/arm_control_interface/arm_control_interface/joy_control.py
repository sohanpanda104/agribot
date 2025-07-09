import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
import time
import threading
import os
import yaml
from ament_index_python.packages import get_package_share_directory


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

        self.saved_pose_filename = self.get_saved_pose_path()

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
        
    def get_saved_pose_path(self):
        config_dir = os.path.join(os.path.expanduser('~'), '.arm_control_interface')
        os.makedirs(config_dir, exist_ok=True)
        return os.path.join(config_dir, 'saved_positions.yaml')


    def save_named_pose(self, pose_name):
        data = {}

        if os.path.exists(self.saved_pose_filename):
            with open(self.saved_pose_filename, 'r') as f:
                data = yaml.safe_load(f) or {}

        data[pose_name] = self.current_positions

        with open(self.saved_pose_filename, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Saved current position as pose '{pose_name}'")

    def load_named_pose(self, pose_name):
        try:
            with open(self.saved_pose_filename, 'r') as f:
                data = yaml.safe_load(f) or {}
                if pose_name in data:
                    for name in self.joint_names:
                        if name in data[pose_name]:
                            self.target_positions[name] = data[pose_name][name]
                    self.publish_trajectory()
                    self.get_logger().info(f"Loaded pose '{pose_name}'")
                else:
                    self.get_logger().warn(f"Pose '{pose_name}' not found")
        except Exception as e:
            self.get_logger().warn(f"Failed to load pose '{pose_name}': {e}")

    def prompt_string(self, prompt):
        curses.echo(False)
        max_y = len(self.joint_names) + 5

        # Clear previous text displaying prompt
        self.stdscr.move(max_y, 0)
        self.stdscr.clrtoeol()

        self.stdscr.addstr(max_y, 2, prompt)
        self.stdscr.clrtoeol()
        self.stdscr.refresh()

        input_str = ""
        x = len(prompt) + 2

        while True:
            ch = self.stdscr.getch()
            if ch in (curses.KEY_ENTER, 10, 13, 27):  # Enter key and 27 for ESC
                self.stdscr.move(max_y + 1, 0)
                break
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                if len(input_str) > 0:
                    input_str = input_str[:-1]
                    x -= 1
                    self.stdscr.move(max_y, x)
                    self.stdscr.delch()
            elif 32 <= ch <= 126:  # Printable characters
                input_str += chr(ch)
                self.stdscr.addch(max_y, x, ch)
                x += 1

            self.stdscr.refresh()

        return input_str.strip()

    def control_loop(self):
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        selected_joint = 0
        step = 0.05

        try:
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

                if key == -1:
                    continue
                if key == ord('q'):
                    self.stop()
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
                elif key == ord('s'):
                    pose_name = self.prompt_string("Enter name to save pose as: ")
                    self.save_named_pose(pose_name)
                elif key == ord('l'):
                    pose_name = self.prompt_string("Enter pose name to load: ")
                    self.load_named_pose(pose_name)

                self.stdscr.refresh()
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().info(f"Exception: {e}")
            self.stop()

    def stop(self):
        self.get_logger().info(f"Stopped")
        self.running = False
        if threading.current_thread() != self.control_thread:
            self.control_thread.join()

def main(stdscr):
    rclpy.init()
    node = ArmControlNode(stdscr)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected. Exiting...")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_with_curses():
    import signal
    signal.signal(signal.SIGINT, lambda sig, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    curses.wrapper(main)

if __name__ == '__main__':
    run_with_curses()

