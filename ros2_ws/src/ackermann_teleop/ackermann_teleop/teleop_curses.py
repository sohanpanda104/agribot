#!/usr/bin/env python3

import curses
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

MAX_STEERING = 0.6
MAX_VELOCITY = 3.0
STEERING_STEP = 0.05
VELOCITY_STEP = 0.1


class TeleopCursesNode(Node):
    def __init__(self):
        super().__init__('teleop_curses_node')
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)

        self.steering = 0.0
        self.velocity = 0.0

        curses.wrapper(self.run)

    def run(self, stdscr):
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "Teleop Control with Arrow Keys")
        stdscr.addstr(1, 0, "UP/DOWN: Velocity | LEFT/RIGHT: Steering")
        stdscr.addstr(2, 0, "SPACE: Stop | q: Quit")

        while rclpy.ok():
            key = stdscr.getch()
            if key != -1:
                if key == curses.KEY_UP:
                    self.velocity = min(MAX_VELOCITY, self.velocity + VELOCITY_STEP)
                elif key == curses.KEY_DOWN:
                    self.velocity = max(-MAX_VELOCITY, self.velocity - VELOCITY_STEP)
                elif key == curses.KEY_LEFT:
                    self.steering = max(-MAX_STEERING, self.steering - STEERING_STEP)
                elif key == curses.KEY_RIGHT:
                    self.steering = min(MAX_STEERING, self.steering + STEERING_STEP)
                elif key == ord(' '):  # spacebar
                    self.steering = 0.0
                    self.velocity = 0.0
                elif key == ord('q'):
                    break

            # Publish messages
            self.publish_cmds()
            self.display_status(stdscr)
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_cmds(self):
        self.steering_pub.publish(Float64(data=self.steering))
        self.velocity_pub.publish(Float64(data=self.velocity))

    def display_status(self, stdscr):
        stdscr.addstr(4, 0, f"Steering: {self.steering:.2f}")
        stdscr.addstr(5, 0, f"Velocity: {self.velocity:.2f}")
        stdscr.refresh()


def main(args=None):
    rclpy.init(args=args)
    try:
        TeleopCursesNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
