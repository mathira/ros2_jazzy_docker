#!/usr/bin/env python3
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# Key -> (linear_x, angular_z)
KEY_BINDINGS = {
    'w': (2.0, 0.0),
    '\x1b[A': (2.0, 0.0),   # up arrow
    's': (-2.0, 0.0),
    '\x1b[B': (-2.0, 0.0),  # down arrow
    'a': (0.0, 2.0),
    '\x1b[D': (0.0, 2.0),   # left arrow
    'd': (0.0, -2.0),
    '\x1b[C': (0.0, -2.0),  # right arrow
    'q': (0.0, 0.0),        # stop
    ' ': (0.0, 0.0),        # stop
}

USAGE = """
Controls:
  w / UP      move forward
  s / DOWN    move backward
  a / LEFT    turn left
  d / RIGHT   turn right
  q / SPACE   stop
  Ctrl-C      quit
Publishing to /turtle1/cmd_vel (Twist)
"""


class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('teleop_turtle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.movement = [0.0, 0.0]
        self.lock = threading.Lock()
        self.get_logger().info('TeleopTurtle started')

    def timer_callback(self):
        with self.lock:
            lin, ang = self.movement
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.publisher.publish(msg)
        self.get_logger().info(f"Sending: linear={lin}, angular={ang}")

    def set_movement(self, lin, ang):
        with self.lock:
            self.movement = [lin, ang]


def read_keys(teleop):
    """Read keyboard input in a background thread and update the state."""
    fd = sys.stdin.fileno()
    old = tty.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                # handle escape sequences (arrows) by reading the next bytes
                if key == '\x1b':
                    seq = sys.stdin.read(2)
                    key = '\x1b' + seq
                if key == 'q':
                    raise KeyboardInterrupt
                if key in KEY_BINDINGS:
                    lin, ang = KEY_BINDINGS[key]
                    teleop.set_movement(lin, ang)
                    print(f'\r{key!r} -> linear={lin}, angular={ang}    ', end='')
                    sys.stdout.flush()
    finally:
        tty.tcsetattr(fd, tty.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtle()
    print(USAGE)
    t = threading.Thread(target=read_keys, args=(node,), daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_movement(0.0, 0.0)
        node.get_logger().info('Stopping...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()