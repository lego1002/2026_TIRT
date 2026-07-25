"""Keyboard teleop for the mecanum base -- no modifier keys required.

Publishes geometry_msgs/Twist on /cmd_vel, which ominibot_driver maps to
robot_speed(lx, ly, az). Designed for a holonomic (mecanum) base, so translation
is a first-class motion on the numeric 3x3 keypad instead of hidden behind Shift
like the stock teleop_twist_keyboard.

    u  i  o      forward-left   forward   forward-right    a : rotate left  (+z, CCW)
    j  k  l      strafe-left    STOP      strafe-right     d : rotate right (-z, CW)
    m  ,  .      back-left      back      back-right       w / s : linear speed +/-
                                                           q / e : turn speed  +/-
    k / space : full stop        Ctrl-C : quit

The 3x3 pad and the turn keys are mutually exclusive: pressing a pad key zeroes
rotation, pressing a/d zeroes translation. This keeps "pad = pure translation,
a/d = pure spin" true to what each key is labelled, rather than accidentally
arcing. (Combine them later by not zeroing the other axis in _read_loop.)

The current command is re-published on every loop (>=10 Hz), which keeps the
driver's cmd_vel watchdog fed so latched motion doesn't stop after cmd_vel_timeout.

If a/d feel swapped relative to the real robot, do NOT flip them here -- set
`angular_z_sign:=-1.0` on ominibot_driver instead, so /odom stays consistent with
the command (see driver_node.py).
"""

import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 3x3 pad -> (x, y) translation direction, REP-103 (x forward, y left).
MOVE_BINDINGS = {
    'u': (1, 1), 'i': (1, 0), 'o': (1, -1),
    'j': (0, 1), 'k': (0, 0), 'l': (0, -1),
    'm': (-1, 1), ',': (-1, 0), '.': (-1, -1),
}
# turn keys -> angular z direction (CCW positive).
TURN_BINDINGS = {'a': 1, 'd': -1}
# speed keys -> multiplicative step (linear and turn scale independently).
LIN_SPEED_BINDINGS = {'w': 1.1, 's': 0.9}
ANG_SPEED_BINDINGS = {'q': 1.1, 'e': 0.9}
STOP_KEYS = {'k', ' '}

# Straight-line resultant speed for diagonals (both axes at once).
DIAG = 1.0 / math.sqrt(2.0)

HELP = __doc__


def get_key(settings, timeout):
    """Read one keystroke in cbreak mode, or '' if none within `timeout` s."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop')
        self.declare_parameter('linear_speed', 0.6)    # m/s at full deflection
        self.declare_parameter('angular_speed', 1.5)   # rad/s at full deflection
        self.declare_parameter('linear_max', 1.5)
        self.declare_parameter('angular_max', 4.0)
        self.declare_parameter('publish_rate', 20.0)   # Hz, also watchdog re-send

        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value
        self.lin_max = self.get_parameter('linear_max').value
        self.ang_max = self.get_parameter('angular_max').value
        self.rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tx = self.ty = 0.0   # translation direction, in {-1, 0, 1}
        self.tz = 0.0             # rotation direction, in {-1, 0, 1}

    def _twist(self):
        t = Twist()
        norm = DIAG if (self.tx and self.ty) else 1.0
        t.linear.x = self.tx * self.lin * norm
        t.linear.y = self.ty * self.lin * norm
        t.angular.z = self.tz * self.ang
        return t

    def _apply(self, key):
        """Update state from a key. Returns False to request quit."""
        if key == '\x03':  # Ctrl-C
            return False
        if key in STOP_KEYS:
            self.tx = self.ty = self.tz = 0.0
        elif key in MOVE_BINDINGS:
            self.tx, self.ty = MOVE_BINDINGS[key]
            self.tz = 0.0                      # pad = pure translation
        elif key in TURN_BINDINGS:
            self.tz = TURN_BINDINGS[key]
            self.tx = self.ty = 0.0            # a/d = pure spin
        elif key in LIN_SPEED_BINDINGS:
            self.lin = min(self.lin * LIN_SPEED_BINDINGS[key], self.lin_max)
            self.get_logger().info(f'linear speed: {self.lin:.2f} m/s')
        elif key in ANG_SPEED_BINDINGS:
            self.ang = min(self.ang * ANG_SPEED_BINDINGS[key], self.ang_max)
            self.get_logger().info(f'turn speed: {self.ang:.2f} rad/s')
        return True

    def run(self, settings):
        print(HELP)
        timeout = 1.0 / self.rate
        while rclpy.ok():
            key = get_key(settings, timeout)
            if key and not self._apply(key):
                break
            self.pub.publish(self._twist())


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = MecanumTeleop()
    try:
        node.run(settings)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())  # stop the base on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
