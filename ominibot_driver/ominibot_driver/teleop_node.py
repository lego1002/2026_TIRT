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
import time
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

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


def slew(current, target, accel, decel, dt):
    """Step `current` toward `target`, limited to accel (speeding up) / decel (slowing).

    A reversal first brakes through zero at `decel`, then speeds up at `accel`.
    """
    if current * target < 0.0 or abs(target) < abs(current):
        step = decel * dt
        goal = 0.0 if current * target < 0.0 else target
    else:
        step = accel * dt
        goal = target
    if abs(goal - current) <= step:
        return goal
    return current + step if goal > current else current - step


class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop')
        self.declare_parameter('linear_speed', 0.2)    # m/s at full deflection
        self.declare_parameter('angular_speed', 0.8)   # rad/s at full deflection
        self.declare_parameter('linear_max', 1.5)
        self.declare_parameter('angular_max', 4.0)
        self.declare_parameter('publish_rate', 20.0)   # Hz, also watchdog re-send
        # Ramp limits. Without them a key press asks for 0 -> full speed within one
        # 50 ms message (0.2 m/s = 4 m/s^2), which breaks the mecanum rollers loose.
        # The wheels then spin freely, /odom counts distance the robot never travelled
        # (the board reports wheel speed, so it cannot see the slip), and the map shifts.
        self.declare_parameter('linear_accel', 0.6)    # m/s^2
        self.declare_parameter('linear_decel', 1.0)    # m/s^2, ~2 cm to stop from 0.2 m/s
        self.declare_parameter('angular_accel', 2.0)   # rad/s^2
        self.declare_parameter('angular_decel', 3.0)   # rad/s^2

        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value
        self.lin_max = self.get_parameter('linear_max').value
        self.ang_max = self.get_parameter('angular_max').value
        self.rate = self.get_parameter('publish_rate').value
        self.lin_acc = self.get_parameter('linear_accel').value
        self.lin_dec = self.get_parameter('linear_decel').value
        self.ang_acc = self.get_parameter('angular_accel').value
        self.ang_dec = self.get_parameter('angular_decel').value

        # BEST_EFFORT depth-1, matching ominibot_driver's cmd_vel subscription. A
        # teleop stream is "latest value wins": if a sample is lost, the next one is
        # already on its way (we re-publish every loop), so retransmitting a stale
        # Twist -- and head-of-line blocking the fresh one behind it -- is strictly
        # worse. This matters when the link is WiFi; harmless when it is loopback,
        # which is the normal case now that teleop runs on the Pi itself.
        self.pub = self.create_publisher(
            Twist, 'cmd_vel',
            QoSProfile(depth=1,
                       history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.tx = self.ty = 0.0   # translation direction, in {-1, 0, 1}
        self.tz = 0.0             # rotation direction, in {-1, 0, 1}
        self.vx = self.vy = self.wz = 0.0   # ramped velocity actually published

    def _twist(self, dt):
        norm = DIAG if (self.tx and self.ty) else 1.0
        self.vx = slew(self.vx, self.tx * self.lin * norm, self.lin_acc, self.lin_dec, dt)
        self.vy = slew(self.vy, self.ty * self.lin * norm, self.lin_acc, self.lin_dec, dt)
        self.wz = slew(self.wz, self.tz * self.ang, self.ang_acc, self.ang_dec, dt)
        t = Twist()
        t.linear.x = self.vx
        t.linear.y = self.vy
        t.angular.z = self.wz
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
        last = time.monotonic()
        while rclpy.ok():
            key = get_key(settings, timeout)
            if key and not self._apply(key):
                break
            now = time.monotonic()
            # A key press returns early, so dt varies; the cap stops a stalled loop
            # from turning into one big velocity jump.
            self.pub.publish(self._twist(min(now - last, 0.1)))
            last = now


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
