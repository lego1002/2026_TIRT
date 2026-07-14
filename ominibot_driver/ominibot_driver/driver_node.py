"""ROS 2 driver node for the OminiBotHV mecanum controller board.

  /cmd_vel (geometry_msgs/Twist)  -->  robot_speed(lx, ly, az) over serial
  board feedback frame            -->  /odom (nav_msgs/Odometry) + odom->base_link TF
                                       /imu  (sensor_msgs/Imu, orientation only)

Odometry is dead-reckoned from the board's reported body velocities (smooth and
locally consistent -- exactly what slam_toolbox wants from the `odom` frame). The IMU
quaternion is published separately on /imu for optional downstream fusion; it is not
mixed into the odom TF so the odom frame stays free of absolute-heading jumps.

A watchdog re-sends the last /cmd_vel at a fixed rate and commands zero if no command
has arrived within cmd_vel_timeout, so the base stops if the teleop link drops.
"""

import math
import threading

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from ominibot_driver.ominibot_hv import OminiBotHV


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OminiBotDriver(Node):
    def __init__(self):
        super().__init__('ominibot_driver')

        # --- parameters -------------------------------------------------------
        self.declare_parameter('port', '/dev/ominibot')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('cmd_rate', 20.0)       # Hz, command re-send rate
        self.declare_parameter('cmd_vel_timeout', 0.5)  # s, stop if no cmd_vel
        # Axis signs to reconcile the board's frame with REP-103 (x fwd, y left,
        # z ccw). Applied to BOTH the outgoing command and the odom feedback so
        # they stay consistent. Set to -1.0 to flip an axis (e.g. "forward is
        # backward" -> linear_x_sign:=-1.0).
        self.declare_parameter('linear_x_sign', 1.0)
        self.declare_parameter('linear_y_sign', 1.0)
        self.declare_parameter('angular_z_sign', 1.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.publish_imu = self.get_parameter('publish_imu').value
        cmd_rate = self.get_parameter('cmd_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.sx = self.get_parameter('linear_x_sign').value
        self.sy = self.get_parameter('linear_y_sign').value
        self.sz = self.get_parameter('angular_z_sign').value

        # --- serial board -----------------------------------------------------
        self.get_logger().info(f'Opening OminiBotHV on {port} @ {baud}')
        self.bot = OminiBotHV(port=port, baud=baud)

        # --- state ------------------------------------------------------------
        self._cmd_lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)             # (lx, ly, az)
        self._last_cmd_time = self.get_clock().now()
        self.x = self.y = self.theta = 0.0
        self._last_odom_time = None

        # --- ROS interfaces ---------------------------------------------------
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Watchdog / command re-send timer (runs in the executor thread).
        self.create_timer(1.0 / cmd_rate, self._send_cmd)

        # Feedback read loop (blocking serial reads -> own thread).
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    # -- /cmd_vel ------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        with self._cmd_lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _send_cmd(self):
        stale = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        with self._cmd_lock:
            lx, ly, az = self._cmd
        if stale > self.cmd_vel_timeout:
            lx = ly = az = 0.0
        try:
            self.bot.robot_speed(self.sx * lx, self.sy * ly, self.sz * az)
        except Exception as exc:  # noqa: BLE001 - keep node alive on serial hiccup
            self.get_logger().warn(f'robot_speed write failed: {exc}')

    # -- feedback -> odom / imu ---------------------------------------------
    def _read_loop(self):
        while self._running and rclpy.ok():
            try:
                data = self.bot.read_feedback()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'read_feedback failed: {exc}')
                continue
            if data is None:
                continue
            if not (self._running and rclpy.ok()):
                break
            try:
                self._publish(data)
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    break  # context torn down mid-publish during shutdown
                self.get_logger().warn(f'publish failed: {exc}')

    def _publish(self, data):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Board feedback -> REP-103 body velocities (same axis signs as the command).
        lx = self.sx * data['lx']
        ly = self.sy * data['ly']
        az = self.sz * data['az']

        # Dead-reckon odom from body velocities.
        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:  # ignore first sample and pathological gaps
                mid = self.theta + 0.5 * az * dt
                self.x += (lx * math.cos(mid) - ly * math.sin(mid)) * dt
                self.y += (lx * math.sin(mid) + ly * math.cos(mid)) * dt
                self.theta = math.atan2(math.sin(self.theta + az * dt),
                                        math.cos(self.theta + az * dt))
        self._last_odom_time = now

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = lx
        odom.twist.twist.linear.y = ly
        odom.twist.twist.angular.z = az
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.02
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(tf)

        if self.publish_imu:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.imu_frame
            n = math.sqrt(data['qw']**2 + data['qx']**2
                          + data['qy']**2 + data['qz']**2)
            if n > 1e-6:
                imu.orientation.w = data['qw'] / n
                imu.orientation.x = data['qx'] / n
                imu.orientation.y = data['qy'] / n
                imu.orientation.z = data['qz'] / n
            imu.angular_velocity.z = az
            # accel/gyro layout unverified -> mark as unavailable per REP-145.
            imu.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(imu)

    def destroy_node(self):
        self._running = False
        if self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        try:
            self.bot.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OminiBotDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
