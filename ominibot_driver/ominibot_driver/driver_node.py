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
        # Board is wired to the Pi's GPIO UART (TX/RX on pins 8/10 -> ttyAMA0).
        # /dev/serial0 is the Pi's stable alias for the primary GPIO UART; it
        # replaced the old /dev/ominibot USB (FTDI) symlink after the board's
        # USB terminal broke. Override with the `port` param if wired elsewhere.
        self.declare_parameter('port', '/dev/serial0')
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
        self.declare_parameter('linear_y_sign', -1.0)  # board strafes opposite REP-103; verified on hardware
        self.declare_parameter('angular_z_sign', -1.0)  # board spins opposite REP-103; verified on hardware
        # Robot geometry (mm) sent to the board so its wheel-rev<->body-velocity
        # conversion matches the real hardware. A wrong wheel_diameter scales the
        # reported velocity (hence odom distance) by real/assumed, which slam_toolbox
        # then has to fight -- the #1 cause of map drift. wheel/axle spacing scale
        # the yaw term for the mecanum mixer.
        self.declare_parameter('wheel_diameter_mm', 48)   # actual wheel is 48mm
        self.declare_parameter('wheel_space_mm', 115)     # left-right wheel spacing (measured on real robot)
        self.declare_parameter('axle_space_mm', 96)       # front-back axle spacing (measured on real robot)
        # Motor/encoder scale sent to the board. encoder_ppr (pulses per motor
        # rev) and gear_ratio multiply with wheel_diameter into a single scale on
        # reported velocity/odom distance. The CircusPi factory defaults (165/55)
        # are for a *different* motor and grossly over-report odom, so these MUST
        # be matched to the real N20 motor. See SLAM_learning_note.md §7.
        self.declare_parameter('encoder_ppr', 165)
        self.declare_parameter('gear_ratio', 55)
        # Closed-loop PID gains written to the board. Factory-tuned for the
        # CircusPi 1:55 chassis; on a mismatched (lighter) motor these can
        # overshoot/oscillate -- a likely cause of chassis vibration. Exposed so
        # they can be lowered from the command line without a rebuild.
        self.declare_parameter('pos_kp', 3000)
        self.declare_parameter('pos_ki', 1050)
        self.declare_parameter('pos_kd', 0)
        self.declare_parameter('vel_kp', 3000)
        self.declare_parameter('vel_ki', 1050)
        # Feedback-velocity correction. The board reports body velocity using a
        # FIXED internal calibration for the CircusPi reference robot and ignores
        # the 0x24/0x23 geometry config on the feedback path (verified on
        # hardware: a config readback confirms gear_ratio etc. are stored, yet
        # changing them does not move odom at all). Result: odom over-reports
        # ~5x. These scales multiply the reported velocity back to real SI units
        # before integration -- this is the ONLY working odom calibration lever.
        # odom_linear_scale from the 1 m straight test (SLAM_learning_note.md
        # §7.1); odom_angular_scale from the 720 deg spin test (§7.2). Set both
        # to 1.0 to see the board's raw (uncorrected) output.
        self.declare_parameter('odom_linear_scale', 0.16)    # measured: raw over-reports ~6.25x (5.0-6.55 across runs)
        self.declare_parameter('odom_angular_scale', 0.195)  # wheel-az fallback scale (only if use_gyro_heading=False)
        # Heading source. The wheel-derived az is destroyed by mecanum roller
        # slip -- a real 360 deg spin over-reports as ~2270 deg of wheel az. The
        # board's raw gyro-Z is a direct yaw-rate measurement, accurate to ~3%
        # on the same spin and immune to slip, so integrate IT for heading. (The
        # IMU quaternion is useless: 6-axis, no magnetometer -> yaw is frozen.)
        self.declare_parameter('use_gyro_heading', True)
        self.declare_parameter('gyro_z_sign', 1.0)   # flip to -1.0 if odom yaw turns the wrong way
        self.declare_parameter('gyro_scale', 1.0)     # 360 deg spin read ~350 deg; ~1.0, refine if needed

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
        self.odom_lin_scale = self.get_parameter('odom_linear_scale').value
        self.odom_ang_scale = self.get_parameter('odom_angular_scale').value
        self.use_gyro_heading = self.get_parameter('use_gyro_heading').value
        self.gyro_sign = self.get_parameter('gyro_z_sign').value
        self.gyro_scale = self.get_parameter('gyro_scale').value

        wheel_diameter = self.get_parameter('wheel_diameter_mm').value
        wheel_space = self.get_parameter('wheel_space_mm').value
        axle_space = self.get_parameter('axle_space_mm').value
        encoder_ppr = self.get_parameter('encoder_ppr').value
        gear_ratio = self.get_parameter('gear_ratio').value
        pos_kp = self.get_parameter('pos_kp').value
        pos_ki = self.get_parameter('pos_ki').value
        pos_kd = self.get_parameter('pos_kd').value
        vel_kp = self.get_parameter('vel_kp').value
        vel_ki = self.get_parameter('vel_ki').value

        # --- serial board -----------------------------------------------------
        self.get_logger().info(
            f'Opening OminiBotHV on {port} @ {baud} '
            f'(wheel_diameter={wheel_diameter}mm, wheel_space={wheel_space}mm, '
            f'axle_space={axle_space}mm, encoder_ppr={encoder_ppr}, '
            f'gear_ratio={gear_ratio}, pos_pid=({pos_kp},{pos_ki},{pos_kd}), '
            f'vel_pid=({vel_kp},{vel_ki}))')
        self.bot = OminiBotHV(port=port, baud=baud,
                              wheel_diameter=wheel_diameter,
                              wheel_space=wheel_space,
                              axle_space=axle_space,
                              encoder_ppr=encoder_ppr,
                              gear_ratio=gear_ratio,
                              pos_kp=pos_kp, pos_ki=pos_ki, pos_kd=pos_kd,
                              vel_kp=vel_kp, vel_ki=vel_ki)

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

        # Board feedback -> REP-103 body velocities. Linear x/y from the wheels
        # (sign + measured scale, since the raw feedback over-reports ~6x). Yaw
        # rate from the raw gyro (immune to mecanum slip); fall back to the
        # scaled wheel az only if gyro heading is disabled.
        lx = self.sx * data['lx'] * self.odom_lin_scale
        ly = self.sy * data['ly'] * self.odom_lin_scale
        if self.use_gyro_heading:
            az = self.gyro_sign * data['gyro_z'] * self.gyro_scale
        else:
            az = self.sz * data['az'] * self.odom_ang_scale

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
