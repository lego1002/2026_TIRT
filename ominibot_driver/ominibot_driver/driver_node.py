"""ROS 2 driver node for the OminiBotHV mecanum controller board.

  /cmd_vel (geometry_msgs/Twist)  -->  robot_speed(lx, ly, az) over serial
  board feedback frame            -->  /odom (nav_msgs/Odometry) + odom->base_link TF
                                       /imu  (sensor_msgs/Imu, orientation only)
                                       /battery_voltage (std_msgs/Float32)

Odometry is dead-reckoned from the board's reported body velocities (smooth and
locally consistent -- exactly what slam_toolbox wants from the `odom` frame). The IMU
quaternion is published separately on /imu for optional downstream fusion; it is not
mixed into the odom TF so the odom frame stays free of absolute-heading jumps.

A watchdog re-sends the last /cmd_vel at a fixed rate and commands zero if no command
has arrived within cmd_vel_timeout, so the base stops if the teleop link drops.
"""

import fcntl
import math
import shutil
import struct
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
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
        # How long to keep retrying a busy port before giving up. oled_status
        # reads pack voltage off this same UART while no driver runs and holds
        # it exclusively for up to ~1.5 s; without this, a bringup that starts
        # inside that window dies instead of waiting it out. See ominibot_hv.
        self.declare_parameter('port_open_retry_s', 6.0)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('cmd_rate', 20.0)       # Hz, command re-send rate (fallback timer)
        # Send each command right after a feedback frame lands, instead of from
        # a free-running timer. The board streams feedback at its own 20 Hz and
        # cannot take a command mid-transmit: a write that overlaps its TX
        # truncates that frame (desync, bcc stays 0). Two 20 Hz clocks that are
        # not quite equal drift through each other, so the collisions come in
        # BEATS -- measured 2026-09-11: a 5-10 s burst of up to 90 % bad frames
        # every ~100 s, robot idle or not, with the kernel's UART overrun
        # counter at 0 (so the bytes were never sent, not lost on the Pi).
        # Writing just after a frame ends puts the command in the quiet 47 ms
        # before the next one and removes the beat entirely. The cmd_rate timer
        # stays as a fallback so the watchdog stop still goes out if feedback
        # ever dies.
        self.declare_parameter('cmd_sync_to_feedback', True)
        # Watchdog window. Raised 0.5 -> 1.0 on 2026-07-27: when /cmd_vel comes from
        # the laptop over WiFi, an ordinary transport stall of a few hundred ms was
        # enough to trip the watchdog, zero the base, and then resume -- which is
        # exactly the "一頓一頓" stutter the operator saw. 1.0 s still stops the robot
        # promptly if the link genuinely dies. (The structural fix is to run teleop
        # on the Pi so /cmd_vel never crosses the network at all -- see pi/robot_tmux.sh
        # and gcs.sh -- but keep this margin for nav2/PC-side publishers.)
        self.declare_parameter('cmd_vel_timeout', 1.0)  # s, stop if no cmd_vel
        # /cmd_vel QoS. BEST_EFFORT by default: over lossy WiFi, RELIABLE makes DDS
        # retransmit *stale* Twists and head-of-line block behind them, so the base
        # acts on old commands in bursts. A velocity stream is inherently
        # "latest value wins" -- dropping a sample is strictly better than delaying
        # every later one. A BEST_EFFORT subscription still matches RELIABLE
        # publishers, so nothing else breaks. Set false to force RELIABLE.
        self.declare_parameter('cmd_vel_best_effort', True)
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
        # PWM duty limits written into the board (0x23 config frame; range 1..7199
        # = 0..100% duty, PDF p.4). The factory 3600/2100 come from the vendor's
        # own comment "motor range: 3v-6v" -- i.e. they are a protection limit for
        # 6V motors on a 12V supply (PDF p.23: board default +12V).
        # THIS ROBOT USES 12V 200rpm N20 MOTORS, so 3600 = 50% duty = 6V caps them
        # at HALF their rated voltage, which halves available torque. Unloaded
        # (wheels in the air) the loop never reaches the ceiling and everything
        # looks perfect; on the floor a wheel that needs more than 6V worth of
        # torque saturates, falls behind its setpoint, and the four wheels desync.
        # Raising this toward ~6800 (94%) is within the motors' rating -- but
        # verify the supply really is 12V first, and raise it gradually.
        # Not a regression suspect: these values were the same last week when the
        # chassis drove fine. This is a torque-headroom fix, not the stutter fix.
        self.declare_parameter('motor_pwm_max', 3600)
        self.declare_parameter('motor_pwm_min', 2100)
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
        # ~6.5x. These scales multiply the reported velocity back to real SI units
        # before integration -- this is the ONLY working odom calibration lever.
        # odom_linear_scale from the 1 m straight test (SLAM_learning_note.md
        # §7.1); odom_angular_scale from the 720 deg spin test (§7.2). Set both
        # to 1.0 to see the board's raw (uncorrected) output, which is how the
        # 2026-07-25 numbers below were taken (tools/odom_check.py).
        self.declare_parameter('odom_linear_scale', 0.153)   # 2026-07-25 1m test: raw over-reports 6.54x
        self.declare_parameter('odom_angular_scale', 0.195)  # wheel-az fallback scale (only if use_gyro_heading=False)
        # Command-velocity correction -- the OTHER HALF of the same calibration
        # error. The board mis-scales the /cmd_vel it receives by the same fixed
        # internal calibration, so a 0.15 m/s command produced only 0.019 m/s of
        # real motion (measured 2026-07-25, tools/step_response.py; /odom and
        # direct observation agreed -- the robot moved <2 cm over a whole test).
        # Only the feedback half had been corrected until then, which is why the
        # teleop defaults had crept up to an absurd 0.6 m/s / 1.5 rad/s for a
        # 15 cm robot: they were compensating for this by hand.
        # Calibrate with tools/vel_sweep.py, which regresses actual-vs-commanded
        # and prints the scale directly. Default 1.0 = uncorrected (the value the
        # sweep must be run with).
        self.declare_parameter('cmd_linear_scale', 1.0)
        self.declare_parameter('cmd_angular_scale', 1.0)
        # Per-motor direction bitmasks written into the board at startup (0x23
        # config frame). Both are CircusPi FACTORY values for the reference
        # robot's wiring, never verified against this build. encoder_direct=10
        # is 0b1010 -- i.e. motors 2 and 4 decoded reversed.
        # Exposed 2026-07-25 chasing an apparent "3 of 4 wheels only work in one
        # direction" result from tools/wheel_test.py. That result was RETRACTED:
        # re-running moved the fault to different wheels every time, and the
        # operator confirmed all four wheels spin fine both ways by eye. It was a
        # measurement artifact (reported body velocity saturates at ~2.77, so the
        # average was really measuring spin-up time). No wiring fault has been
        # demonstrated -- these stay at the factory values and are exposed only
        # so the hypothesis can be tested cheaply if it ever comes back:
        #     ./run_robot.sh encoder_direct:=0
        #     ./run_robot.sh encoder_direct:=15
        self.declare_parameter('motor_direct', 0)
        self.declare_parameter('encoder_direct', 10)
        # Heading source. The wheel-derived az is destroyed by mecanum roller
        # slip -- a real 360 deg spin over-reports as ~2270 deg of wheel az. The
        # board's raw gyro-Z is a direct yaw-rate measurement, accurate to ~3%
        # on the same spin and immune to slip, so integrate IT for heading. (The
        # IMU quaternion is useless: 6-axis, no magnetometer -> yaw is frozen.)
        self.declare_parameter('use_gyro_heading', True)
        self.declare_parameter('gyro_z_sign', 1.0)   # flip to -1.0 if odom yaw turns the wrong way
        self.declare_parameter('gyro_scale', 1.014)   # 2026-07-25 analyze_bag: odom 145.3 deg vs scan truth 137.1 deg
        # Gyro zero-offset (bias) tracking. A MEMS gyro does not read exactly 0
        # at rest, and the offset changes with every power-up and with die
        # temperature. Measured on this board 2026-08-30, robot completely
        # stationary, 1236 samples over 60 s: raw gyro_z averaged -0.000447 rad/s
        # -> the integrated odom heading rotated -1.6 deg/min while nothing moved.
        # That error grows with ELAPSED TIME, not with distance driven, which is
        # exactly the "map is fine at the start and bends later, and it differs
        # every run" symptom. So measure the offset at startup and keep tracking
        # it whenever the base is idle. Set gyro_auto_bias:=false to go back to
        # the raw reading.
        self.declare_parameter('gyro_auto_bias', True)
        self.declare_parameter('gyro_bias_init_samples', 40)   # ~2 s at 20 Hz
        self.declare_parameter('gyro_bias_tau', 30.0)          # idle re-estimate time constant [s]
        self.declare_parameter('gyro_bias_still_time', 0.5)    # must be idle this long before trusting a sample [s]
        # Longest gap between two feedback frames that odom will still integrate
        # across. Feedback normally arrives at 20 Hz; when the serial link
        # degrades (2026-09-11: 90% of frames lost for 5 s at a time while the
        # Pi was browning out) the gaps stretch to ~0.7 s. The old hard-coded
        # 0.5 s cap threw that motion away entirely, so the robot drove on while
        # /odom stood still -- SLAM saw no travel, inserted nothing, and the live
        # scan sat 20 cm off the walls in RViz right after the first drive.
        # Integrating across the gap with the mean of the two bracketing
        # velocities is far closer to the truth than zero. Anything longer than
        # this is dropped and logged: guessing across a multi-second hole is
        # worse than a visible kink.
        self.declare_parameter('odom_max_gap', 1.0)            # [s]

        port = self.get_parameter('port').value
        port_open_retry_s = float(self.get_parameter('port_open_retry_s').value)
        baud = self.get_parameter('baud').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        self.publish_imu = self.get_parameter('publish_imu').value
        cmd_rate = self.get_parameter('cmd_rate').value
        self.cmd_sync_to_feedback = self.get_parameter('cmd_sync_to_feedback').value
        self._cmd_period = 1.0 / cmd_rate
        self._last_send_mono = 0.0
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.sx = self.get_parameter('linear_x_sign').value
        self.sy = self.get_parameter('linear_y_sign').value
        self.sz = self.get_parameter('angular_z_sign').value
        self.odom_lin_scale = self.get_parameter('odom_linear_scale').value
        self.odom_ang_scale = self.get_parameter('odom_angular_scale').value
        self.cmd_lin_scale = self.get_parameter('cmd_linear_scale').value
        self.cmd_ang_scale = self.get_parameter('cmd_angular_scale').value
        self.use_gyro_heading = self.get_parameter('use_gyro_heading').value
        self.gyro_sign = self.get_parameter('gyro_z_sign').value
        self.gyro_scale = self.get_parameter('gyro_scale').value
        self.gyro_auto_bias = self.get_parameter('gyro_auto_bias').value
        self.gyro_bias_init_n = self.get_parameter('gyro_bias_init_samples').value
        self.gyro_bias_tau = self.get_parameter('gyro_bias_tau').value
        self.gyro_bias_still_time = self.get_parameter('gyro_bias_still_time').value
        self.odom_max_gap = float(self.get_parameter('odom_max_gap').value)

        wheel_diameter = self.get_parameter('wheel_diameter_mm').value
        motor_direct = self.get_parameter('motor_direct').value
        encoder_direct = self.get_parameter('encoder_direct').value
        wheel_space = self.get_parameter('wheel_space_mm').value
        axle_space = self.get_parameter('axle_space_mm').value
        encoder_ppr = self.get_parameter('encoder_ppr').value
        gear_ratio = self.get_parameter('gear_ratio').value
        motor_pwm_max = self.get_parameter('motor_pwm_max').value
        motor_pwm_min = self.get_parameter('motor_pwm_min').value
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
            f'vel_pid=({vel_kp},{vel_ki}), '
            f'pwm=({motor_pwm_min}..{motor_pwm_max} of 7199))')
        self.bot = OminiBotHV(port=port, baud=baud,
                              motor_direct=motor_direct,
                              encoder_direct=encoder_direct,
                              motor_pwm_max=motor_pwm_max,
                              motor_pwm_min=motor_pwm_min,
                              wheel_diameter=wheel_diameter,
                              wheel_space=wheel_space,
                              axle_space=axle_space,
                              encoder_ppr=encoder_ppr,
                              gear_ratio=gear_ratio,
                              pos_kp=pos_kp, pos_ki=pos_ki, pos_kd=pos_kd,
                              vel_kp=vel_kp, vel_ki=vel_ki,
                              open_retry_s=port_open_retry_s)

        # --- state ------------------------------------------------------------
        self._cmd_lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)             # (lx, ly, az)
        self._last_cmd_time = self.get_clock().now()
        self.x = self.y = self.theta = 0.0
        self._last_odom_time = None
        self._prev_vel = None        # (lx, ly, az) of the previous feedback frame
        # Gyro bias state. _bias_ready gates heading integration: integrating
        # before the offset is known just bakes it in permanently.
        self.gyro_bias = 0.0
        self._bias_init = []
        self._bias_ready = not (self.use_gyro_heading and self.gyro_auto_bias)
        self._still_since = None

        # --- ROS interfaces ---------------------------------------------------
        # Depth 1: only the newest Twist matters. A deeper queue just means that
        # after a WiFi stall the base replays a backlog of commands it should have
        # skipped. See the cmd_vel_best_effort parameter for the reliability choice.
        cmd_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if self.get_parameter('cmd_vel_best_effort').value
                         else ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, cmd_qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu', 20)
        # Battery voltage from the same feedback frame. Published because motor
        # behaviour degrading "after driving for a while" is far more often
        # supply sag than mechanical wear -- a sagging pack lowers available
        # torque, the board's velocity PID pushes harder to hold the setpoint,
        # and the result reads as stutter. Without this topic that hypothesis
        # can't be told apart from gearbox backlash. Low rate: it changes slowly.
        self.batt_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self._batt_decim = 0
        self.tf_broadcaster = TransformBroadcaster(self)

        # Watchdog / command re-send timer (runs in the executor thread). With
        # cmd_sync_to_feedback the read thread does the sending and this only
        # fires when feedback has been missing for two periods.
        self.create_timer(1.0 / cmd_rate, self._send_cmd_timer)

        # Serial link health. The board<->Pi link moved from USB (FTDI) to the
        # Pi's GPIO UART on 2026-07-19; if noise corrupts a command frame the
        # board rejects it on checksum, its own watchdog zeros the motors, and
        # the chassis stutters. read_feedback() cannot surface that on its own --
        # it returns None for a bad checksum exactly as it does for an idle link.
        # Logging the counters makes a degrading link visible during real driving
        # (tools/motor_diag.py needs the driver stopped, so it cannot watch a
        # teleop run). Silent while the link is clean.
        self._last_stats = dict(self.bot.stats)
        # Kernel-side UART counters (TIOCGICOUNT on the driver's own fd, no
        # root needed) and the Pi firmware's power flags, so the WARN below can
        # tell "bytes were lost in the Pi's UART FIFO" from "the board sent
        # garbage". Added 2026-09-11 while chasing the desync beats: the Pi was
        # logging "Undervoltage detected!" every 20-40 s and the first guess was
        # FIFO overrun under CPU throttling. The counter said overrun=0 through
        # an 81 %-bad window, which is what pointed at the board's TX instead
        # (see cmd_sync_to_feedback). Kept: the brown-outs are real on their
        # own, and this is the only place they show up without a laptop.
        self._last_icount = self._uart_icount()
        self._vcgencmd = shutil.which('vcgencmd')
        self._last_throttled = self._pi_throttled()
        self._last_power_warn = 0.0
        if self._last_throttled and self._last_throttled & 0xF0000:
            self.get_logger().warn(
                f'Pi firmware reports power trouble since boot '
                f'({self._throttled_text(self._last_throttled)}) -- '
                'a sagging 5 V rail throttles the CPU and loses UART bytes')
        self.create_timer(5.0, self._log_link_health)

        # Feedback read loop (blocking serial reads -> own thread).
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    # -- /cmd_vel ------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        with self._cmd_lock:
            self._cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _send_cmd_timer(self):
        if (self.cmd_sync_to_feedback
                and time.monotonic() - self._last_send_mono < 2.0 * self._cmd_period):
            return      # the read thread is sending in step with the feedback
        self._send_cmd()

    def _send_cmd(self):
        self._last_send_mono = time.monotonic()
        stale = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        with self._cmd_lock:
            lx, ly, az = self._cmd
        if stale > self.cmd_vel_timeout:
            lx = ly = az = 0.0
        try:
            # cmd_*_scale converts real SI units into whatever the board thinks
            # they are (see the parameter declaration). Signs are applied to BOTH
            # command and feedback so the two stay consistent.
            self.bot.robot_speed(self.sx * lx * self.cmd_lin_scale,
                                 self.sy * ly * self.cmd_lin_scale,
                                 self.sz * az * self.cmd_ang_scale)
        except Exception as exc:  # noqa: BLE001 - keep node alive on serial hiccup
            self.get_logger().warn(f'robot_speed write failed: {exc}')

    # -- serial link health --------------------------------------------------
    _TIOCGICOUNT = 0x545D   # linux/serial.h: struct serial_icounter_struct

    def _uart_icount(self):
        """Kernel per-port error counters {frame, overrun, parity, brk,
        buf_overrun}, or None if the driver does not support the ioctl.
        `overrun` = the UART FIFO overflowed before the ISR drained it (bytes
        lost inside the Pi -- interrupt latency, i.e. CPU throttling / load);
        `buf_overrun` = the tty buffer overflowed (this process not reading)."""
        try:
            buf = fcntl.ioctl(self.bot.ser.fileno(), self._TIOCGICOUNT,
                              bytes(20 * 4))
            v = struct.unpack('20i', buf)
        except (OSError, AttributeError):
            return None
        return {'frame': v[6], 'overrun': v[7], 'parity': v[8], 'brk': v[9],
                'buf_overrun': v[10]}

    def _pi_throttled(self):
        """`vcgencmd get_throttled` bitmask, or None off a Pi / on failure.
        Bit 0 under-voltage NOW, 1 ARM freq capped NOW, 2 throttled NOW,
        3 soft temp limit NOW; bits 16-19 = the same, has occurred since boot."""
        if not self._vcgencmd:
            return None
        try:
            out = subprocess.run([self._vcgencmd, 'get_throttled'],
                                 capture_output=True, text=True, timeout=1.0)
            return int(out.stdout.strip().split('=')[1], 16)
        except Exception:  # noqa: BLE001 - diagnostics must never take the driver down
            return None

    @staticmethod
    def _throttled_text(flags):
        names = ['under-voltage', 'arm-freq-capped', 'throttled', 'soft-temp-limit']
        now = [n for i, n in enumerate(names) if flags & (1 << i)]
        past = [n for i, n in enumerate(names) if flags & (1 << (16 + i))]
        parts = []
        if now:
            parts.append('NOW: ' + ','.join(now))
        if past:
            parts.append('since boot: ' + ','.join(past))
        return f'0x{flags:x}' + (' ' + '; '.join(parts) if parts else ' ok')

    def _log_link_health(self):
        now = dict(self.bot.stats)
        delta = {k: now[k] - self._last_stats.get(k, 0) for k in now}
        self._last_stats = now
        bad = delta['bcc_fail'] + delta['desync'] + delta['short']
        total = bad + delta['good']

        icount = self._uart_icount()
        uart = ''
        fifo_overrun = False
        if icount and self._last_icount:
            d = {k: icount[k] - self._last_icount[k] for k in icount}
            uart = (f' | uart overrun={d["overrun"]} buf_overrun={d["buf_overrun"]}'
                    f' frame={d["frame"]}')
            fifo_overrun = d['overrun'] > 0
        self._last_icount = icount

        throttled = self._pi_throttled()
        power = ''
        if throttled is not None and (
                throttled & 0xF
                or (self._last_throttled is not None
                    and (throttled & 0xF0000) != (self._last_throttled & 0xF0000))):
            # Sampled once per 5 s and a brown-out lasts 2-4 s, so this catches
            # roughly every other one. On a chronically sagging supply that is
            # a line every 5 s, so the standalone "link clean, but" warning is
            # rate-limited to once a minute; a degraded-link line always
            # carries it.
            power = f' | power: {self._throttled_text(throttled)}'
        self._last_throttled = throttled

        if total == 0:
            self.get_logger().warn(
                'serial link silent: no feedback frames for 5s '
                f'(board powered? MODE_ON? TX/RX swapped?){uart}{power}')
            return
        if bad / total > 0.02:
            self.get_logger().warn(
                f'serial link degraded: {bad}/{total} bad frames '
                f'({100.0 * bad / total:.1f}%) in 5s [bcc={delta["bcc_fail"]} '
                f'desync={delta["desync"]} short={delta["short"]} '
                f'timeout={delta["timeout"]}]{uart}{power} -- '
                + ('UART FIFO overrun: bytes lost inside the Pi (CPU starved -- '
                   'check the 5 V supply, then Pi load), not on the wire'
                   if fifo_overrun
                   else 'corrupted command frames make the board stop the motors, '
                        'which feels like stuttering'))
        elif power and time.monotonic() - self._last_power_warn > 60.0:
            self._last_power_warn = time.monotonic()
            self.get_logger().warn(f'link clean, but{power}')

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
            # The board has just finished transmitting: send now, before the
            # (slower) odom publish, so the write lands well clear of its next
            # frame. See cmd_sync_to_feedback.
            if self.cmd_sync_to_feedback:
                self._send_cmd()
            try:
                self._publish(data)
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    break  # context torn down mid-publish during shutdown
                self.get_logger().warn(f'publish failed: {exc}')

    def _track_gyro_bias(self, data, now, dt):
        """Measure and follow the gyro's zero offset while the base is idle.

        "Idle" is judged from the board's own wheel feedback (lx/ly/az all
        exactly 0.0 when stopped) plus the absence of a live non-zero command,
        and it has to hold for gyro_bias_still_time before any sample counts --
        a gyro rings for a moment after the wheels stop, and averaging that ring
        into the offset would be worse than not correcting at all.

        Startup: average gyro_bias_init_samples readings, then unfreeze heading.
        After that: a slow exponential average (gyro_bias_tau) so the offset can
        follow die temperature over a 30-minute session without ever reacting
        fast enough to eat a real rotation.
        """
        with self._cmd_lock:
            commanded = self._cmd
        stale = (now - self._last_cmd_time).nanoseconds * 1e-9
        idle = (commanded == (0.0, 0.0, 0.0)) or stale > self.cmd_vel_timeout
        still = (idle and data['lx'] == 0.0
                 and data['ly'] == 0.0 and data['az'] == 0.0)

        if not still:
            self._still_since = None
            return
        if self._still_since is None:
            self._still_since = now
            return
        if (now - self._still_since).nanoseconds * 1e-9 < self.gyro_bias_still_time:
            return

        if not self._bias_ready:
            self._bias_init.append(data['gyro_z'])
            if len(self._bias_init) >= self.gyro_bias_init_n:
                self.gyro_bias = sum(self._bias_init) / len(self._bias_init)
                self._bias_ready = True
                self.get_logger().info(
                    f'gyro zero offset = {self.gyro_bias:+.6f} rad/s '
                    f'({math.degrees(self.gyro_bias) * 60.0:+.1f} deg/min of '
                    f'heading drift removed; {len(self._bias_init)} samples)')
            return

        if dt > 0.0:
            alpha = min(dt / self.gyro_bias_tau, 1.0)
            self.gyro_bias += alpha * (data['gyro_z'] - self.gyro_bias)

    def _publish(self, data):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Board feedback -> REP-103 body velocities. Linear x/y from the wheels
        # (sign + measured scale, since the raw feedback over-reports ~6x). Yaw
        # rate from the raw gyro (immune to mecanum slip); fall back to the
        # scaled wheel az only if gyro heading is disabled.
        dt = 0.0
        gap = 0.0
        if self._last_odom_time is not None:
            gap = (now - self._last_odom_time).nanoseconds * 1e-9
            dt = gap if 0.0 < gap <= self.odom_max_gap else 0.0
        self._last_odom_time = now

        lx = self.sx * data['lx'] * self.odom_lin_scale
        ly = self.sy * data['ly'] * self.odom_lin_scale
        if self.use_gyro_heading:
            if self.gyro_auto_bias:
                self._track_gyro_bias(data, now, dt)
            az = self.gyro_sign * (data['gyro_z'] - self.gyro_bias) * self.gyro_scale
        else:
            az = self.sz * data['az'] * self.odom_ang_scale

        # A gap of more than a few missed frames means the serial link dropped
        # feedback while the robot may well have been moving. Say so -- a
        # silent hole here is exactly the "map is fine, then suddenly the scan
        # is 20 cm off the wall" symptom, and it is otherwise invisible.
        moving = (any(abs(v) > 1e-3 for v in (lx, ly, az))
                  or (self._prev_vel is not None
                      and any(abs(v) > 1e-3 for v in self._prev_vel)))
        if gap > 0.25 and moving:
            if dt > 0.0:
                self.get_logger().warn(
                    f'feedback gap {gap:.2f}s while moving -- integrated with '
                    'the mean of the bracketing velocities (see serial link stats)')
            else:
                self.get_logger().warn(
                    f'feedback gap {gap:.2f}s while moving exceeds odom_max_gap '
                    f'{self.odom_max_gap:.1f}s -- that motion is LOST from /odom')

        # Dead-reckon odom from body velocities, trapezoidal (mean of the
        # previous and current frame's velocities over the interval between
        # them). Held off until the gyro zero offset is known (a couple of
        # seconds of standing still at startup) -- the robot is not moving
        # during that window anyway, and integrating early would bake the
        # offset into every pose that follows.
        if dt > 0.0 and self._bias_ready:
            if self._prev_vel is not None:
                vx = 0.5 * (lx + self._prev_vel[0])
                vy = 0.5 * (ly + self._prev_vel[1])
                wz = 0.5 * (az + self._prev_vel[2])
            else:
                vx, vy, wz = lx, ly, az
            mid = self.theta + 0.5 * wz * dt
            self.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
            self.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
            self.theta = math.atan2(math.sin(self.theta + wz * dt),
                                    math.cos(self.theta + wz * dt))
        self._prev_vel = (lx, ly, az)

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

        # ~1 Hz (feedback streams at ~20 Hz); battery voltage is a slow signal.
        self._batt_decim += 1
        if self._batt_decim >= 20:
            self._batt_decim = 0
            self.batt_pub.publish(Float32(data=float(data['battery'])))

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
