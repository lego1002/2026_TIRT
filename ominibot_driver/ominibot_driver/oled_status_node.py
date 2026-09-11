"""ROS 2 node driving the SPI OLED bolted to the robot as a status readout.

  /battery_voltage (std_msgs/Float32)  --> pack voltage + charge bar
  /odom            (nav_msgs/Odometry) --> measured body velocity
  /scan            (sensor_msgs/LaserScan) + /odom --> liveness rates

The point of this node is the battery-only run: at the venue there is no
laptop attached, so `ros2 topic echo` is not available and a flat pack or a
dead lidar is otherwise invisible until the robot misbehaves on the floor.
Everything drawn here answers a question you would otherwise need the PC for.

The IP line matters more than it looks: the robot's DDS link is pinned to a
static 10.77.0.2 alias that only exists once wifi associates, so "did this
boot onto the right network" is a real pre-race check -- and it is exactly
the check you cannot run from the PC, because failing it is what stops the
PC from seeing the robot at all.

Rates are computed from arrival timestamps rather than message headers so
that a stalled publisher reads as 0.0 Hz instead of freezing at its last
value. A topic silent for `stale_after` seconds prints as "--".

The panel is written from a plain rclpy timer at `refresh_rate`; a full
128x64 frame is 1 KB over SPI, which is nothing next to the 10 Hz lidar.
"""

import subprocess
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class RateTracker:
    """Sliding-window arrival-rate estimator for one topic."""

    def __init__(self, window=20):
        self._stamps = deque(maxlen=window)
        self.last = 0.0

    def tick(self):
        now = time.monotonic()
        self._stamps.append(now)
        self.last = now

    def hz(self):
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else 0.0

    def stale(self, after):
        return self.last == 0.0 or (time.monotonic() - self.last) > after


class OledStatus(Node):
    def __init__(self):
        super().__init__('oled_status')

        # Panel wiring / controller. Defaults match the module bundled with the
        # chassis kit: 6-pin SPI (no CS line), SSD1306 controller. Flip
        # `controller` to sh1106 if the panel lights but the image is shifted.
        self.declare_parameter('controller', 'ssd1306')
        self.declare_parameter('spi_port', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('gpio_dc', 24)
        self.declare_parameter('gpio_rst', 25)
        self.declare_parameter('width', 128)
        self.declare_parameter('height', 64)
        self.declare_parameter('rotate', 0)      # 0-3, quarter turns
        self.declare_parameter('contrast', 255)  # 0-255

        self.declare_parameter('refresh_rate', 2.0)   # Hz, panel redraws
        self.declare_parameter('stale_after', 3.0)    # s, then a topic reads "--"

        # Pack voltage -> charge bar endpoints. Defaults are a 3S LiPo (12.6 V
        # full, 10.5 V at a safe cutoff), which is what a "+12 V" OminiBotHV
        # supply normally is. THESE ARE ASSUMPTIONS -- measure the real pack
        # full and empty, then set them, or the bar is decorative.
        self.declare_parameter('batt_full_v', 12.6)
        self.declare_parameter('batt_empty_v', 10.5)
        # Below this the voltage text blinks. Sagging supply is the single most
        # common cause of "the motors got weak after a few minutes".
        self.declare_parameter('batt_warn_v', 11.1)

        # Standalone battery read. /battery_voltage only exists while the driver
        # runs, i.e. while someone has started bringup from the laptop -- so on a
        # freshly booted robot (the boot service, pi/oled_boot.sh) the one number
        # you actually want before touching anything reads "--". The board streams
        # its feedback frames continuously on the UART whether or not anyone has
        # configured it, so when nothing is publishing the topic this node reads
        # the pack voltage straight off the wire.
        #
        # This poll is deliberately READ-ONLY: it never sends the vendor init
        # sequence, so it cannot overwrite the driver's calibrated geometry/PID
        # config, and it cannot move the base. If the board happens not to be
        # streaming, the panel simply keeps showing "--" as before.
        #
        # The port is exclusive (see ominibot_hv.py), so the two readers must not
        # overlap: the poll is skipped whenever the topic is live, whenever a
        # publisher exists, and whenever a driver process is merely *running*
        # (that last check is the one that matters -- driver_node opens the port
        # about a second before it creates the publisher, so waiting for the
        # publisher to appear would be too late). It then holds the port for at
        # most `batt_listen_s`, and driver_node retries a busy port for a few
        # seconds, so the remaining race closes itself.
        self.declare_parameter('batt_serial_fallback', True)
        self.declare_parameter('batt_port', '/dev/serial0')
        self.declare_parameter('batt_baud', 115200)
        self.declare_parameter('batt_poll_period', 10.0)  # s between direct reads
        self.declare_parameter('batt_listen_s', 1.5)      # max time holding the port
        # A voltage older than this reads "--" again rather than staying frozen
        # on the glass: a dead board must not look like a healthy pack.
        self.declare_parameter('batt_stale_after', 60.0)

        p = self.get_parameter
        self.stale_after = p('stale_after').value
        self.batt_full = p('batt_full_v').value
        self.batt_empty = p('batt_empty_v').value
        self.batt_warn = p('batt_warn_v').value
        self.batt_stale_after = p('batt_stale_after').value

        self._init_panel()

        self.batt_v = None
        self.batt_stamp = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.rate_odom = RateTracker()
        self.rate_scan = RateTracker()
        self._blink = False
        self._ip = '...'
        self._ip_checked = 0.0

        # BEST_EFFORT everywhere: a best-effort subscriber matches both
        # best-effort and reliable publishers, while a reliable subscriber
        # silently fails to match a best-effort one. This node only ever wants
        # the newest sample, so depth 1 and no delivery guarantee is right --
        # and it keeps a slow SPI redraw from ever back-pressuring the lidar.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Float32, 'battery_voltage', self._batt_cb, qos)
        self.create_subscription(Odometry, 'odom', self._odom_cb, qos)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos)

        self.create_timer(1.0 / p('refresh_rate').value, self._draw)

        self._stop = threading.Event()
        self._batt_thread = None
        if p('batt_serial_fallback').value:
            # Own thread, not a ROS timer: a poll blocks on serial for up to
            # batt_listen_s, and on the executor that would freeze the panel
            # (and the rate estimators' notion of "now") for the same time.
            self._batt_thread = threading.Thread(
                target=self._batt_poll_loop, daemon=True, name='oled_batt_poll')
            self._batt_thread.start()

        self.get_logger().info(
            f"OLED status up: {p('controller').value} "
            f"{p('width').value}x{p('height').value} on spi"
            f"{p('spi_port').value}.{p('spi_device').value} "
            f"DC=GPIO{p('gpio_dc').value} RST=GPIO{p('gpio_rst').value}"
        )

    def _init_panel(self):
        """Open the panel. Fatal on failure -- a status display that silently
        isn't there is worse than not having one, since you would trust it."""
        from luma.core.interface.serial import spi
        from luma.oled.device import sh1106, ssd1306

        p = self.get_parameter
        name = str(p('controller').value).lower()
        driver = {'ssd1306': ssd1306, 'sh1106': sh1106}.get(name)
        if driver is None:
            raise ValueError(f"unknown controller '{name}' (use ssd1306 or sh1106)")

        serial = spi(
            port=p('spi_port').value,
            device=p('spi_device').value,
            gpio_DC=p('gpio_dc').value,
            gpio_RST=p('gpio_rst').value,
        )
        self.device = driver(
            serial,
            width=p('width').value,
            height=p('height').value,
            rotate=p('rotate').value,
        )
        self.device.contrast(int(p('contrast').value))
        # Keep the last frame on the glass after this process exits, so a crash
        # leaves the final reading visible instead of a black panel that looks
        # identical to "never started".
        self.device.persist = True

    def _batt_cb(self, msg):
        self.batt_v = msg.data
        self.batt_stamp = time.monotonic()

    def _odom_cb(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.wz = msg.twist.twist.angular.z
        self.rate_odom.tick()

    def _scan_cb(self, _msg):
        self.rate_scan.tick()

    # -- standalone battery read (only while no driver owns the port) --------

    def _driver_running(self):
        """True if an ominibot_driver process exists, even if it has not opened
        the port yet. Matched on the executable path like run_robot.sh's pkill
        patterns -- a bare node name also matches a grep, a tail or an editor
        that merely mentions it."""
        try:
            return subprocess.run(
                ['pgrep', '-f', '/ominibot_driver_node'],
                capture_output=True, timeout=2.0,
            ).returncode == 0
        except Exception:  # noqa: BLE001
            return True     # can't tell -> assume it is, and stay off the port

    def _topic_has_publisher(self):
        try:
            return self.count_publishers(
                self.resolve_topic_name('battery_voltage')) > 0
        except Exception:  # noqa: BLE001
            return False

    def _read_batt_once(self):
        """Listen for one valid streaming frame and return its pack voltage.

        Frame: 0x7b 0x00 vel(6) imu(20) battery(2) bcc(1) 0x7d -- the same
        layout ominibot_hv.read_feedback() parses; only the checksum helper is
        borrowed from it, since constructing an OminiBotHV would send the vendor
        config frames and this path must stay read-only.
        """
        import serial

        from ominibot_driver.ominibot_hv import OminiBotHV

        port = self.get_parameter('batt_port').value
        baud = int(self.get_parameter('batt_baud').value)
        listen = float(self.get_parameter('batt_listen_s').value)

        ser = serial.Serial(port, baud, timeout=0.3, exclusive=True)
        try:
            deadline = time.monotonic() + listen
            while time.monotonic() < deadline:
                if ser.read(1) != b'\x7b':
                    continue
                if ser.read(1) != b'\x00':
                    continue        # readback reply or mid-frame junk
                body = ser.read(29)
                if len(body) < 29 or ser.read(1) != b'\x7d':
                    continue
                if OminiBotHV.calculate_bcc(
                        bytearray(b'\x7b\x00') + body[0:28]) != body[28]:
                    continue
                return int.from_bytes(body[26:28], 'big', signed=True) / 1000.0
        finally:
            ser.close()
        return None

    def _batt_poll_loop(self):
        period = float(self.get_parameter('batt_poll_period').value)
        logged_busy = False
        # First read comes early so a booted-but-idle robot shows its pack
        # voltage within seconds rather than after a full poll period.
        wait = min(period, 3.0)
        while not self._stop.wait(wait):
            wait = period
            # The driver is the authoritative source; never compete with it.
            if self.batt_stamp and (time.monotonic() - self.batt_stamp) < period:
                continue
            if self._topic_has_publisher() or self._driver_running():
                continue
            try:
                volts = self._read_batt_once()
            except Exception as exc:  # noqa: BLE001 - a status panel never dies
                if not logged_busy:
                    self.get_logger().warn(f'battery serial read failed: {exc}')
                    logged_busy = True
                continue
            logged_busy = False
            if volts is not None:
                if self.batt_v is None:
                    # Once, so journalctl shows the standalone path working; a
                    # per-poll line would drown the log on an idle robot.
                    self.get_logger().info(
                        f'pack voltage read directly from the board: {volts:.2f} V '
                        '(no driver running)')
                self.batt_v = volts
                self.batt_stamp = time.monotonic()

    def _host_ip(self):
        """Cheapest reliable read of the current global IPv4, refreshed slowly.

        Prefers the 10.77.0.x DDS alias when present -- that is the address the
        PC actually talks to, so it is the one worth showing.
        """
        if time.monotonic() - self._ip_checked < 5.0:
            return self._ip
        self._ip_checked = time.monotonic()
        try:
            out = subprocess.run(
                ['ip', '-4', '-o', 'addr', 'show', 'scope', 'global'],
                capture_output=True, text=True, timeout=2.0,
            ).stdout
            addrs = [ln.split()[3].split('/')[0] for ln in out.splitlines() if len(ln.split()) > 3]
            alias = [a for a in addrs if a.startswith('10.77.0.')]
            self._ip = alias[0] if alias else (addrs[0] if addrs else 'no-net')
        except Exception:  # noqa: BLE001 - a status panel must never take the node down
            self._ip = 'ip-err'
        return self._ip

    def _draw(self):
        from luma.core.render import canvas

        self._blink = not self._blink
        w = self.device.width

        batt = self.batt_v
        if batt is not None and (time.monotonic() - self.batt_stamp) > self.batt_stale_after:
            batt = None     # stale reading -> "--", not a frozen healthy-looking number
        low = batt is not None and batt < self.batt_warn
        span = max(self.batt_full - self.batt_empty, 1e-6)
        frac = 0.0 if batt is None else min(max((batt - self.batt_empty) / span, 0.0), 1.0)

        odom_dead = self.rate_odom.stale(self.stale_after)
        scan_dead = self.rate_scan.stale(self.stale_after)

        with canvas(self.device) as d:
            d.text((0, 0), f"TIRT {self._host_ip()}", fill="white")
            d.line((0, 10, w - 1, 10), fill="white")

            # Battery: hide the text on alternate frames when low, so a flat
            # pack blinks and catches the eye across the room.
            if batt is None:
                d.text((0, 14), "BAT  --.-V", fill="white")
            elif not (low and self._blink):
                d.text((0, 14), f"BAT {batt:5.2f}V {frac * 100:3.0f}%", fill="white")

            d.rectangle((0, 26, w - 1, 36), outline="white")
            if frac > 0:
                d.rectangle((2, 28, 2 + int((w - 5) * frac), 34), fill="white")

            if odom_dead:
                d.text((0, 40), "vel  -- no odom --", fill="white")
            else:
                d.text((0, 40), f"x{self.vx:+.2f} y{self.vy:+.2f} w{self.wz:+.2f}",
                       fill="white")

            scan_txt = "scan --" if scan_dead else f"scan {self.rate_scan.hz():4.1f}"
            odom_txt = "odom --" if odom_dead else f"odom {self.rate_odom.hz():4.1f}"
            d.text((0, 52), f"{scan_txt}  {odom_txt}", fill="white")

    def destroy_node(self):
        # Leave a deliberate "stopped" frame rather than a black panel, which
        # is indistinguishable from "never booted" at a glance.
        #
        # BaseException, not Exception: this runs from main()'s finally block
        # during SIGINT teardown, and pushing a frame is ~1 KB of SPI. A second
        # Ctrl-C (or launch re-signalling) lands *inside* that write and raises
        # KeyboardInterrupt, which Exception does not catch -- that escaped as a
        # traceback and a -2 exit that `ros2 launch` reports as "process has
        # died [ERROR]" on an ordinary, correct shutdown.
        stop = getattr(self, '_stop', None)
        if stop is not None:
            stop.set()
        try:
            from luma.core.render import canvas
            with canvas(self.device) as d:
                d.text((0, 24), "  ROS stopped", fill="white")
        except BaseException:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OledStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Same reasoning as destroy_node's own guard: teardown must not turn a
        # normal Ctrl-C into a non-zero exit.
        try:
            node.destroy_node()
        except BaseException:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
