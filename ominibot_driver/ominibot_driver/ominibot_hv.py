"""Self-contained serial protocol for the CircusPi OminiBotHV controller board.

Distilled from OminiBotHV-master/example/OminiBot_HV_Meca.py so the driver package
does not depend on the vendor example being on the Python path. Frame format is
``\\x7b <cmd> ... <bcc> \\x7d`` with a big-endian XOR checksum.

Two threads use one instance: the ROS timer thread calls robot_speed()/forced_stop()
(writes), the read thread calls read_feedback() (reads). Writes are guarded by a lock;
pyserial allows a concurrent read on another thread.

Two frame lengths arrive on the same wire and the parser must tell them apart:

  * the 32-byte **streaming** feedback frame (body velocity + IMU + battery), which
    carries 0x00 in the byte after the 0x7b start marker (PDF p.20 "預留"), and
  * the 14-byte **readback reply** frames (0x33/0x34/0x35/0x36/0x50, PDF p.14-18),
    which carry their own command code in that same position.

The old parser assumed every frame was 32 bytes, so a single readback reply
desynced it for several frames afterwards. It now branches on that byte, which is
what makes read_motor_speeds() (per-wheel encoder rates, 0x36) usable while the
board is streaming.

`stats` counts frame outcomes. This exists because the board->Pi link moved from
USB (FTDI) to the Pi's GPIO UART on 2026-07-19, and raw UART jumpers next to
motor leads are exactly the kind of link that corrupts frames only once the
motors draw current. read_feedback() returns None for timeout / desync / bad BCC
alike, so without these counters a noisy link and an idle one look identical.
"""

import struct
import threading
import time

import serial

# Readback reply command codes (PDF p.14). 14-byte frames, as opposed to the
# 32-byte streaming feedback frame which has 0x00 in the same byte position.
REPLY_SYSTEM = 0x33     # 系統配置 (motor/encoder dir, PWM limits, encoder PPR)
REPLY_GEOMETRY = 0x34   # 小車尺寸參數 (wheel/axle space, gear ratio, diameter)
REPLY_BODY_VEL = 0x35   # 車體速度控制模式 (Vx, Vy, Vz)
REPLY_MOTOR_VEL = 0x36  # 各馬達獨立控制 with 編碼器 (M1..M4)  <-- per-wheel eye
REPLY_PID = 0x50        # PID 參數
REPLY_CODES = (REPLY_SYSTEM, REPLY_GEOMETRY, REPLY_BODY_VEL,
               REPLY_MOTOR_VEL, REPLY_PID)


class OminiBotHV:
    def __init__(self,
                 port='/dev/serial0',
                 baud=115200,
                 divisor_mode=4,
                 motor_direct=0,
                 encoder_direct=10,
                 motor_pwm_max=3600,
                 motor_pwm_min=2100,
                 encoder_ppr=165,
                 wheel_space=110,
                 axle_space=110,
                 gear_ratio=55,
                 wheel_diameter=60,
                 pos_kp=3000,
                 pos_ki=1050,
                 pos_kd=0,
                 vel_kp=3000,
                 vel_ki=1050,
                 open_retry_s=0.0):
        # exclusive=True (POSIX) so a second instance fails loudly with "port
        # busy" instead of silently sharing the port and corrupting each other's
        # reads -- that contention was killing odom and breaking the SLAM map.
        #
        # open_retry_s > 0 keeps retrying a busy port for that long before
        # giving up. Callers that own the base (driver_node) pass a few seconds:
        # oled_status_node reads the pack voltage off this same UART whenever no
        # driver is running, holding it exclusively for up to ~1.5 s, and a
        # bringup started inside that window would otherwise die on the spot.
        # A genuinely stuck second driver still fails, just a few seconds later.
        # Default 0.0 keeps the old fail-fast behaviour for diagnostic callers.
        deadline = time.monotonic() + open_retry_s
        while True:
            try:
                self.ser = serial.Serial(port, baud, timeout=1, exclusive=True)
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.5)
        self.robot_mode = divisor_mode
        self._write_lock = threading.Lock()

        # Serial link health. Incremented by read_feedback(); plain int += under
        # the GIL, so the read thread can count while another thread snapshots.
        # A rising bcc_fail/desync rate *while the motors are loaded* is the
        # signature of electrical noise on the GPIO-UART link (missing/thin
        # ground return, TX/RX routed alongside motor leads) -- not of a ROS,
        # calibration or mechanical fault.
        self.stats = {
            'good': 0,        # valid 32-byte streaming frame
            'bcc_fail': 0,    # framed correctly but checksum wrong -> corrupted bits
            'desync': 0,      # unexpected byte where a marker was expected (one per event)
            'resync_bytes': 0,  # bytes thrown away hunting for the next 0x7b
            'timeout': 0,     # nothing on the wire within the read timeout
            'short': 0,       # frame truncated mid-read
            'reply': 0,       # valid 14-byte readback reply
            'reply_bad': 0,   # readback reply with a bad checksum
        }
        # code -> {'payload': bytes, 'time': monotonic, 'bcc_ok': bool}
        self.last_reply = {}

        # The firmware needs settle time between config frames; without these
        # delays it never starts streaming feedback (matches the vendor example).
        self.forced_stop()
        time.sleep(0.5)

        # System setting (motor voltage range + encoder ppr).
        sys_set = bytearray(b'\x7b\x23')
        sys_set += motor_direct.to_bytes(1, 'big')
        sys_set += encoder_direct.to_bytes(1, 'big')
        sys_set += motor_pwm_max.to_bytes(2, 'big')
        sys_set += motor_pwm_min.to_bytes(2, 'big')
        sys_set += encoder_ppr.to_bytes(2, 'big')
        sys_set += bytearray(b'\x00\x00')
        self._send(sys_set)
        time.sleep(0.1)

        # Robot size setting (wheel/axle spacing mm, gear ratio, wheel diameter mm).
        bot_set = bytearray(b'\x7b\x24')
        bot_set += wheel_space.to_bytes(2, 'big')
        bot_set += axle_space.to_bytes(2, 'big')
        bot_set += gear_ratio.to_bytes(2, 'big')
        bot_set += wheel_diameter.to_bytes(2, 'big')
        bot_set += bytearray(b'\x00\x00')
        self._send(bot_set)
        time.sleep(0.1)

        # Closed-loop PID setting.
        pid_set = bytearray(b'\x7b\x40')
        pid_set += pos_kp.to_bytes(2, 'big')
        pid_set += pos_ki.to_bytes(2, 'big')
        pid_set += pos_kd.to_bytes(2, 'big')
        pid_set += vel_kp.to_bytes(2, 'big')
        pid_set += vel_ki.to_bytes(2, 'big')
        self._send(pid_set)
        time.sleep(0.1)

    @staticmethod
    def calculate_bcc(data):
        bcc = 0
        for byte in data:
            bcc ^= byte
        return bcc

    def _send(self, frame):
        """Append BCC + end byte and write the frame (thread-safe)."""
        frame = bytearray(frame)
        frame += self.calculate_bcc(frame).to_bytes(1, 'big')
        frame += bytearray(b'\x7d')
        with self._write_lock:
            self.ser.write(frame)

    def forced_stop(self):
        frame = bytearray(b'\x7b\x25\x00')
        frame += self.robot_mode.to_bytes(1, 'big')
        frame += bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
        self._send(frame)

    def robot_speed(self, lx, ly, az):
        """Body-frame velocity command: lx/ly in m/s, az in rad/s."""
        frame = bytearray(b'\x7b\x25\x02')
        frame += self.robot_mode.to_bytes(1, 'big')
        frame += struct.pack('!i', int(lx * 1000))[2:]
        frame += struct.pack('!i', int(ly * 1000))[2:]
        frame += struct.pack('!i', int(az * 1000))[2:]
        frame += bytearray(b'\x00\x00')
        self._send(frame)

    def motor_speed(self, m1, m2, m3, m4):
        """Per-wheel velocity command (rev/s * 1000)."""
        frame = bytearray(b'\x7b\x26\x02')
        frame += self.robot_mode.to_bytes(1, 'big')
        for m in (m1, m2, m3, m4):
            frame += struct.pack('!i', int(m * 1000))[2:]
        self._send(frame)

    def request_readback(self, code):
        """Ask the board to send one 14-byte readback reply (see REPLY_* codes).

        The reply arrives interleaved with the streaming feedback, so it is
        read_feedback() that picks it up and stashes it in self.last_reply.
        """
        self._send(bytearray([0x7b, code]) + bytearray(10))

    def read_motor_speeds(self, timeout=1.0):
        """Per-wheel encoder rates (m1..m4) via the 0x36 readback, or None.

        This is the only way to see the four wheels *individually*: the
        streaming frame reports body velocity, which the firmware has already
        collapsed from four encoders to three numbers via the mecanum forward
        kinematics (PDF p.33). That projection discards exactly one degree of
        freedom -- (V1 - V2 - V3 + V4), the wheel-desync/slip mode -- so
        "are the four wheels keeping up with each other?" is structurally
        unanswerable from /odom no matter how it is post-processed.

        Units are the board's own (raw/1000, "mm/s 或是 1000*r" per PDF p.10),
        NOT calibrated SI -- the board's internal scale is ~6.5x off for this
        chassis. That does not matter for a sync check, which compares the four
        against each other.

        NOTE: unverified on this board as of 2026-07-26. The vendor example only
        ever implements the 0x33/0x34/0x50 readbacks; 0x35/0x36 are documented
        (PDF p.14-15) but never exercised. Returns None if the board does not
        answer, so callers must handle that rather than assume support.
        """
        self.request_readback(REPLY_MOTOR_VEL)
        sent_at = time.monotonic()
        deadline = sent_at + timeout
        while time.monotonic() < deadline:
            self.read_feedback()    # drains the stream, stashes any reply
            reply = self.last_reply.get(REPLY_MOTOR_VEL)
            if reply and reply['time'] >= sent_at:
                p = reply['payload']    # p[0]=控制模式 p[1]=小車型別 p[2:]=M1..M4
                return tuple(
                    int.from_bytes(p[2 + 2 * i:4 + 2 * i], 'big', signed=True) / 1000.0
                    for i in range(4))
        return None

    def _read_reply(self, code):
        """Consume the rest of a 14-byte readback reply and stash it."""
        rest = self.ser.read(12)        # payload(10) + bcc(1) + 0x7d(1)
        if len(rest) < 12:
            self.stats['short'] += 1
            return
        payload, bcc, end = rest[0:10], rest[10], rest[11]
        bcc_ok = (self.calculate_bcc(bytearray([0x7b, code]) + payload) == bcc
                  and end == 0x7d)
        self.stats['reply' if bcc_ok else 'reply_bad'] += 1
        self.last_reply[code] = {
            'payload': payload,
            'time': time.monotonic(),
            'bcc_ok': bcc_ok,
        }

    def read_feedback(self):
        """Read one feedback frame from the streaming board.

        Streaming frame, after the 0x7b start byte:
          0x00(1) vel(6) imu(20) battery(2) bcc(1) 0x7d(1)  = 31 bytes

        A 14-byte readback reply (0x33/0x34/0x35/0x36/0x50 in place of that
        0x00) is consumed and stashed in self.last_reply instead; this method
        still returns None for it, so the caller's loop is unaffected.

        Returns dict {lx, ly, az, gyro_z, qw, qx, qy, qz, battery} on a valid
        streaming frame, or None on timeout / desync / bad checksum / readback
        reply (caller just retries). Every outcome bumps a self.stats counter --
        check those to tell a quiet link from a corrupted one.
        """
        start = self.ser.read(1)
        if start == b'':
            self.stats['timeout'] += 1
            return None
        if start != b'\x7b':
            # Landed mid-frame. Skip forward to the next start byte in ONE
            # read_until() rather than returning and discarding one byte per
            # call. Measured 2026-08-30: the board drops a few frames whenever
            # the driver writes a command (~6% at cmd_rate 20 Hz, bcc always 0,
            # so it is framing, not corruption). Byte-at-a-time recovery costs
            # one Python loop iteration per junk byte, and inside the ROS node
            # that loop competes for the GIL with the executor -- it fell behind
            # the 640 B/s stream, the kernel buffer grew, and a burst that lasts
            # ~40 bytes here turned into a self-sustaining cascade: 455 desyncs
            # and 4 good frames in 5 s, i.e. /odom froze for five seconds while
            # the robot was still driving. A stale odom->base_link TF for that
            # long is exactly what puts a permanent kink in a SLAM map.
            self.stats['desync'] += 1
            junk = self.ser.read_until(b'\x7b')
            self.stats['resync_bytes'] += len(junk)
            if not junk.endswith(b'\x7b'):
                return None      # timed out before any start byte showed up
            # else: fall through, we are positioned just after a start byte

        marker = self.ser.read(1)
        if len(marker) < 1:
            self.stats['short'] += 1
            return None
        if marker[0] in REPLY_CODES:
            self._read_reply(marker[0])
            return None
        if marker[0] != 0x00:
            self.stats['desync'] += 1
            return None

        body = self.ser.read(29)     # vel(6) imu(20) battery(2) bcc(1)
        if len(body) < 29:
            self.stats['short'] += 1
            return None
        end = self.ser.read(1)
        if end != b'\x7d':
            self.stats['desync'] += 1
            return None

        robot_vel = body[0:6]
        imu_val = body[6:26]
        bat = body[26:28]
        bcc = body[28]

        check = bytearray(b'\x7b\x00') + robot_vel + imu_val + bat
        if self.calculate_bcc(check) != bcc:
            self.stats['bcc_fail'] += 1
            return None
        self.stats['good'] += 1

        def s16(b):
            return int.from_bytes(b, 'big', signed=True) / 1000.0

        return {
            'lx': s16(robot_vel[0:2]),
            'ly': s16(robot_vel[2:4]),
            'az': s16(robot_vel[4:6]),
            # Raw MEMS gyro yaw rate (rad/s). imu_val layout: [0:6]=accel xyz,
            # [6:12]=gyro xyz, [12:20]=quat wxyz. gyro Z ([10:12]) is a direct
            # yaw-rate measurement -- immune to mecanum wheel slip, unlike az.
            'gyro_z': s16(imu_val[10:12]),
            'qw': s16(imu_val[12:14]),
            'qx': s16(imu_val[14:16]),
            'qy': s16(imu_val[16:18]),
            'qz': s16(imu_val[18:20]),
            'battery': s16(bat),
        }

    def reset_stats(self):
        """Zero the link counters, e.g. at the start of each measured phase."""
        for key in self.stats:
            self.stats[key] = 0

    def close(self):
        try:
            self.forced_stop()
        finally:
            self.ser.close()
