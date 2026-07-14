"""Self-contained serial protocol for the CircusPi OminiBotHV controller board.

Distilled from OminiBotHV-master/example/OminiBot_HV_Meca.py so the driver package
does not depend on the vendor example being on the Python path. Frame format is
``\\x7b <cmd> ... <bcc> \\x7d`` with a big-endian XOR checksum.

Two threads use one instance: the ROS timer thread calls robot_speed()/forced_stop()
(writes), the read thread calls read_feedback() (reads). Writes are guarded by a lock;
pyserial allows a concurrent read on another thread.
"""

import struct
import threading
import time

import serial


class OminiBotHV:
    def __init__(self,
                 port='/dev/ominibot',
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
                 vel_ki=1050):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.robot_mode = divisor_mode
        self._write_lock = threading.Lock()

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

    def read_feedback(self):
        """Read one feedback frame from the streaming board.

        Frame after the 0x7b start byte:
          flag(1) vel(6) imu(20) battery(2) bcc(1) 0x7d(1)  = 31 bytes

        Returns dict {lx, ly, az, qw, qx, qy, qz, battery} on a valid frame,
        or None on timeout / desync / bad checksum (caller just retries).
        """
        start = self.ser.read(1)
        if start != b'\x7b':
            return None  # timeout (empty) or mid-frame byte; resync on next call
        body = self.ser.read(30)
        if len(body) < 30:
            return None
        end = self.ser.read(1)
        if end != b'\x7d':
            return None

        robot_vel = body[1:7]
        imu_val = body[7:27]
        bat = body[27:29]
        bcc = body[29]

        check = bytearray(b'\x7b\x00') + robot_vel + imu_val + bat
        if self.calculate_bcc(check) != bcc:
            return None

        def s16(b):
            return int.from_bytes(b, 'big', signed=True) / 1000.0

        return {
            'lx': s16(robot_vel[0:2]),
            'ly': s16(robot_vel[2:4]),
            'az': s16(robot_vel[4:6]),
            'qw': s16(imu_val[12:14]),
            'qx': s16(imu_val[14:16]),
            'qy': s16(imu_val[16:18]),
            'qz': s16(imu_val[18:20]),
            'battery': s16(bat),
        }

    def close(self):
        try:
            self.forced_stop()
        finally:
            self.ser.close()
