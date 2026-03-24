import serial
import struct
import time

class ominibothv:
    def __init__(self,
                 port = '/dev/ominibot',
                 baud = 115200,
                 divisor_mode = 4,
                 motor_direct = 0,
                 encoder_direct = 10,
                 motor_pwm_max = 3600,
                 motor_pwm_min = 2100,
                 encoder_ppr = 165,
                 wheel_space = 110,
                 axle_space = 110,
                 gear_ratio = 55,
                 wheel_diameter = 60,
                 pos_kp = 3000,
                 pos_ki = 1050,
                 pos_kd = 0,
                 vel_kp = 3000,
                 vel_ki = 1050):

        self.ser = serial.Serial(port, baud, timeout = 1)

        self.robot_mode = divisor_mode

        self.forced_stop()
        time.sleep(0.5)

        #system setting(motor range: 3v-6v, encoder ppr: 660/4 = 165)
        sys_set = bytearray(b'\x7b\x23')
        sys_set += motor_direct.to_bytes(1, byteorder='big')
        sys_set += encoder_direct.to_bytes(1, byteorder='big')
        sys_set += motor_pwm_max.to_bytes(2, byteorder='big')
        sys_set +=  motor_pwm_min.to_bytes(2, byteorder='big')
        sys_set +=  encoder_ppr.to_bytes(2, byteorder='big')
        sys_set += bytearray(b'\x00\x00')
        bcc = self.calculate_bcc(sys_set).to_bytes(1, byteorder='big')
        sys_set += bcc + bytearray(b'\x7d')
        self.ser.write(sys_set)
        time.sleep(0.1)

        # robot size setting(motor dear 1:55)
        bot_set = bytearray(b'\x7b\x24')
        bot_set += wheel_space.to_bytes(2, byteorder='big')
        bot_set += axle_space.to_bytes(2, byteorder='big')
        bot_set += gear_ratio.to_bytes(2, byteorder='big')
        bot_set += wheel_diameter.to_bytes(2, byteorder='big')
        bot_set += bytearray(b'\x00\x00')
        bcc = self.calculate_bcc(bot_set).to_bytes(1, byteorder='big')
        bot_set += bcc + bytearray(b'\x7d')
        self.ser.write(bot_set)
        time.sleep(0.1)


        # robot pid setting
        pid_set = bytearray(b'\x7b\x40')
        pid_set += pos_kp.to_bytes(2, byteorder='big')
        pid_set += pos_ki.to_bytes(2, byteorder='big')
        pid_set += pos_kd.to_bytes(2, byteorder='big')
        pid_set += vel_kp.to_bytes(2, byteorder='big')
        pid_set += vel_ki.to_bytes(2, byteorder='big')
        bcc = self.calculate_bcc(pid_set).to_bytes(1, byteorder='big')
        pid_set += bcc + bytearray(b'\x7d')
        self.ser.write(pid_set)
        time.sleep(0.1)


    def check_cmd(self, name):
        if name == 'system':
            self.ser.write(b'\x7b\x33\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x48\x7d')
        elif name == 'robot':
            self.ser.write(b'\x7b\x34\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x4f\x7d')
        else:
            self.ser.write(b'\x7b\x50\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x2b\x7d')

        read_val = self.ser.read(14)

        return read_val

    def calculate_bcc(self, data):
        bcc = 0
        for byte in data:
            bcc ^= byte
        return bcc

    def clamp_number(self, num , a, b):
        return max(min(num, max(a, b)), min(a, b))

    def forced_stop(self):
        set_motor_go = bytearray(b'\x7b\x25\x00')
        set_motor_go += self.robot_mode.to_bytes(1, byteorder='big')
        set_motor_go += bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
        bcc = self.calculate_bcc(set_motor_go).to_bytes(1, byteorder='big')
        set_motor_go += bcc + bytearray(b'\x7d')
        self.ser.write(set_motor_go)

    def motor_speed(self, m1, m2, m3, m4):
        set_motor_go = bytearray(b'\x7b\x26\x02')
        set_motor_go += self.robot_mode.to_bytes(1, byteorder='big')
        set_motor_go += struct.pack('!i',int(m1*1000))[2:]
        set_motor_go += struct.pack('!i',int(m2*1000))[2:]
        set_motor_go += struct.pack('!i',int(m3*1000))[2:]
        set_motor_go += struct.pack('!i',int(m4*1000))[2:]

        bcc = self.calculate_bcc(set_motor_go).to_bytes(1, byteorder='big')

        set_motor_go += bcc + bytearray(b'\x7d')

        self.ser.write(set_motor_go)

    def robot_speed(self, lx, ly, az):
        set_motor_go = bytearray(b'\x7b\x25\x02')
        set_motor_go += self.robot_mode.to_bytes(1, byteorder='big')
        set_motor_go += struct.pack('!i',int(lx*1000))[2:]
        set_motor_go += struct.pack('!i',int(ly*1000))[2:]
        set_motor_go += struct.pack('!i',int(az*1000))[2:]
        set_motor_go += bytearray(b'\x00\x00')

        bcc = self.calculate_bcc(set_motor_go).to_bytes(1, byteorder='big')

        set_motor_go += bcc + bytearray(b'\x7d')
        self.ser.write(set_motor_go)


    def serial_close(self):
        self.ser.close()

    def serial_write(self, cmd):
        self.ser.write(cmd)

    def serial_read(self, choose=None):
        self.ser.read(choose)
    def read_robot_data(self):
        while True:
            if self.ser.read().hex() == '7b':
                robot_vel = self.ser.read(7)[1:]
                imu_val = self.ser.read(20)
                bat_val = self.ser.read(3)
                if self.ser.read(1).hex() == '7d':
                    check_code = bytearray(b'\x7b\x00') + robot_vel + imu_val + bat_val[:2]
                    bcc = self.calculate_bcc(check_code)
                    if hex(bcc) == hex(bat_val[2]):
                        check = 1
                    else:
                        check = 0
                    break

        return check, robot_vel, imu_val, bat_val[:2]

import numpy as np

def quaternion_to_euler(q):
    # 歸一化四元數
    q = q / np.linalg.norm(q)

    # 計算歐拉角
    roll = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
    pitch = np.arcsin(2*(q[0]*q[2] - q[3]*q[1]))
    yaw = np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))

    # 將歐拉角從弧度轉換為角度
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw



if __name__ == '__main__':
    import time
    pi = ominibothv(
        port = 'COM22',
        baud = 115200,
        divisor_mode = 4,
        motor_direct = 0,
        encoder_direct = 10,
        motor_pwm_max = 3600,
        motor_pwm_min = 2100,
        encoder_ppr = 165,
        wheel_space = 110,
        axle_space = 110,
        gear_ratio = 55,
        wheel_diameter = 60,
        pos_kp = 3000,
        pos_ki = 1050,
        pos_kd = 0,
        vel_kp = 3000,
        vel_ki = 1050)

    pi.robot_speed(0.08, 0.0, 0.0)

    ot = time.ctime(time.time())
    t1 = time.time()
    while True:

        checkCmd, robot_vel, imu_val, bat_val = pi.read_robot_data()

        battry = int.from_bytes(bat_val, byteorder='big', signed=True)/1000
        print(battry)

        lx = int.from_bytes(robot_vel[:2], byteorder='big', signed=True)/1000
        ly = int.from_bytes(robot_vel[2:4], byteorder='big', signed=True)/1000
        az = int.from_bytes(robot_vel[4:], byteorder='big', signed=True)/1000
        print(lx, ly, az)

        qw = int.from_bytes(imu_val[12:14], byteorder='big', signed=True)/1000
        qx = int.from_bytes(imu_val[14:16], byteorder='big', signed=True)/1000
        qy = int.from_bytes(imu_val[16:18], byteorder='big', signed=True)/1000
        qz = int.from_bytes(imu_val[18:], byteorder='big', signed=True)/1000
        print(qw, qx, qy, qz)

        quaternion = np.array([qw, qx, qy, qz])
        roll, pitch, yaw = quaternion_to_euler(quaternion)
        # print("Yaw: ", yaw)

        if time.time() - t1 >= 3:
            break

    pi.robot_speed(0.0, 0.0, 0.0)
