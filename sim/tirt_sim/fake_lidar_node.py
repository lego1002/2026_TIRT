"""假光達 —— 模擬版的 sllidar_node(RPLidar C1)。

發布和真光達同樣的 /scan(frame_id=laser_frame,10Hz,360°),所以
slam_toolbox 和 Nav2 收到的東西在格式上完全一致。

--- 最重要的設計:「真實外參」和「你以為的外參」是兩個獨立參數 ---

真車上有兩個東西描述光達裝在哪:

    (a) 光達實體裝的位置/角度        <- 物理現實,你不能改,只能量
    (b) base_link -> laser_frame TF  <- 你在 robot_bringup.launch.py 填的數字

SLAM 用的是 (b)。當 (a) != (b),SLAM 就會拿著錯的假設去比對掃描,結果是
地圖被扇形塗抹、牆變雙線、走廊彎折 —— 而且**畫面上不會有任何錯誤訊息**。
2026-07-25 就是這樣:光達幾乎是反裝的(真實 +167°),launch 檔卻填 0°。

所以這裡把 (a) 做成 sim_laser_* 參數(模擬器用來產生 scan),(b) 仍然由
sim_bringup.launch.py 的 laser_* 參數餵給 static_transform_publisher。
兩個設成不一樣,你就能在安全的地方**親眼看到那個 bug 長什麼樣**,
然後練習用 tools/scan_match_check.py 把它抓出來。

預設兩者相同(= 已校準好的車),要重現 bug 請用 sim/faults.md 裡的處方。
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tirt_sim.maze import load_world


class FakeLidar(Node):

    def __init__(self):
        super().__init__('fake_lidar')

        self.declare_parameter('world', '')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('rate', 10.0)          # C1 標準轉速
        self.declare_parameter('samples', 450)        # C1 @10Hz 約 450 點/圈
        self.declare_parameter('range_min', 0.15)     # 對齊 mapper_params 的 min_laser_range
        self.declare_parameter('range_max', 12.0)     # C1 規格

        # 光達「實際」裝在哪(相對 base_link)。這是物理現實,不是 TF。
        self.declare_parameter('sim_laser_x', 0.014)
        self.declare_parameter('sim_laser_y', -0.014)
        self.declare_parameter('sim_laser_yaw', 0.0)

        # 感測雜訊
        self.declare_parameter('range_noise', 0.005)  # 每點高斯雜訊(m),C1 約 ±0.5cm
        self.declare_parameter('dropout_prob', 0.01)  # 白牆反光造成的無回波比例

        p = self.get_parameter
        world_path = p('world').value
        if not world_path:
            raise RuntimeError('fake_lidar 需要 world:=<迷宮 yaml 路徑>')
        self.world = load_world(world_path)

        self.frame = p('frame_id').value
        self.n = int(p('samples').value)
        self.rmin = p('range_min').value
        self.rmax = p('range_max').value
        self.lx = p('sim_laser_x').value
        self.ly = p('sim_laser_y').value
        self.lyaw = p('sim_laser_yaw').value
        self.noise = p('range_noise').value
        self.drop = p('dropout_prob').value

        # 角度陣列固定不變,先算好。-pi ~ +pi,和 C1 一樣繞一整圈。
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / self.n
        self.angles = (self.angle_min +
                       np.arange(self.n, dtype=np.float32) * self.angle_inc)

        self.pose = None
        self.create_subscription(Odometry, '/sim/ground_truth', self._truth_cb, 10)

        # /scan 用 **RELIABLE**(預設 QoS),不是 SensorDataQoS。
        #
        # 這裡踩過一次(2026-08-01):原本用 qos_profile_sensor_data(BEST_EFFORT),
        # 結果 RViz 的 LaserScan 顯示完全空白 —— RViz 預設用 RELIABLE 訂閱,而
        # 「RELIABLE 訂閱者」和「BEST_EFFORT 發布者」是**不相容**的,DDS 直接不配對,
        # 兩邊都不會報錯,畫面就只是安靜地什麼都沒有。
        #
        # 選 RELIABLE 的兩個理由:
        #   1. 真的 sllidar_node 就是用預設 QoS 發布的,rviz/view_robot.rviz 在實機上
        #      看得到雷射點就是證據。模擬的職責是**照抄真車**,不是選理論上比較對的那個。
        #   2. RELIABLE 發布者可以同時滿足 RELIABLE 和 BEST_EFFORT 的訂閱者(相容方向是
        #      發布端比訂閱端嚴格即可),所以 slam_toolbox / Nav2 兩種寫法都吃得到。
        # 想試 BEST_EFFORT 的行為差異就設 scan_best_effort:=true。
        self.declare_parameter('scan_best_effort', False)
        scan_qos = (qos_profile_sensor_data
                    if self.get_parameter('scan_best_effort').value else 10)
        self.pub = self.create_publisher(LaserScan, 'scan', scan_qos)

        rate = p('rate').value
        self.scan_time = 1.0 / rate
        self.create_timer(self.scan_time, self._scan)

        self.get_logger().info(
            f'假光達啟動 | {self.n} 點 @ {rate:.0f}Hz | '
            f'實際外參 x={self.lx:.3f} y={self.ly:.3f} yaw={math.degrees(self.lyaw):.1f}°')

    def _truth_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _scan(self):
        if self.pose is None:
            return
        bx, by, byaw = self.pose

        # base_link -> 光達實際位置。注意這裡用的是 sim_laser_*(物理現實),
        # 而 TF 樹上發布的是 laser_*(你以為的)。兩者不同 = 重現外參 bug。
        c, s = math.cos(byaw), math.sin(byaw)
        lx = bx + self.lx * c - self.ly * s
        ly = by + self.lx * s + self.ly * c
        lyaw = byaw + self.lyaw

        rng = self.world.raycast(lx, ly, lyaw, self.angles, self.rmax)

        if self.noise:
            rng = rng + np.random.normal(0.0, self.noise, size=rng.shape)
        if self.drop:
            mask = np.random.random(rng.shape) < self.drop
            rng[mask] = np.inf

        # 量程外一律填 inf。LaserScan 的規範是 <range_min 或 >range_max 的值
        # 應被使用端丟棄;填 inf 最不會被誤讀成「那裡有牆」。
        rng[(rng < self.rmin) | (rng > self.rmax)] = np.inf

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_min + (self.n - 1) * self.angle_inc)
        msg.angle_increment = float(self.angle_inc)
        msg.time_increment = float(self.scan_time / self.n)
        msg.scan_time = float(self.scan_time)
        msg.range_min = float(self.rmin)
        msg.range_max = float(self.rmax)
        msg.ranges = rng.astype(np.float32).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    # 被 SIGTERM(pkill)關掉時不要吐 traceback,理由同 fake_base_node。
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
