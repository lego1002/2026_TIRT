#!/usr/bin/env python3
"""即時顯示 /odom 的累積位移與轉角 —— 校 odom_linear_scale / gyro_scale 用。

為什麼需要這個:直接 `ros2 topic echo /odom` 出來是四元數,肉眼很難判斷「轉了幾度」;
而且要的是「從開始到現在累積了多少」,不是瞬時值。這支把兩者都算好印成一行。

用法(Pi 或 PC 都可以,走 DDS):
    source /opt/ros/humble/setup.bash
    source dds/setup_dds.sh            # PC 端記得 DDS_SERVER=<pi_ip>
    python3 tools/odom_check.py

按 Ctrl-C 結束時會印出總結。測試中想歸零重來,直接 Ctrl-C 再跑一次即可。

--- 校準流程 ---

先把兩個倍率歸 1,直接量板子的「原始」輸出,這樣算出來的倍率是一步到位的絕對值,
不必再乘現值(現值本身可能就是錯的,拿它當基準只會把錯誤帶進去):

    ./run_robot.sh odom_linear_scale:=1.0 gyro_scale:=1.0

A) 直線刻度(算 odom_linear_scale)
   1. 地上用捲尺標出 1.00 m 起訖點,車頭對準起點。
   2. 跑這支,teleop 慢速直線前進到終點停下(≤0.15 m/s)。
   3. 看 `直線位移` →  odom_linear_scale = 1.00 / 直線位移
      (工具結束時會直接幫你算好印出來)

B) 旋轉刻度(算 gyro_scale)
   1. 地上做個朝向記號,原地慢轉剛好 360°(慢!≤0.3 rad/s)回到原朝向。
   2. 看 `累積轉角` →  gyro_scale = 360 / |累積轉角|
   注意 `yaw(當前)` 和 `累積轉角` 不同:前者是現在朝哪(會繞回 ±180°),
   後者是一路轉過的總量(不繞回),校 360° 要看後者。

兩個值算出來後帶進去驗一次:
    ./run_robot.sh odom_linear_scale:=<A> gyro_scale:=<B>
再跑一次 A/B,這次應該讀到 1.00m 與 360°。確認無誤才寫回 launch 的 default_value。

注意 `minimum_travel_distance: 0.2` / `minimum_travel_heading: 0.17`:slam_toolbox 是看
**odom 說走了多遠**才決定收不收新的一幀。odom 低報 → 門檻跨不過去 → 掃到新地方地圖也不長。
所以這兩個倍率沒校準前,調 slam 參數是白費工。
"""

import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    """四元數 → yaw(rad)。driver 只繞 Z 轉,所以用簡化式就夠。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCheck(Node):
    def __init__(self):
        super().__init__('odom_check')
        self.first = None          # (x, y, yaw) 起始值
        self.prev = None           # 上一筆 (x, y, yaw),用來累加
        self.path_len = 0.0        # 走過的路徑總長(含彎路)
        self.turned = 0.0          # 累積轉角(不繞回,有正負)
        self.n = 0
        self.create_subscription(Odometry, '/odom', self._cb, 20)
        print('等 /odom ... (Ctrl-C 結束並印總結)\n', file=sys.stderr)

    def _cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        cur = (p.x, p.y, yaw)

        if self.first is None:
            self.first = cur
        else:
            px, py, pyaw = self.prev
            self.path_len += math.hypot(p.x - px, p.y - py)
            # 差值先正規化到 ±180°,再累加,才不會在 ±180° 交界跳一大圈
            d = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            self.turned += d
        self.prev = cur
        self.n += 1

        fx, fy, _ = self.first
        straight = math.hypot(p.x - fx, p.y - fy)
        t = msg.twist.twist

        print(f'\r直線位移 {straight:6.3f}m | 路徑長 {self.path_len:6.3f}m | '
              f'累積轉角 {math.degrees(self.turned):+8.1f}° | '
              f'yaw(當前) {math.degrees(yaw):+7.1f}° | '
              f'vx {t.linear.x:+6.3f} vy {t.linear.y:+6.3f} wz {t.angular.z:+6.3f}',
              end='', flush=True)

    def summary(self):
        print('\n')
        if self.first is None:
            print('沒收到任何 /odom —— 底盤驅動沒起來,或 DDS 沒連上。')
            print('  檢查: ros2 topic hz /odom   /   ros2 node list | grep ominibot')
            return
        fx, fy, _ = self.first
        px, py, _ = self.prev
        straight = math.hypot(px - fx, py - fy)
        deg = math.degrees(self.turned)
        print(f'收到 {self.n} 筆 /odom')
        print(f'  直線位移(起點→終點) : {straight:.3f} m')
        print(f'  路徑總長            : {self.path_len:.3f} m')
        print(f'  累積轉角            : {deg:+.1f}°')
        print()
        print('（下面的算式假設你跑 bringup 時已把兩個倍率設成 1.0；'
              '否則要再乘上當時用的值）')
        if straight > 0.01:
            print(f'  → 若實際走了 1.00m: odom_linear_scale = {1.0 / straight:.3f}')
        if abs(deg) > 1.0:
            print(f'  → 若實際轉了 360° : gyro_scale       = {360.0 / abs(deg):.3f}')


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException,
            rclpy._rclpy_pybind11.RCLError):
        pass  # Ctrl-C / context 被拆掉都算正常結束,直接去印總結
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
