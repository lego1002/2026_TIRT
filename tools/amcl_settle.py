#!/usr/bin/env python3
"""讓 AMCL 在「車子不動」的情況下收斂定位,並印出它對自己位置有多確定。

用法(導航模式開著時,Pi 上):
    python3 tools/amcl_settle.py          # 預設最多 30 次
    python3 tools/amcl_settle.py 50

為什麼需要這支:AMCL 只有在車子移動超過 update_min_d / update_min_a 時才會拿光達
去比對地圖。剛用 2D Pose Estimate 放好位置、車子停著不動,粒子雲(RViz 裡的綠色
箭頭)就永遠散成一大片 —— 看起來像定位壞了,其實只是它還沒機會比對。
/request_nomotion_update 讓 AMCL「不動也比對一次」,連續叫幾次粒子就會收攏。

判讀:印出的是 AMCL 自己估的標準差。位置 < 5 cm 且角度 < 5° 就算定好了。
叫了很多次還是很散 → 2D Pose Estimate 放的位置/朝向離真實差太多,重放一次;
或這個位置的牆太少/太對稱,光達看到的東西不足以分辨。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty

GOOD_XY = 0.05      # m
GOOD_YAW = 5.0      # deg


class Settle(Node):
    def __init__(self):
        super().__init__('amcl_settle')
        self.pose = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._cb, qos)
        self.cli = self.create_client(Empty, '/request_nomotion_update')

    def _cb(self, msg):
        self.pose = msg

    def spread(self):
        c = self.pose.pose.covariance
        return math.sqrt(c[0]), math.sqrt(c[7]), math.degrees(math.sqrt(c[35]))


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rclpy.init()
    node = Settle()
    if not node.cli.wait_for_service(timeout_sec=5.0):
        print('找不到 AMCL(/request_nomotion_update)。導航模式有開嗎?')
        return 1

    for i in range(1, n + 1):
        node.pose = None
        rclpy.spin_until_future_complete(node, node.cli.call_async(Empty.Request()), timeout_sec=3.0)
        # AMCL 處理完下一張 scan 才會發新的 /amcl_pose
        t0 = time.time()
        while node.pose is None and time.time() - t0 < 2.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            continue
        sx, sy, syaw = node.spread()
        p = node.pose.pose.pose
        print(f'{i:3d}  位置不確定 ±{max(sx, sy) * 100:5.1f} cm   角度 ±{syaw:5.1f}°   '
              f'(x={p.position.x:+.2f} y={p.position.y:+.2f})')
        if max(sx, sy) < GOOD_XY and syaw < GOOD_YAW:
            print('\n定位好了 —— RViz 裡綠色箭頭應該已經縮成車子身上一小團。')
            return 0

    print('\n還沒收斂。2D Pose Estimate 再放準一點(位置和朝向都要對),然後再跑一次這支。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
