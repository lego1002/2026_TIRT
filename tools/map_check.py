#!/usr/bin/env python3
"""監看 /map 到底有沒有在長大 —— 診斷「scan 掃到新地方但地圖沒更新」用。

`ros2 topic hz /map` 只能告訴你「有沒有在發」,但 slam_toolbox 就算完全沒收新 scan
也會照著 map_update_interval 週期性重發同一張圖 —— 看起來很正常,其實是死的。
這支直接數格子:未知 / 空曠 / 佔有 各幾格,以及每次更新變化了多少。

用法(PC 端,SLAM 跑起來之後):
    source /opt/ros/humble/setup.bash
    source dds/setup_dds.sh          # 記得 DDS_SERVER=<pi_ip>
    python3 tools/map_check.py

然後 teleop 把車開到沒掃過的地方,看這裡的數字。

怎麼判讀:
  * `佔有` 和 `空曠` 持續增加  → 地圖正常在長,問題不在 SLAM 收不收 scan。
  * 尺寸和格數完全不動        → slam_toolbox 沒有加入新節點。往這三個方向查:
      1. slam_toolbox 那個終端有沒有狂噴 "Message Filter dropping message" /
         "queue is full" → scan 根本沒進去(WiFi 抖動害 TF 等不到,見
         0721_net_issue_plan.md)。
      2. odom 有沒有在動:另開 tools/odom_check.py,確認車動時 `路徑長` 有在增加。
         slam_toolbox 是看 odom 位移超過 minimum_travel_distance(0.2m)/
         minimum_travel_heading(0.17rad) 才收新的一幀。
      * 完全收不到 /map(一直停在「等 /map」)→ SLAM 沒跑,或 DDS 沒連上,
        或 QoS 不合(本工具已用 TRANSIENT_LOCAL,和 slam_toolbox 的 latched 發布相符)。
"""

import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapCheck(Node):
    def __init__(self):
        super().__init__('map_check')
        # slam_toolbox 用 latched(transient local)發布 /map,訂閱端 QoS 必須相符,
        # 否則會「topic 看得到但收不到資料」。
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.prev = None
        self.n = 0
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        print('等 /map ... (Ctrl-C 結束)\n', file=sys.stderr)

    def _cb(self, msg):
        info = msg.info
        unknown = free = occ = 0
        for v in msg.data:
            if v < 0:
                unknown += 1
            elif v < 50:
                free += 1
            else:
                occ += 1

        self.n += 1
        cur = (info.width, info.height, free, occ)
        if self.prev is None:
            delta = '  (第一張)'
        elif cur == self.prev:
            delta = '  ← 完全沒變'
        else:
            _, _, pf, po = self.prev
            delta = f'  Δ空曠 {free - pf:+d}  Δ佔有 {occ - po:+d}'
        self.prev = cur

        area = info.width * info.height * info.resolution ** 2
        print(f'#{self.n:3d} {info.width}x{info.height} @{info.resolution:.3f}m '
              f'({area:.1f}m²) | 未知 {unknown:7d} 空曠 {free:6d} 佔有 {occ:5d}'
              f'{delta}', flush=True)


def main():
    rclpy.init()
    node = MapCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException,
            rclpy._rclpy_pybind11.RCLError):
        pass
    finally:
        if node.n == 0:
            print('\n完全沒收到 /map —— SLAM 沒跑、DDS 沒連上,或 QoS 不合。')
            print('  檢查: ros2 topic list | grep map  /  PC 端 ./run_slam.sh 有沒有在跑')
        else:
            print(f'\n共收到 {node.n} 張 /map。')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
