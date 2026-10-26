#!/usr/bin/env python3
"""選單「健康檢查」的量測部分:一個節點同時數 /scan、/odom 的到達頻率,順便讀電池電壓。

    python3 tools/health_check.py [秒數,預設 5]

輸出是 key=value,一行一個(scan_hz / odom_hz / batt_v),給 menu.sh 解析;
沒收到的值是空的。

為什麼不用 `ros2 topic hz`:選單原本在背景同時開兩個 hz + 一個 echo,用
`timeout -s INT` 收尾。非互動 bash 的背景工作會把 SIGINT 設成忽略,hz 的輸出又是
區塊緩衝 —— 結果時好時壞,/odom 明明 20 Hz 卻常常回報「沒有資料」(2026-10-26)。
一個行程、一次 discovery、自己數訊息,就沒有這些問題。
"""
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class Health(Node):
    def __init__(self):
        super().__init__('menu_health_check')
        self.stamps = {'scan': [], 'odom': []}
        self.batt = None
        # BEST_EFFORT 訂閱可以配對 RELIABLE 和 BEST_EFFORT 兩種發布者,反過來不行。
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.stamps['scan'].append(time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.stamps['odom'].append(time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/battery_voltage', self._batt, qos_profile_sensor_data)

    def _batt(self, m):
        self.batt = m.data


def rate(ts):
    # 用第一則到最後一則之間的間隔算,不受 discovery 花了多久影響。
    if len(ts) < 3:
        return ''
    return f'{(len(ts) - 1) / (ts[-1] - ts[0]):.1f}'


def main():
    window = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Health()
    end = time.monotonic() + window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'scan_hz={rate(node.stamps["scan"])}')
    print(f'odom_hz={rate(node.stamps["odom"])}')
    print(f'batt_v={"" if node.batt is None else f"{node.batt:.2f}"}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
