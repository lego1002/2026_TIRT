#!/usr/bin/env python3
"""在 Pi 上錄診斷用 rosbag —— 取代壞掉的 `ros2 bag record`。

**為什麼需要這支**:Pi 上的 ROS 2 CLI graph 查詢在 Discovery Server 架構下是瞎的
(`ros2 topic list` 只印得出 /parameter_events 和 /rosout,即使 bringup 正在狂發資料)。
`ros2 bag record` 靠的正是同一套查詢去找 publisher,所以它會**安靜地錄出一個空 bag**
—— 2026-07-25 實際踩到:錄完 60 秒,db3 裡 0 個 topic、0 筆訊息,而且完全沒有錯誤訊息。

普通的 rclpy 訂閱者不受影響(直接按 topic 名稱配對 endpoint,不需要 graph),
所以這支就用 rclpy 訂閱,再用 rosbag2_py 寫出標準格式的 bag,之後照樣能 `ros2 bag play`。

用法(Pi 上跑,bringup 已經起來):
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 tools/record_diag.py ~/bags/diag1        # Ctrl-C 結束

錄的時候的標準動作(慢!每段之間務必停好停滿,靜止段是用來切分段落的):
    1. 靜止 5 秒
    2. 原地慢轉整整 360°(≤0.3 rad/s),停
    3. 靜止 5 秒
    4. 朝一面牆直線前進約 1 m(≤0.15 m/s),停
    5. 靜止 5 秒
"""

import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

TOPICS = [
    ('/scan', LaserScan, 'sensor_msgs/msg/LaserScan', False),
    ('/odom', Odometry, 'nav_msgs/msg/Odometry', False),
    ('/tf', TFMessage, 'tf2_msgs/msg/TFMessage', False),
    ('/tf_static', TFMessage, 'tf2_msgs/msg/TFMessage', True),  # latched
]


class Recorder(Node):
    def __init__(self, out_path):
        super().__init__('record_diag')
        self.writer = SequentialWriter()
        self.writer.open(StorageOptions(uri=out_path, storage_id='sqlite3'),
                         ConverterOptions('', ''))
        self.counts = {}

        for name, msg_type, type_str, latched in TOPICS:
            self.writer.create_topic(TopicMetadata(
                name=name, type=type_str, serialization_format='cdr'))
            self.counts[name] = 0
            # /tf_static 是 latched 發布,QoS 必須用 TRANSIENT_LOCAL 才收得到,
            # 否則錄出來的 bag 會缺 base_link->laser_frame,重播時整條 TF 鏈斷掉。
            qos = QoSProfile(
                depth=200,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                            else DurabilityPolicy.VOLATILE),
            )
            self.create_subscription(
                msg_type, name,
                lambda msg, n=name: self._write(n, msg), qos)

        print(f'錄製中 → {out_path}   (Ctrl-C 結束)', file=sys.stderr)
        self.create_timer(2.0, self._tick)

    def _write(self, name, msg):
        self.writer.write(name, serialize_message(msg),
                          self.get_clock().now().nanoseconds)
        self.counts[name] += 1

    def _tick(self):
        line = '  '.join(f'{k} {v}' for k, v in self.counts.items())
        print(f'\r{line}', end='', flush=True)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'diag_bag'
    rclpy.init()
    node = Recorder(out)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException,
            rclpy._rclpy_pybind11.RCLError):
        pass
    finally:
        print()
        total = sum(node.counts.values())
        if total == 0:
            print('一筆都沒收到 —— bringup 沒在跑,或 topic 名稱不對。')
        else:
            for k, v in node.counts.items():
                print(f'  {k:12s} {v:6d} 筆')
        del node.writer          # 收尾寫出 metadata.yaml
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
