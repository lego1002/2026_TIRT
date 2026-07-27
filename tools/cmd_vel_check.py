#!/usr/bin/env python3
"""量 /cmd_vel 的到達節奏 —— 用來證明「一頓一頓」是不是網路造成的。

為什麼需要這個:`ros2 topic hz /cmd_vel` 只給你一個平均值,而抖動的本質是
**偶爾一次很長的空檔**。平均 20 Hz 的訊號完全可以每隔幾秒斷 0.8 秒一次,
hz 看起來卻很漂亮。這支專門找那些空檔,並且直接告訴你有幾次超過 driver 的
watchdog(cmd_vel_timeout),因為每一次超過就是底盤被歸零一次 = 你感覺到的一頓。

用法(**在 Pi 上跑**,那才是命令實際被收到的地方):
    ./robotctl shell                       # 或直接 ssh 進 Pi
    cd ~/2026_TIRT
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source dds/setup_dds.sh
    python3 tools/cmd_vel_check.py         # 然後去遙控車子跑 30 秒,Ctrl-C 看結果

    python3 tools/cmd_vel_check.py --timeout 1.0    # 對齊你實際設的 cmd_vel_timeout

--- 怎麼判讀 ---

  p50 ≈ 1/發布頻率(teleop 是 ~0.05s),而且 p99 / 最大值也差不多 → 鏈路健康。
  p50 正常但最大值遠大於 timeout,且「超過 watchdog 次數」> 0
      → 就是它。每一次都對應底盤被歸零一次。
  情境比較才有意義:
      teleop 跑在 Pi 上(現在的預設)→ 應該完全沒有超時,最大值貼著 p50。
      teleop 跑在筆電上 → 若出現超時,證實問題在 WiFi 傳輸,不是底盤或韌體。

發布端 QoS 是 BEST_EFFORT(driver 和 teleop 都是),所以這支也用 BEST_EFFORT 訂閱,
否則 QoS 不相容會一筆都收不到。
"""

import argparse
import statistics
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class CmdVelCheck(Node):
    def __init__(self, timeout):
        super().__init__('cmd_vel_check')
        self.timeout = timeout
        self.gaps = []
        self.last = None
        self.n = 0
        self.n_moving = 0
        qos = QoSProfile(depth=10,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, '/cmd_vel', self._cb, qos)
        self.create_timer(2.0, self._tick)
        print(f'訂閱 /cmd_vel(BEST_EFFORT),watchdog 門檻 = {timeout:.2f}s。'
              '開始遙控,結束按 Ctrl-C。\n')

    def _cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is not None:
            self.gaps.append(now - self.last)
        self.last = now
        self.n += 1
        # 「有在下指令」的樣本才算數:停著不動時 teleop 送的全零指令雖然也會餵飽
        # watchdog,但那段時間的空檔對「開起來會不會頓」沒有意義。
        if abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z) > 1e-6:
            self.n_moving += 1

    def _tick(self):
        if not self.gaps:
            print('\r尚未收到 /cmd_vel …', end='', flush=True)
            return
        recent = self.gaps[-100:]
        over = sum(1 for g in self.gaps if g > self.timeout)
        print(f'\r收到 {self.n} 筆(其中 {self.n_moving} 筆非零) | '
              f'近期間隔 p50 {statistics.median(recent) * 1000:6.1f} ms  '
              f'max {max(recent) * 1000:7.1f} ms | '
              f'超過 watchdog {over} 次   ', end='', flush=True)

    def report(self):
        print('\n')
        if len(self.gaps) < 2:
            print('樣本太少,沒收到足夠的 /cmd_vel。檢查:')
            print('  - teleop 真的有在跑嗎?(./robotctl status 看 teleop 視窗)')
            print('  - DDS 通嗎?(ros2 topic list 看得到 /cmd_vel 嗎)')
            return
        g = sorted(self.gaps)
        def pct(p):
            return g[min(len(g) - 1, int(len(g) * p))]
        over = [x for x in self.gaps if x > self.timeout]
        print('=' * 58)
        print(f'  樣本數        {self.n}  (非零指令 {self.n_moving})')
        print(f'  到達間隔 p50  {pct(0.50) * 1000:8.1f} ms   '
              f'(= 發布頻率 {1.0 / pct(0.50):.1f} Hz)')
        print(f'           p95  {pct(0.95) * 1000:8.1f} ms')
        print(f'           p99  {pct(0.99) * 1000:8.1f} ms')
        print(f'           max  {max(g) * 1000:8.1f} ms')
        print(f'  超過 watchdog({self.timeout:.2f}s)  {len(over)} 次')
        print('=' * 58)
        if over:
            print(f'\n→ 有 {len(over)} 次空檔超過 watchdog,每一次底盤都被歸零一次 ——')
            print('  這就是「一頓一頓」。如果 teleop 是跑在筆電上,把它搬到 Pi 上跑')
            print('  (./robotctl up teleop,或直接用 ./gcs.sh)即可根治。')
            print(f'  最長的三次空檔: {", ".join(f"{x*1000:.0f} ms" for x in sorted(over)[-3:])}')
        elif max(g) > 4 * pct(0.50):
            print('\n→ 沒有觸發 watchdog,但最大空檔遠大於中位數,鏈路仍有抖動。')
            print('  底盤應該不會頓,但若要求更穩,teleop 建議跑在 Pi 上。')
        else:
            print('\n→ 節奏平穩,沒有斷流。/cmd_vel 這一段是健康的;')
            print('  若車子還是會頓,那就不是網路問題,改查 tools/motor_diag.py。')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=1.0,
                    help='driver 的 cmd_vel_timeout(秒),預設 1.0')
    args = ap.parse_args()

    rclpy.init()
    node = CmdVelCheck(args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
