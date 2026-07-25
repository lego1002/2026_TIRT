#!/usr/bin/env python3
"""量測底盤的速度階躍響應 —— 調 PID / 判斷「卡頓」到底是什麼造成的。

為什麼需要這支:憑手感邊開邊調 PID 是最沒效率的做法。手感沒有數字、不可重現,
而且「怪怪的」同時符合太多原因(PID 震盪、電池電壓垂降、齒輪背隙、馬達過熱),
光靠開車的感覺永遠分不出是哪一個。這支給你四個可以互相比較的數字。

做法:對 /cmd_vel 送**方波**(+v / -v 交替),同時記錄 /odom 回報的實際速度,
然後算出每一次換向的:

    死區時間 (dead time)  下令換向 → 實際速度真的反向,中間過了多久
                          (解析度只有 ~50 ms,因為 /odom 才 20 Hz;而且量法本身
                           會系統性高估約 50 ms。看趨勢,不要看絕對值。)
    上升時間 (rise time)  10% → 90% 目標速度所花的時間
    超越量 (overshoot)    衝過頭多少 %
    穩態抖動 (ripple)     到達穩態後速度的標準差

用 +v/-v 交替而不是 0/+v,有兩個理由:
  1. 車子原地來回,不會一路開走,室內小空間也能測。
  2. **換向正是背隙會現形的地方**。齒輪有背隙的話,下令反轉之後齒還要先空轉
     過一段間隙才咬到,死區時間會明顯變長。所以這支順便直接檢驗背隙假設。

--- 怎麼判讀 ---

超越量大 + 穩態抖動大 + 聽得到高頻嗡嗡聲   → vel_kp 太高,砍半
到達穩態後才慢慢震盪、或停下來還會抽動      → vel_ki 太高(積分飽和),砍半
上升時間很長、追不上目標速度                → 增益太低,或電力不足(看電壓)
**死區時間長**(> ~200ms)且其他都正常       → 機械背隙,調 PID 救不了
四個數字冷機、熱機差很多                    → 電池垂降或發熱,不是 PID 問題

--- 用法 ---

    source /opt/ros/humble/setup.bash
    source dds/setup_dds.sh            # PC 端記得 DDS_SERVER=<pi_ip>
    python3 tools/step_response.py                    # 預設:直線 0.15 m/s
    python3 tools/step_response.py --axis yaw         # 改測原地旋轉
    python3 tools/step_response.py --speed 0.2 --cycles 8

⚠️  車子會自己動(前後來回約 ±0.2 m)。跑之前先確認周圍淨空、teleop 已關掉
    (兩個節點同時發 /cmd_vel 會打架)。任何時候 Ctrl-C 會立刻送零速度停車。

--- 有效率的調法 ---

1. 先量一次當基準,把數字抄下來。
2. **一次只動一個增益,而且用砍半法**(3000 → 1500 → 750 …),不要小幅微調 ——
   小幅微調的差異會被雜訊蓋掉,看不出來。
3. 每改一次就重跑這支,比同樣那四個數字。
4. 找到最好的那一組之後,再回頭把另一個增益也砍半試一次。

    ./run_robot.sh vel_kp:=1500 vel_ki:=1050
    ./run_robot.sh vel_kp:=1500 vel_ki:=500

   (PID 是在節點啟動時一次寫進板子韌體的,所以每次都要重跑 run_robot.sh。)

5. 冷機測一次、連續開 10 分鐘之後再測一次。數字漂掉 → 去看 /battery_voltage,
   那就不是 PID 的問題。
"""

import argparse
import math
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class StepResponse(Node):
    def __init__(self, axis, speed, hold, cycles):
        super().__init__('step_response')
        self.axis = axis
        self.speed = speed
        self.hold = hold
        self.cycles = cycles

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 50)
        self.create_subscription(Float32, 'battery_voltage', self._batt_cb, 10)

        self.samples = []       # (t, measured) — 一路累積,結束後才分析
        self.batt = []          # (t, volts)
        self._lock = threading.Lock()
        self._target = 0.0
        self.t0 = time.monotonic()

    def _odom_cb(self, msg):
        v = (msg.twist.twist.angular.z if self.axis == 'yaw'
             else msg.twist.twist.linear.x)
        with self._lock:
            self.samples.append((time.monotonic() - self.t0, v, self._target))

    def _batt_cb(self, msg):
        self.batt.append((time.monotonic() - self.t0, msg.data))

    def _send(self, v):
        t = Twist()
        if self.axis == 'yaw':
            t.angular.z = v
        else:
            t.linear.x = v
        with self._lock:
            self._target = v
        self.pub.publish(t)

    def run(self):
        unit = 'rad/s' if self.axis == 'yaw' else 'm/s'
        print(f'方波測試: {self.axis} 軸, ±{self.speed} {unit}, '
              f'每段 {self.hold}s, {self.cycles} 個週期')
        print('等 /odom 上線 …')
        deadline = time.monotonic() + 10.0
        while not self.samples and time.monotonic() < deadline:
            time.sleep(0.1)
        if not self.samples:
            print('✗ 收不到 /odom。確認 ./run_robot.sh 有在跑、DDS_SERVER 設對了。')
            return False

        # 換向清單: +v, -v, +v, -v, ... 每個週期兩段。
        self.edges = []   # (切換時刻, 切換前的目標, 切換後的目標)
        prev = 0.0
        for i in range(self.cycles * 2):
            v = self.speed if i % 2 == 0 else -self.speed
            self.edges.append((time.monotonic() - self.t0, prev, v))
            self._send(v)
            # 指令要持續重發,不然驅動的 cmd_vel watchdog(0.5s)會把車停掉。
            end = time.monotonic() + self.hold
            while time.monotonic() < end:
                self._send(v)
                time.sleep(0.05)
            prev = v
        self._send(0.0)
        for _ in range(10):
            self._send(0.0)
            time.sleep(0.05)
        return True

    # -- 分析 ---------------------------------------------------------------
    def report(self):
        with self._lock:
            samples = list(self.samples)
        if len(samples) < 20:
            print('樣本太少,無法分析。')
            return

        print()
        print('=' * 68)
        print(f'{"#":>3} {"目標":>8} {"死區(ms)":>9} {"上升(ms)":>9} '
              f'{"超越(%)":>8} {"穩態":>8} {"抖動":>7}')
        print('-' * 68)

        dead_times, rises, overshoots, ripples, steadies = [], [], [], [], []
        for i, (t_edge, before, after) in enumerate(self.edges):
            seg = [(t, v) for t, v, _ in samples if t_edge <= t < t_edge + self.hold]
            if len(seg) < 5:
                continue
            r = self._analyse(seg, t_edge, before, after)
            if r is None:
                continue
            dead, rise, over, steady, ripple = r
            dead_times.append(dead)
            rises.append(rise)
            overshoots.append(over)
            ripples.append(ripple)
            steadies.append(steady)
            print(f'{i:>3} {after:>8.3f} {dead*1000:>9.0f} {rise*1000:>9.0f} '
                  f'{over:>8.1f} {steady:>8.3f} {ripple:>7.4f}')

        print('=' * 68)
        if not dead_times:
            print('沒有可分析的換向段。')
            return

        med = statistics.median
        d, ri, ov, rp = med(dead_times), med(rises), med(overshoots), med(ripples)
        print(f'中位數:  死區 {d*1000:.0f} ms   上升 {ri*1000:.0f} ms   '
              f'超越 {ov:.1f} %   抖動 {rp:.4f}')
        print()

        # ---- 前置檢查:車子到底有沒有追到目標速度? -------------------------
        # 這一關沒過的話,下面所有指標全部無效。死區/上升/超越都是以「最終會到
        # 達目標」為前提定義的;如果響應停在目標的 13% 就不動了,「第一次跨過
        # 目標 10%」量到的是它爬到自己天花板的時間,不是機械死區 —— 會誤報成
        # 背隙。2026-07-25 實測就踩到這個坑。
        reach = med([abs(s) for s in steadies]) / abs(self.speed)
        if reach < 0.7:
            print('✗' * 34)
            print(f'底盤只達到命令速度的 {reach*100:.0f}% '
                  f'(命令 {abs(self.speed):.3f}, 實際 {med([abs(s) for s in steadies]):.3f})')
            print('✗' * 34)
            print()
            print('**上面所有指標都無效** —— 它們的定義都預設車子最終會追到目標。')
            print('現在的問題不是 PID 調得好不好,是命令速度和實際速度對不上,')
            print(f'差了約 {1/max(reach,1e-6):.1f} 倍。先用這支找出換算關係:')
            print()
            print('    python3 tools/vel_sweep.py'
                  + ('  --axis yaw' if self.axis == 'yaw' else ''))
            print()
            print('校出 cmd_linear_scale / cmd_angular_scale 之後再回來跑本工具。')
            return

        # 判讀。這些門檻是經驗值,用來指方向,不是硬性合格線。
        verdicts = []
        # 門檻 0.20s:回授只有 ~20 Hz,加上「跨過目標 10%」這個判準本身的延遲,
        # 用合成訊號實測這個量法會系統性高估約 50 ms(注入 100ms → 讀到 150ms)。
        # 所以絕對值要寬鬆看待;真正有意義的是**同一台車不同設定之間的比較**。
        if d > 0.20:
            verdicts.append(
                f'⚠  死區 {d*1000:.0f} ms 偏長 → 機械背隙的嫌疑大。'
                '調 PID 救不了背隙,要從齒輪箱/聯軸器/輪轂固定螺絲下手。')
        else:
            verdicts.append(
                f'✓  死區 {d*1000:.0f} ms 正常 → **看不到明顯背隙**,'
                '「齒輪磨耗」不太可能是主因。')
        if ov > 25:
            verdicts.append(f'⚠  超越 {ov:.0f}% 過大 → vel_kp 太高,砍半試試。')
        if rp > 0.02 * max(abs(self.speed), 1e-6) * 5:
            verdicts.append(f'⚠  穩態抖動 {rp:.4f} 偏大 → vel_kp 太高,或機械共振。')
        if ri > 0.6:
            verdicts.append(f'⚠  上升 {ri*1000:.0f} ms 很慢 → 增益太低,'
                            '或扭力不足(看下面電壓)。')
        if len(verdicts) == 1:
            verdicts.append('✓  超越/抖動/上升都在合理範圍 → '
                            '這組 PID 本身沒問題,問題在別的地方。')
        for v in verdicts:
            print(v)

        # 電壓垂降:整場最高 vs 最低。這是「跑一陣子才變怪」的第一嫌疑犯。
        print()
        if self.batt:
            volts = [v for _, v in self.batt]
            hi, lo = max(volts), min(volts)
            print(f'電池: 最高 {hi:.2f} V  最低 {lo:.2f} V  垂降 {hi-lo:.2f} V')
            if hi - lo > 1.0:
                print('⚠  測試期間垂降超過 1 V → 供電是主要嫌疑。'
                      '電池老化或線徑不足,扭力會隨時間掉,PID 怎麼調都補不回來。')
            elif hi < 1.0:
                print('   (讀數看起來不像電壓,板子的電池欄位可能不是 V 或沒接。'
                      '拿三用電表量一下電池端電壓對照。)')
        else:
            print('沒收到 /battery_voltage —— 驅動要先重 build:')
            print('  cd ~/ros2_ws && colcon build --packages-select ominibot_driver '
                  '--symlink-install')

    @staticmethod
    def _analyse(seg, t_edge, before, after):
        """從一段換向後的資料算出 (死區, 上升, 超越%, 穩態, 抖動)。"""
        target = after
        if abs(target) < 1e-6:
            return None
        sign = math.copysign(1.0, target)

        # 死區: 速度第一次真的往新方向跑到目標的 10%。
        t_dead = None
        for t, v in seg:
            if v * sign > 0.1 * abs(target):
                t_dead = t - t_edge
                break
        if t_dead is None:
            return None

        # 上升: 從 10% 到第一次碰到 90%。
        t_90 = None
        for t, v in seg:
            if v * sign >= 0.9 * abs(target):
                t_90 = t - t_edge
                break
        rise = (t_90 - t_dead) if t_90 is not None else float('nan')

        # 穩態: 取這一段的後半段(此時應已安定)。
        tail = [v for t, v in seg if t - t_edge > 0.5 * (seg[-1][0] - t_edge)]
        if len(tail) < 3:
            return None
        steady = statistics.mean(tail)
        ripple = statistics.pstdev(tail)

        # 超越: 整段裡朝目標方向的最大值,相對目標超出多少。
        peak = max((v * sign for _, v in seg), default=0.0)
        over = max(0.0, (peak - abs(target)) / abs(target) * 100.0)

        return t_dead, rise, over, steady, ripple


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--axis', choices=['x', 'yaw'], default='x',
                    help='x = 前後直線 (預設), yaw = 原地旋轉')
    ap.add_argument('--speed', type=float, default=None,
                    help='方波振幅。預設 x 軸 0.15 m/s、yaw 軸 0.5 rad/s')
    ap.add_argument('--hold', type=float, default=1.5,
                    help='每段維持幾秒 (預設 1.5)')
    ap.add_argument('--cycles', type=int, default=5,
                    help='幾個週期 (預設 5,每週期兩次換向)')
    args = ap.parse_args()
    if args.speed is None:
        args.speed = 0.5 if args.axis == 'yaw' else 0.15

    rclpy.init()
    node = StepResponse(args.axis, args.speed, args.hold, args.cycles)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        if node.run():
            node.report()
    except KeyboardInterrupt:
        print('\n中斷 —— 送零速度停車。')
        for _ in range(10):
            node._send(0.0)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
