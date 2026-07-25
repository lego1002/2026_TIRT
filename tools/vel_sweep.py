#!/usr/bin/env python3
"""掃描「命令速度 → 實際速度」的對應關係 —— 校 cmd_linear_scale / cmd_angular_scale。

為什麼需要這支(2026-07-25 發現的問題):
命令 /cmd_vel 0.15 m/s,車子實際只跑 0.019 m/s —— **差了快 8 倍**。
`/odom` 的讀數和肉眼觀察互相吻合(整場測試車子只挪動不到 2 cm),所以這不是量測誤差,
是命令真的沒被執行到該有的量。

原因:板子的內部標定是給 CircusPi 原廠參考車用的,和這台 N20 小車差了約 6.5 倍。
這個誤差**命令和回授兩條路都有**,但之前只修了回授那一半:

    回授路徑:  板子回報值 × odom_linear_scale(0.153) → 正確的 m/s   ✅ 已修
    命令路徑:  我們送 0.15 m/s → 板子當成自己的單位解讀 → 只跑 0.019  ❌ 沒修

佐證:teleop 的預設速度是 `linear_speed = 0.6` m/s、`angular_speed = 1.5` rad/s。
對一台 15 cm 的迷宮小車來說這快得離譜 —— 除非車子根本跑不到那麼快,
否則沒有人會把預設值設成這樣。那個 0.6 就是當初被迫一路往上加出來的。

這支的工作:對每個命令速度量出實際速度,做線性迴歸,算出要補的倍率。
順便判斷是「單純的固定倍率」還是「有靜摩擦死區」(兩者的修法不一樣)。

--- 用法 ---

    source /opt/ros/humble/setup.bash
    source dds/setup_dds.sh            # PC 端記得 DDS_SERVER=<pi_ip>
    python3 tools/vel_sweep.py                 # 直線
    python3 tools/vel_sweep.py --axis yaw      # 原地旋轉

⚠️  車子會依序用不同速度前進後退(正負交替,盡量留在原地,但仍會漂移)。
    **請留 1.5 m 以上的淨空**,並把 teleop 關掉。Ctrl-C 會立刻停車。

⚠️  跑之前先把補償歸一,量到的才是板子的原始行為:
        ./run_robot.sh cmd_linear_scale:=1.0 cmd_angular_scale:=1.0

--- 怎麼判讀 ---

斜率 slope = 實際 / 命令。要設的補償就是它的倒數:

    cmd_linear_scale = 1 / slope

截距接近 0 → 單純的倍率問題,設好 scale 就解決。
截距明顯為負 → 有靜摩擦死區(命令要大於某個值輪子才會動),
              低速時再怎麼補倍率都會有一段完全不動的區間,
              這種要靠降低 vel_kp 之外的手段(或接受最低速度限制)。
R² < 0.9   → 響應不線性,可能已經在馬達的飽和區,把 --max 調小重測。
"""

import argparse
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VelSweep(Node):
    def __init__(self, axis, speeds, hold, repeat=3):
        super().__init__('vel_sweep')
        self.axis = axis
        self.speeds = speeds
        self.hold = hold
        self.repeat = repeat
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 50)
        self._lock = threading.Lock()
        self._buf = []
        self._collecting = False

    def _odom_cb(self, msg):
        v = (msg.twist.twist.angular.z if self.axis == 'yaw'
             else msg.twist.twist.linear.x)
        with self._lock:
            if self._collecting:
                self._buf.append(v)

    def _send(self, v):
        t = Twist()
        if self.axis == 'yaw':
            t.angular.z = v
        else:
            t.linear.x = v
        self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self._send(0.0)
            time.sleep(0.05)

    def measure(self, cmd):
        """送一個定速命令,丟掉前半段(過渡),取後半段的平均。"""
        with self._lock:
            self._buf = []
            self._collecting = False
        end = time.monotonic() + self.hold
        settle = time.monotonic() + self.hold * 0.5
        started = False
        while time.monotonic() < end:
            self._send(cmd)          # 持續重發,餵飽驅動的 watchdog
            if not started and time.monotonic() > settle:
                with self._lock:
                    self._buf = []
                    self._collecting = True
                started = True
            time.sleep(0.05)
        with self._lock:
            self._collecting = False
            data = list(self._buf)
        self.stop()
        time.sleep(0.4)              # 每次之間停一下,避免殘餘動量污染下一點
        return statistics.mean(data) if len(data) >= 3 else None

    def run(self):
        unit = 'rad/s' if self.axis == 'yaw' else 'm/s'
        print(f'速度掃描: {self.axis} 軸, 每點 {self.hold}s')
        print(f'命令點: {", ".join(f"{s:.2f}" for s in self.speeds)} {unit}')
        print('等 /odom 上線 …')
        deadline = time.monotonic() + 10.0
        with self._lock:
            self._collecting = True
        while not self._buf and time.monotonic() < deadline:
            time.sleep(0.1)
        with self._lock:
            self._collecting = False
        if not self._buf:
            print('✗ 收不到 /odom。確認 ./run_robot.sh 有在跑、DDS_SERVER 設對了。')
            return None

        print()
        # 每個命令點量 self.repeat 次。單次抽樣在 2026-07-25 讓 wheel_test.py
        # 連續兩次對硬體下了錯誤判決(每跑一次「故障輪」就換一顆),同一個教訓
        # 在這裡也適用:先證明量得準,再談量到了什麼。
        hdr = ' '.join(f'{"#"+str(k+1):>9}' for k in range(self.repeat))
        print(f'{"命令":>8} {hdr} {"中位":>9} {"離散":>7} {"比值":>8}')
        print('-' * (18 + 10 * self.repeat + 18))
        pts = []
        shaky = 0
        # 正負交替,讓車子大致留在原地。
        for i, s in enumerate(self.speeds):
            cmd = s if i % 2 == 0 else -s
            # 重複量測時正負也交替,否則同方向連跑 N 次會一路漂走。
            # 迴歸本來就取絕對值,所以方向交替不影響結果。
            raw = [self.measure(cmd if k % 2 == 0 else -cmd)
                   for k in range(self.repeat)]
            vals = [abs(v) for v in raw if v is not None]
            if len(vals) < 2:
                print(f'{cmd:>8.3f}  (沒資料)')
                continue
            med = statistics.median(vals)
            spread = statistics.pstdev(vals) / med if med > 1e-9 else float('inf')
            if spread > 0.2:
                shaky += 1
            cells = ' '.join(f'{v:>9.4f}' for v in vals)
            ratio = med / abs(cmd) if abs(cmd) > 1e-9 else float('nan')
            flag = '  ← 不穩!' if spread > 0.2 else ''
            print(f'{cmd:>8.3f} {cells} {med:>9.4f} {spread*100:>6.0f}% '
                  f'{ratio:>8.3f}{flag}')
            # 統一取正,負向點鏡射回來一起做迴歸。
            pts.append((abs(cmd), med))

        if shaky >= 2:
            print()
            print('=' * 60)
            print(f'✗ {shaky} 個命令點的重複量測自己就對不起來(離散 > 20%)。')
            print('  **這代表量測不可信,下面的迴歸不要當真。**')
            print('  先確認:teleop 有沒有關掉(兩個節點搶 /cmd_vel)、')
            print('  車子有沒有撞到東西或卡住、地面是不是會打滑。')
            print('=' * 60)
        return pts


def linfit(pts):
    """最小平方擬合 y = a*x + b,回傳 (a, b, r2)。"""
    n = len(pts)
    if n < 2:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx < 1e-12:
        return None
    a = sxy / sxx
    b = my - a * mx
    sst = sum((y - my) ** 2 for y in ys)
    ssr = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ssr / sst if sst > 1e-12 else float('nan')
    return a, b, r2


def report(pts, axis):
    print()
    fit = linfit(pts)
    if fit is None:
        print('資料點不足,無法迴歸。')
        return
    a, b, r2 = fit
    unit = 'rad/s' if axis == 'yaw' else 'm/s'
    pname = 'cmd_angular_scale' if axis == 'yaw' else 'cmd_linear_scale'
    print('=' * 60)
    print(f'迴歸: 實際 = {a:.4f} × 命令 + {b:+.4f}      R² = {r2:.3f}')
    print('=' * 60)

    if abs(a) < 1e-6:
        print('✗ 斜率幾乎是 0 —— 車子完全沒動。檢查馬達電源/接線,先別談校準。')
        return

    scale = 1.0 / a
    print()
    print(f'→  {pname} = 1 / {a:.4f} = **{scale:.2f}**')
    print()
    print(f'   ./run_robot.sh {pname}:={scale:.2f}')
    print()

    # 資料品質先判斷。R² 低的時候,截距(死區)是拿來擬合散亂點的自由參數,
    # 不代表任何物理意義 —— 2026-07-25 就踩過:R²=0.63 的資料被報成「死區 0.128 m/s」,
    # 但實際上那是逐點量測本身就飄,不是靜摩擦。所以死區只在 R² 夠高時才報。
    if r2 >= 0.9:
        print(f'✓  R² = {r2:.3f},線性良好,這個倍率可信。')
        if a > 1e-6:
            deadband = -b / a
            if deadband > 0.02 * max(p[0] for p in pts):
                print(f'⚠  推算死區 ≈ {deadband:.3f} {unit}(命令小於這個值輪子不會動)。')
                print('   這是靜摩擦,補倍率補不掉 —— 低速時仍會有一段完全不動的區間。')
            else:
                print(f'✓  死區可忽略({deadband:.4f} {unit})→ 單純的倍率問題。')
        return

    print(f'⚠  R² = {r2:.3f} 偏低 —— **響應不是一條線,逐點量測在飄**。')
    print()
    print(f'   斜率 {a:.4f} 仍可當作大致的量級(尤其兩軸各自量出來如果很接近),')
    print('   但截距(死區)在這種資料上沒有物理意義,所以不報 —— 它只是拿來')
    print('   擬合散點的自由參數。')
    print()
    # 逐點比值的離散程度 + 正負向不對稱,是「單一輪子有問題」的典型指紋。
    ratios = [g / c for c, g in pts if c > 1e-9]
    if len(ratios) >= 3:
        spread = statistics.pstdev(ratios) / max(statistics.mean(ratios), 1e-9)
        print(f'   逐點比值: {", ".join(f"{r:.3f}" for r in ratios)}')
        print(f'   離散程度 {spread*100:.0f}% —— 如果命令變大實際卻沒跟著變大,')
        print('   或正轉反轉差很多,那不是標定問題,是有輪子出力不正常。')
    print()
    print('   下一步:把車抬離地面(輪子懸空),跑')
    print('       python3 tools/wheel_test.py')
    print('   逐顆輪子單獨測,直接找出是不是有一顆沒在動。')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--axis', choices=['x', 'yaw'], default='x')
    ap.add_argument('--max', type=float, default=None,
                    help='掃描上限。預設 x=0.6 m/s、yaw=1.5 rad/s(teleop 的預設值)')
    ap.add_argument('--points', type=int, default=6, help='幾個命令點 (預設 6)')
    ap.add_argument('--hold', type=float, default=2.0,
                    help='每點維持幾秒 (預設 2.0,前半段當過渡丟掉)')
    ap.add_argument('--repeat', type=int, default=3,
                    help='每個命令點重複量幾次 (預設 3)。用來確認量得準,不要降到 1。')
    args = ap.parse_args()
    top = args.max if args.max is not None else (1.5 if args.axis == 'yaw' else 0.6)
    speeds = [top * (i + 1) / args.points for i in range(args.points)]

    rclpy.init()
    node = VelSweep(args.axis, speeds, args.hold, args.repeat)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        pts = node.run()
        if pts:
            report(pts, args.axis)
    except KeyboardInterrupt:
        print('\n中斷 —— 送零速度停車。')
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
