#!/usr/bin/env python3
"""為什麼 Nav2 的 goal 一下就 aborted —— 從 costmap 幾何找答案。

用途:goal 送出去馬上 aborted、車子完全不動、而 nav2 閒置時又不噴任何警告時,
第一個該跑的就是這支。它回答三個問題:

  1. 機器人現在站的那一格,planner 認為可不可走?
     (最常見的死因:車停得離牆太近 → 那一格是 inscribed → 連「起點」都不合法,
      planner 直接拒絕 → goal 立刻 aborted。畫面上看起來就是「按了沒反應」。)
  2. 從機器人出發,planner 到得了地圖的多少比例?
     (只有幾 % 的話就是走道被 inflation 封死切成好幾塊,不是 planner 壞了。)
  3. 換成不同的 robot_radius,這個迷宮的連通性會怎麼變?
     → 直接告訴你 robot_radius 該設多少,不必反覆猜、重啟、再試。

在**跑 Nav2 的那台機器**上執行(或任何看得到它 topic 的機器):
    python3 tools/costmap_check.py

唯讀:只訂閱 /global_costmap/costmap、/map 和 TF,不發任何指令、不會讓車移動。

★ 代價刻度的陷阱:nav2_costmap_2d 發布到 /global_costmap/costmap 的
  OccupancyGrid 是**重新縮放成 0..100** 的版本,不是 raw 0..255 的內部代價。對照:
      100 = LETHAL_OBSTACLE (raw 254)
       99 = INSCRIBED_INFLATED_OBSTACLE (raw 253) ← planner 視為不可走
    1..98 = 一般 inflation 梯度(可走,只是不鼓勵)
        0 = 完全空
       -1 = NO_INFORMATION (raw 255) ← allow_unknown:=false 時也不可走
  拿 253/254 去比對這張圖永遠不會命中,會得到「整張圖都是空的」的錯誤結論。
"""
import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener

BLOCK = 99          # 已縮放刻度:>=99 → planner 不可走
WALL_THRESH = 65    # /map 上多少以上算牆(和 map yaml 的 occupied_thresh 0.65 一致)


def flood(w, h, passable, sx, sy):
    """4-connected flood fill;回傳 (visited, count)。起點不可走時回傳 0。"""
    seen = bytearray(w * h)
    if not passable(sx, sy):
        return seen, 0
    seen[sy * w + sx] = 1
    q = deque([(sx, sy)])
    n = 1
    while q:
        i, j = q.popleft()
        for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            u, v = i + a, j + b
            if 0 <= u < w and 0 <= v < h and not seen[v * w + u] and passable(u, v):
                seen[v * w + u] = 1
                n += 1
                q.append((u, v))
    return seen, n


def nearest_passable(w, h, passable, cx, cy, max_r=60):
    best = None
    for r in range(1, max_r):
        for j in range(cy - r, cy + r + 1):
            for i in range(cx - r, cx + r + 1):
                if max(abs(i - cx), abs(j - cy)) != r:
                    continue
                if 0 <= i < w and 0 <= j < h and passable(i, j):
                    d = math.hypot(i - cx, j - cy)
                    if best is None or d < best[0]:
                        best = (d, i, j)
        if best:
            return best
    return None


class Collector(Node):
    def __init__(self):
        super().__init__('costmap_check')
        self.costmap = None
        self.static_map = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'costmap', m), 10)
        # /map 是 latched,一定要 TRANSIENT_LOCAL 才收得到已經發過的那一份。
        self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'static_map', m),
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.tf = Buffer()
        self._tl = TransformListener(self.tf, self)

    def ready(self):
        return (self.costmap is not None and self.static_map is not None
                and self.tf.can_transform('map', 'base_link', rclpy.time.Time()))


def report_costmap(g, tf):
    w, h, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = g.data

    def passable(i, j):
        return 0 <= d[j * w + i] < BLOCK

    t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
    x, y = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y ** 2 + q.z ** 2)))
    cx = int((x - ox) / res)
    cy = int((y - oy) / res)

    print(f'global costmap {w}x{h} @ {res:.3f} m,origin ({ox:.2f}, {oy:.2f})')
    print(f'機器人 map 座標 x={x:.3f} y={y:.3f} yaw={yaw:.1f}°  → cell ({cx},{cy})')
    if not (0 <= cx < w and 0 <= cy < h):
        print('✗ 機器人不在 costmap 範圍內 → planner 必定失敗(定位錯了,或地圖不對)')
        return None
    v = d[cy * w + cx]
    kind = ('未知(-1)' if v < 0 else 'LETHAL(100)' if v >= 100
            else 'INSCRIBED(99) → planner 視為不可走' if v >= BLOCK
            else f'inflation 梯度({v},可走)' if v > 0 else '完全空(0)')
    print(f'機器人所在格代價 = {v} → {kind}')

    total = sum(1 for k in range(w * h) if 0 <= d[k] < BLOCK)
    _, reach = flood(w, h, passable, cx, cy)
    if reach:
        print(f'從機器人出發可到 {reach}/{total} 格 '
              f'({100.0 * reach / max(total, 1):.1f}%,{reach * res * res:.2f} m²)')
    else:
        near = nearest_passable(w, h, passable, cx, cy)
        print('✗ 起點不合法,planner 連規劃都不會開始 → goal 立刻 aborted')
        if near:
            print(f'  最近的合法格在 ({near[1]},{near[2]}),距離 {near[0] * res:.3f} m')
            _, r2 = flood(w, h, passable, near[1], near[2])
            print(f'  從那一格出發可到 {r2}/{total} 格 '
                  f'({100.0 * r2 / max(total, 1):.1f}%)')
    return (cx, cy)


def sweep_radius(m, tf, radii):
    """在原始 /map 上重算不同 robot_radius 的內切阻擋與連通性。"""
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    occ = [1 if (v >= WALL_THRESH or v < 0) else 0 for v in m.data]
    t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
    cx = int((t.transform.translation.x - ox) / res)
    cy = int((t.transform.translation.y - oy) / res)

    print()
    print(f'原始 /map {w}x{h};牆或未知的格數 {sum(occ)}')
    print('robot_radius 掃描(單一連通區越大越好;越小越危險但越過得去):')
    for radius in radii:
        rc = int(math.ceil(radius / res))
        blocked = bytearray(w * h)
        for j in range(h):
            for i in range(w):
                if not occ[j * w + i]:
                    continue
                for b in range(max(0, j - rc), min(h, j + rc + 1)):
                    for a in range(max(0, i - rc), min(w, i + rc + 1)):
                        if math.hypot(a - i, b - j) * res <= radius:
                            blocked[b * w + a] = 1

        def ok(i, j, _b=blocked):
            return not _b[j * w + i]

        free = w * h - sum(blocked)
        here = ok(cx, cy) if (0 <= cx < w and 0 <= cy < h) else False
        seed = (cx, cy) if here else None
        if seed is None:
            near = nearest_passable(w, h, ok, cx, cy, max_r=40)
            seed = (near[1], near[2]) if near else None
        if seed is None:
            print(f'  {radius:.3f} m → 完全沒有可走格')
            continue
        _, reach = flood(w, h, ok, seed[0], seed[1])
        note = '←車現在這格可走' if here else '(車現在這格不可走)'
        print(f'  {radius:.3f} m → 可走 {free:5d} 格,最大連通區 {reach:5d} 格 '
              f'({reach * res * res:6.2f} m²)  {note}')
    print()
    print('怎麼讀:連通區面積出現斷崖的那個值就是這個迷宮的上限,robot_radius 要設在')
    print('斷崖之下。設太大 → 走道被封死、planner 說找不到路;設太小 → 車角會擦牆。')


def main():
    radii = [float(a) for a in sys.argv[1:]] or [0.12, 0.11, 0.10, 0.095, 0.09, 0.08]
    rclpy.init()
    n = Collector()
    end = n.get_clock().now().nanoseconds + 25e9
    while rclpy.ok() and n.get_clock().now().nanoseconds < end and not n.ready():
        rclpy.spin_once(n, timeout_sec=0.2)

    if n.costmap is None:
        print('沒收到 /global_costmap/costmap —— Nav2 沒在跑,或 DDS 沒接上'
              '(先 source dds/setup_dds.sh)')
    elif not n.tf.can_transform('map', 'base_link', rclpy.time.Time()):
        print('收到 costmap 但查不到 map->base_link TF —— 定位那一側有問題'
              '(AMCL 沒起來 / Pi 的 odom TF 沒過來)')
    else:
        here = report_costmap(n.costmap, n.tf)
        if here is not None and n.static_map is not None:
            sweep_radius(n.static_map, n.tf, radii)
        elif n.static_map is None:
            print('(沒收到 /map,跳過 robot_radius 掃描)')

    n.destroy_node()
    rclpy.shutdown()


main()
