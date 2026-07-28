#!/usr/bin/env python3
"""光達點雲和地圖疊不起來時,錯多少、往哪邊錯?

在 RViz 看到 /scan 和 /map 對不上時,這支給你數字而不是感覺:

  1. 用「現在的 TF」把 /scan 打到 map 座標,算每個光點到最近牆的平均距離
     → 這就是「疊得多爛」的客觀分數(完美對齊時應該接近地圖解析度 0.05m)
  2. 在現在的位姿附近暴搜 (dx, dy, dyaw),找出最合的位姿
     → 如果找得到明顯更好的,就是 AMCL 收斂錯了(位置對不上,不是地圖爛),
       而且會直接告訴你差多少,可以拿去 RViz 的 2D Pose Estimate 修
     → 如果連最好的位姿也很差,那是地圖和現場不符(現場被改過 / 地圖本身歪了)

在跑 Nav2 的機器上(或任何看得到 topic 的機器):
    python3 tools/scan_match_check.py

唯讀:只訂閱 /map、/scan 和 TF,不發任何指令、不會讓車移動。

★ 注意這支比對的是**靜態地圖**(/map),不是 costmap。所以它只回答「定位對不對」,
  不受 robot_radius / inflation 影響(那些是 tools/costmap_check.py 的事)。
"""
import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

WALL = 65          # /map 上多少算牆(對齊 map yaml 的 occupied_thresh 0.65)
MAX_BEAMS = 180    # 取樣這麼多道光束就夠,全用會慢很多而結論不變


class C(Node):
    def __init__(self):
        super().__init__('scan_match_check')
        self.map = None
        self.scan = None
        self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'map', m),
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(
            LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.tf = Buffer()
        self._tl = TransformListener(self.tf, self)


def distance_field(m):
    """每一格到最近牆的距離(公尺),用 BFS 近似(4-connected)。"""
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    INF = 10 ** 9
    dist = [INF] * (w * h)
    q = deque()
    for k, v in enumerate(m.data):
        if v >= WALL:
            dist[k] = 0
            q.append(k)
    while q:
        k = q.popleft()
        j, i = divmod(k, w)
        # divmod 給 (row, col) 反了,修正:k = row*w + col
        row, col = k // w, k % w
        for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            u, v2 = col + a, row + b
            if 0 <= u < w and 0 <= v2 < h:
                nk = v2 * w + u
                if dist[nk] > dist[k] + 1:
                    dist[nk] = dist[k] + 1
                    q.append(nk)
    return dist, w, h, res


def score(pts, dist, w, h, res, ox, oy, dx, dy, dyaw):
    """把點雲平移旋轉後,回傳 (平均到牆距離 m, 落在圖外的點數)。"""
    c, s = math.cos(dyaw), math.sin(dyaw)
    tot = 0.0
    out = 0
    n = 0
    for px, py in pts:
        x = c * px - s * py + dx
        y = s * px + c * py + dy
        i = int((x - ox) / res)
        j = int((y - oy) / res)
        if not (0 <= i < w and 0 <= j < h):
            out += 1
            continue
        d = dist[j * w + i]
        tot += min(d, 40) * res      # 夾住,免得少數離群點主導分數
        n += 1
    if n == 0:
        return 999.0, out
    return tot / n, out


def main():
    rclpy.init()
    n = C()
    end = n.get_clock().now().nanoseconds + 25e9
    while rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.map and n.scan and n.tf.can_transform(
                'map', n.scan.header.frame_id, rclpy.time.Time()):
            break
    if not n.map:
        print('沒收到 /map'); return
    if not n.scan:
        print('沒收到 /scan'); return
    frame = n.scan.header.frame_id
    if not n.tf.can_transform('map', frame, rclpy.time.Time()):
        print(f'查不到 map->{frame} 的 TF'); return

    m = n.map
    dist, w, h, res = distance_field(m)
    ox, oy = m.info.origin.position.x, m.info.origin.position.y

    # /scan 的極座標 → laser 座標系的 xy
    sc = n.scan
    raw = []
    for k, r in enumerate(sc.ranges):
        if not (sc.range_min < r < sc.range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = sc.angle_min + k * sc.angle_increment
        raw.append((r * math.cos(a), r * math.sin(a)))
    if not raw:
        print('/scan 沒有有效點'); return
    step = max(1, len(raw) // MAX_BEAMS)
    beams = raw[::step]

    # laser → map(用目前的 TF)
    t = n.tf.lookup_transform('map', frame, rclpy.time.Time())
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    tx, ty = t.transform.translation.x, t.transform.translation.y
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(c * x - s * y + tx, s * x + c * y + ty) for x, y in beams]

    print(f'/map {w}x{h} @ {res:.3f}m,/scan frame={frame},有效光點 {len(raw)}'
          f'(取樣 {len(beams)} 道)')
    print(f'目前 TF: map->{frame} x={tx:.3f} y={ty:.3f} yaw={math.degrees(yaw):.2f}°')
    base, out0 = score(pts, dist, w, h, res, ox, oy, 0, 0, 0)
    print(f'目前對齊分數(平均離牆距離)= {base:.3f} m'
          f'{f",有 {out0} 點落在地圖外" if out0 else ""}')
    print()

    # 暴搜:繞「點雲自己的形心」旋轉,避免旋轉時整團跑掉
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    rel = [(p[0] - cx, p[1] - cy) for p in pts]

    best = (base, 0.0, 0.0, 0.0)
    for dyaw_deg in [x * 1.0 for x in range(-30, 31)]:
        dyaw = math.radians(dyaw_deg)
        for idx in range(-10, 11):
            dx = idx * 0.05
            for jdy in range(-10, 11):
                dy = jdy * 0.05
                sc2, _ = score(rel, dist, w, h, res, ox, oy,
                               cx + dx, cy + dy, dyaw)
                if sc2 < best[0]:
                    best = (sc2, dx, dy, dyaw_deg)

    bscore, bdx, bdy, bdyaw = best
    print(f'暴搜最佳: 分數 {bscore:.3f} m  '
          f'(需要修正 dx={bdx:+.2f}m dy={bdy:+.2f}m dyaw={bdyaw:+.1f}°)')
    print()

    # ★ 不要只看「平均離牆距離」的絕對值就判定好壞。在窄迷宮裡大多數光點打在
    #   0.2-0.5m 的近牆上,角度誤差在近處只造成一兩格偏移,平均值會被近點稀釋 ——
    #   實測 10° 的航向誤差平均分數也只有 0.04m(小於一格),看起來「還不錯」,
    #   但同樣的 10° 在 3m 遠處就是 3*sin(10°) ≈ 0.52m 的偏移,在 RViz 上非常明顯。
    #   所以判斷要看「有沒有明顯更好的位姿」,不是絕對分數。
    far_err = 3.0 * math.sin(math.radians(abs(bdyaw)))
    needs = abs(bdyaw) >= 3.0 or math.hypot(bdx, bdy) >= 0.05
    improved = bscore < base * 0.75
    good = res * 1.5

    if needs and improved:
        print('→ **定位有偏差**:存在明顯更合的位姿,所以地圖本身沒問題。')
        if abs(bdyaw) >= 3.0:
            print(f'   航向差 {bdyaw:+.1f}° —— 這在 3m 遠處會變成約 {far_err:.2f} m 的偏移,')
            print('   所以近牆看起來還算貼合、遠牆整片歪掉,正是「疊不起來」的典型樣子。')
        print('   立刻修:RViz 的 2D Pose Estimate 按上面的修正量重設位姿。')
        print('   或者讓車走一段:AMCL 只在移動超過 update_min_d(0.10m)/')
        print('   update_min_a(0.15rad≈8.6°)時才更新,停著不動它不會自己修。')
        print('   走了還是持續歪 → 往上游查 odom / 光達外參')
        print('   (notes/SLAM_learning_note.md §7、tools/analyze_bag.py)。')
    elif base <= good:
        print('→ 對齊沒有明顯可改善的空間。RViz 上看起來歪的話,先確認 Fixed Frame 是')
        print('   map,而且看的是 /map 而不是 costmap。')
    else:
        print('→ 連最佳位姿都對不上 = **地圖和現場不符**,不是定位問題。')
        print('   現場被移動過就重新建圖;不然要懷疑地圖本身在建的時候就歪了')
        print('   (光達外參 laser_yaw / laser_x / laser_y,見 tools/analyze_bag.py)。')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
