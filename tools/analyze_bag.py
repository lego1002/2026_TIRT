#!/usr/bin/env python3
"""從診斷 bag 算出真正的旋轉刻度與光達 yaw 偏移 —— 不靠肉眼看 RViz。

核心想法:**用 /scan 自己當真值**,拿它去驗 /odom。
光達看到的世界是不會說謊的,odom 才是會的那個。

  A) 旋轉段:車原地轉時,連續兩幀 scan 的差別就是「轉了多少」。用環形互相關
     (circular cross-correlation)算出每一對的角位移,加總 = 真實轉了幾度。
     拿去比 odom 報的度數 → 得到 gyro_scale 該乘多少。

  B) 直線段:車平移時,某方位角 θ 的距離變化率 Δr(θ) ≈ -d·cos(θ-φ),其中 φ 就是
     「在光達座標系裡,車往哪個方向走」。用最小平方擬合 cos/sin 分量解出 φ,
     再和 odom 說的行進方向 ψ 相比 → 差值就是 laser_yaw 該填的值。
     (推導:若 laser_frame 相對 base_link 轉了 α,則 base_link 裡的方向 ψ
      在光達座標系看起來是 ψ-α,所以 α = ψ - φ。)

用法:
    python3 tools/analyze_bag.py ~/bags/diag1

搭配 tools/record_diag.py 錄的 bag(靜止→原地轉360°→靜止→直行1m→靜止)。
靜止段是用來自動切分段落的,所以務必停好停滿。
"""

import math
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

# 判定「在動」的門檻(odom twist)。低於此值視為靜止,用來切段。
W_MOVING = 0.05    # rad/s
V_MOVING = 0.02    # m/s


def load(bag_dir):
    import glob
    import sqlite3
    files = sorted(glob.glob(f'{bag_dir}/*.db3'))
    if not files:
        sys.exit(f'找不到 {bag_dir}/*.db3')
    scans, odoms = [], []
    for f in files:
        con = sqlite3.connect(f)
        tid = {name: i for i, name in con.execute('select id,name from topics')}
        for topic, store, cls in (('/scan', scans, LaserScan),
                                  ('/odom', odoms, Odometry)):
            if topic not in tid:
                continue
            for ts, blob in con.execute(
                    'select timestamp,data from messages where topic_id=? '
                    'order by timestamp', (tid[topic],)):
                store.append((ts * 1e-9, deserialize_message(bytes(blob), cls)))
    return scans, odoms


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clean(scan):
    """ranges → 乾淨的 numpy 陣列,無效值(inf/nan/0/超出範圍)填 max_range。"""
    r = np.asarray(scan.ranges, dtype=np.float64)
    bad = ~np.isfinite(r) | (r < scan.range_min) | (r > scan.range_max)
    r = r.copy()
    r[bad] = scan.range_max
    return r


def shift_between(r1, r2):
    """用環形互相關求 r2 相對 r1 的位移(單位:bin,可為小數)。

    注意符號:機器人逆時針轉 Δ 時,世界特徵在感測器座標裡是往順時針跑,
    所以這個函式回傳的是 **−Δ**。呼叫端要取負號才會得到機器人的轉角。
    (2026-07-25 一度漏掉這個負號,導致誤判成 gyro_z_sign 反了。
     tools/selftest_analyze.py 用合成資料把這個約定釘死。)
    """
    a = r1 - r1.mean()
    b = r2 - r2.mean()
    n = len(a)
    corr = np.fft.irfft(np.fft.rfft(b) * np.conj(np.fft.rfft(a)), n)
    k = int(np.argmax(corr))
    # 拋物線內插取次 bin 精度
    y0, y1, y2 = corr[(k - 1) % n], corr[k], corr[(k + 1) % n]
    denom = y0 - 2 * y1 + y2
    delta = 0.5 * (y0 - y2) / denom if abs(denom) > 1e-12 else 0.0
    s = k + delta
    return s - n if s > n / 2 else s          # 折回 ±n/2


def segments(odoms, key, thresh):
    """找出 key(twist) 連續超過門檻的區段,回傳 [(起 index, 迄 index)]。"""
    flags = [abs(key(o[1])) > thresh for o in odoms]
    out, start = [], None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        elif not f and start is not None:
            if i - start >= 5:
                out.append((start, i))
            start = None
    if start is not None and len(flags) - start >= 5:
        out.append((start, len(flags)))
    return out


def analyze_rotation(scans, odoms, seg):
    t0, t1 = odoms[seg[0]][0], odoms[seg[1] - 1][0]
    odom_yaw = sum(
        math.atan2(math.sin(yaw_of(odoms[i + 1][1].pose.pose.orientation)
                            - yaw_of(odoms[i][1].pose.pose.orientation)),
                   math.cos(yaw_of(odoms[i + 1][1].pose.pose.orientation)
                            - yaw_of(odoms[i][1].pose.pose.orientation)))
        for i in range(seg[0], seg[1] - 1))

    sub = [s for s in scans if t0 <= s[0] <= t1]
    if len(sub) < 3:
        print('  旋轉段內 scan 太少,跳過'); return
    inc = sub[0][1].angle_increment
    total_bins = 0.0
    for (ta, sa), (tb, sb) in zip(sub, sub[1:]):
        total_bins += shift_between(clean(sa), clean(sb))
    scan_yaw = -total_bins * inc      # 負號:見 shift_between 的 docstring

    print(f'  時間長度      : {t1 - t0:.1f} s   (scan {len(sub)} 幀)')
    print(f'  odom 報的轉角 : {math.degrees(odom_yaw):+8.1f}°')
    print(f'  scan 實測轉角 : {math.degrees(scan_yaw):+8.1f}°   ← 真值')
    if abs(odom_yaw) > 0.05:
        ratio = scan_yaw / odom_yaw
        print(f'  比值 scan/odom: {ratio:+.4f}')
        if ratio < 0:
            print('  ⚠️  **符號相反** → gyro_z_sign 要翻成 -1.0'
                  '（odom 轉向和實際相反,這會讓地圖以雙倍速度扇形展開）')
        print(f'  → 新 gyro_scale = 現值 × {abs(ratio):.4f}')


def analyze_translation(scans, odoms, seg):
    t0, t1 = odoms[seg[0]][0], odoms[seg[1] - 1][0]
    # odom 說的行進方向(base_link 座標系):對整段平均,避免單筆雜訊
    vx = np.mean([odoms[i][1].twist.twist.linear.x for i in range(*seg)])
    vy = np.mean([odoms[i][1].twist.twist.linear.y for i in range(*seg)])
    psi = math.atan2(vy, vx)

    sub = [s for s in scans if t0 <= s[0] <= t1]
    if len(sub) < 3:
        print('  直線段內 scan 太少,跳過'); return

    # 逐對擬合 Δr(θ) = -d·cos(θ-φ) = A·cosθ + B·sinθ,累加 A、B
    A_sum = B_sum = 0.0
    for (ta, sa), (tb, sb) in zip(sub, sub[1:]):
        r1, r2 = clean(sa), clean(sb)
        th = sa.angle_min + np.arange(len(r1)) * sa.angle_increment
        dr = r2 - r1
        # 只用兩幀都有有效回波、且變化量合理的光束
        ok = ((r1 < sa.range_max * 0.99) & (r2 < sa.range_max * 0.99)
              & (np.abs(dr) < 0.2))
        if ok.sum() < 30:
            continue
        M = np.stack([np.cos(th[ok]), np.sin(th[ok])], axis=1)
        coef, *_ = np.linalg.lstsq(M, dr[ok], rcond=None)
        A_sum += coef[0]; B_sum += coef[1]

    phi = math.atan2(-B_sum, -A_sum)
    alpha = math.atan2(math.sin(psi - phi), math.cos(psi - phi))
    print(f'  時間長度        : {t1 - t0:.1f} s   (scan {len(sub)} 幀)')
    print(f'  odom 行進方向 ψ : {math.degrees(psi):+8.1f}°  (base_link;純前進應 ≈0)')
    print(f'  scan 行進方向 φ : {math.degrees(phi):+8.1f}°  (光達座標系)  ← 真值')
    print(f'  → laser_yaw 應為 : {alpha:+.4f} rad = {math.degrees(alpha):+.1f}°')


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else sys.exit('用法: analyze_bag.py <bag_dir>')
    scans, odoms = load(bag)
    print(f'讀到 /scan {len(scans)} 幀、/odom {len(odoms)} 筆\n')
    if len(scans) < 10 or len(odoms) < 10:
        sys.exit('資料太少,無法分析。')

    rot = segments(odoms, lambda o: o.twist.twist.angular.z, W_MOVING)
    lin = segments(odoms, lambda o: math.hypot(o.twist.twist.linear.x,
                                               o.twist.twist.linear.y), V_MOVING)
    # 旋轉段會同時觸發線速度門檻(反之亦然),各取最長的那段最保險
    rot = [max(rot, key=lambda s: s[1] - s[0])] if rot else []
    lin = [s for s in lin if not rot or not (s[0] < rot[0][1] and rot[0][0] < s[1])]
    lin = [max(lin, key=lambda s: s[1] - s[0])] if lin else []

    print('=== A) 旋轉段:驗 gyro_scale / gyro_z_sign ===')
    if rot:
        analyze_rotation(scans, odoms, rot[0])
    else:
        print('  找不到旋轉段(車有原地轉嗎?)')

    print('\n=== B) 直線段:驗 laser_yaw ===')
    if lin:
        analyze_translation(scans, odoms, lin[0])
    else:
        print('  找不到獨立的直線段(旋轉和直線要分開做,中間停好停滿)')


if __name__ == '__main__':
    main()
