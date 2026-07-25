#!/usr/bin/env python3
"""用合成資料釘死 analyze_bag.py 的符號約定。

為什麼要有這支:2026-07-25 第一版 analyze_bag.py 的旋轉符號反了,結果把一台
完全正常的車誤判成「gyro_z_sign 需要翻轉」。符號錯誤在真實資料上看不出來
(數字一樣漂亮,只是意義相反),只有拿已知答案的合成資料餵進去才抓得到。

改動 analyze_bag.py 的相關數學後,務必跑這支:
    python3 tools/selftest_analyze.py
"""

import math
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from analyze_bag import shift_between   # noqa: E402

N = 720
INC = 2 * math.pi / N
A_MIN = -math.pi
IDX = np.arange(N)


def world(ang):
    """一個有足夠特徵、不對稱的假環境(對稱環境會讓互相關有多重峰)。"""
    return 3.0 + 1.5 * np.sin(3 * ang) + 0.7 * np.cos(7 * ang) + 0.3 * np.sin(11 * ang)


def check(name, got, want, tol):
    ok = abs(got - want) < tol
    print(f'  {"PASS" if ok else "FAIL"}  {name}: 期望 {want:+.2f}, 得到 {got:+.2f}')
    return ok


def test_rotation():
    """機器人逆時針轉 +Δ → analyze_bag 的 scan_yaw 應該也是 +Δ。"""
    print('旋轉符號:')
    allok = True
    for deg in (+10.0, -10.0, +25.0):
        d = math.radians(deg)
        # 感測器方位 θ 的光束,在機器人 yaw=y 時指向世界角 θ+y
        r1 = world(A_MIN + IDX * INC + 0.0)
        r2 = world(A_MIN + IDX * INC + d)
        scan_yaw = -shift_between(r1, r2) * INC     # 和 analyze_rotation 同一個式子
        allok &= check(f'機器人轉 {deg:+.1f}°', math.degrees(scan_yaw), deg, 0.5)
    return allok


def test_translation():
    """機器人在光達座標系往 φ 方向平移 → 解出來的 φ 應該一致。"""
    print('平移方向:')
    allok = True
    th = A_MIN + IDX * INC
    for deg in (0.0, +90.0, +167.0, -120.0):
        phi = math.radians(deg)
        d = 0.02
        r1 = world(th)
        # 特徵在方位 θ、距離 r;機器人位移 d·(cos φ, sin φ) 後的新距離
        px, py = r1 * np.cos(th), r1 * np.sin(th)
        qx, qy = px - d * math.cos(phi), py - d * math.sin(phi)
        r2 = np.hypot(qx, qy)
        # 重採樣回原本的方位角格點(近似:小位移下方位變化可忽略)
        dr = r2 - r1
        M = np.stack([np.cos(th), np.sin(th)], axis=1)
        coef, *_ = np.linalg.lstsq(M, dr, rcond=None)
        got = math.degrees(math.atan2(-coef[1], -coef[0]))
        diff = (got - deg + 180) % 360 - 180
        allok &= check(f'往 {deg:+.1f}° 走', deg + diff, deg, 2.0)
    return allok


if __name__ == '__main__':
    ok = test_rotation() & test_translation()
    print('\n全部通過 ✅' if ok else '\n有測試失敗 ❌')
    sys.exit(0 if ok else 1)
