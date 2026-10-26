#!/usr/bin/env python3
"""把 vel_sweep 量到的標定值,一次套進所有互相牽連的地方。

為什麼需要這支
--------------
「改速度要三個地方一起改」(notes/nav2_tuning.md §1)這件事,從 2026-07-28 到現在
已經踩過兩次。更糟的是 2026-10-09 發現的第三個牽連:`min_speed_xy` / `min_speed_theta`
這道速度下限,**本身就是為了繞過 cmd 路徑 8× 衰減而貼的 OK 繃**,校正之後必須
同一次拿掉,否則:

  * 只校正、不拿下限 → `min_speed_xy: 0.06` 的語意從「命令空間 0.0075 m/s」一夕
    變成「真實 0.06 m/s」,嚴格 8 倍。44cm 走道裡會開始噴 no valid trajectories。
  * 只拿下限、不校正 → DWB 又開始挑 0.02-0.03 m/s 的軌跡(真實 0.004 m/s),
    輪子原地打滑、車不動,progress_checker 判定卡住。

這兩件事沒有「先做哪個比較安全」的選項,只有「一起做」。這支就是那個 atomic 的
一起做 —— 它同時改 FollowPath、velocity_smoother、behavior_server 和那道下限,
改完還會把煞停距離、合成速度上限這些推導過的約束重新驗一次。

DWB 為什麼不能停車(這是「到定點一直抖動」的直接原因)
----------------------------------------------------
nav2 humble `dwb_plugins/src/kinematic_parameters.cpp`:

    if (min_speed_xy_ >= 0.0 && vmag_sq + EPSILON < min_speed_xy_sq_ &&
        min_speed_theta_ >= 0.0 && fabs(theta) + EPSILON < min_speed_theta_) {return false;}

兩個條件是 **AND**。速度 (0,0,0) 同時低於兩個門檻 → 被判 illegal → **「停住」這條
軌跡根本不在候選名單裡**。只要 goal checker 還沒 latch,DWB 每個 cycle 都只能從
「平移 ≥ min_speed_xy」或「自轉 ≥ min_speed_theta」裡面挑一條。終點附近路徑只剩
幾公分,PathDist / PathAlign 幾乎分不出高下,就變成在一堆同分的非零軌跡裡亂挑。

把 min_speed_theta 設成 0.0 就能讓整條判斷式永遠為 false(`fabs(theta) < 0.0`
不可能成立),下限整個失效、停車變回合法。本腳本兩個都關,是為了讓意圖在 YAML
上看得出來,而不是靠「其中一個設 0 讓另一個變成死參數」這種隱晦的寫法。

用法
----
    # 1) 先量(三軸都要,見 tools/vel_sweep.py)
    ./robotctl args "cmd_linear_scale:=1.0 cmd_angular_scale:=1.0"
    ./robotctl restart bringup
    python3 tools/vel_sweep.py --axis x
    python3 tools/vel_sweep.py --axis yaw
    python3 tools/vel_sweep.py --axis y

    # 2) 再套(預設是 dry-run,只印 diff,不寫檔)
    python3 tools/apply_calibration.py --lin-scale 7.9 --ang-scale 8.2
    python3 tools/apply_calibration.py --lin-scale 7.9 --ang-scale 8.2 --apply

    # 3) 第一趟刻意跑慢,確認方向和停車都對了再往上推
    python3 tools/apply_calibration.py --lin-scale 7.9 --ang-scale 8.2 \
            --max-vx 0.20 --max-vy 0.15 --max-vtheta 0.8 --apply

改完 nav2_params.yaml 不需要 colcon build(--symlink-install),但要**重啟** nav2
(nav2 在 configure 時讀參數)。driver 那邊的 cmd_*_scale 是 launch arg,腳本最後
會把該下的指令印出來。
"""

import argparse
import difflib
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PARAMS = REPO / 'config' / 'nav2_params.yaml'

# 板子回授的飽和點(raw 2.77,見 CLAUDE.md / driver_node.py)乘上 odom_linear_scale
# 就是 /odom 還能誠實回報的最高速度。超過這個值 /odom 會無聲少報 —— 對 SLAM 和
# AMCL 來說那等於里程計開始說謊,比跑慢還糟。
ODOM_SAT_RAW = 2.77
DEFAULT_ODOM_LINEAR_SCALE = 0.153

# 第一趟上路刻意保守。校正後車子會比操作者看過的快 8 倍,先確認「會停」再談快。
FIRST_RUN_VX = 0.20
FIRST_RUN_VTHETA = 0.8
# 麥克納姆橫移靠滾子,效率天生比直行差。沒有量測值時的保守推定;
# `vel_sweep.py --axis y` 量到之後請用 --max-vy 明確指定。
STRAFE_EFFICIENCY = 0.7


def fmt(v):
    """固定兩位小數。

    ★ **一定要留小數點**,不能把 -1.0 印成 `-1`。nav2 這些速度參數都是以 double
    宣告的,YAML 裡寫成整數會被 rclcpp 當成 integer 型別的 parameter override,
    declare_parameter 當場拋 InvalidParameterTypeException —— 整個 nav2_container
    起不來,而錯誤訊息只會說某個參數型別不對,不會告訴你是因為少了 `.0`。
    """
    return f'{v:.2f}'


def read_scalar(text, key):
    """讀出 `  key: <number>` 的值,只認沒有被註解掉的那一行。"""
    m = re.search(rf'^([ \t]*){re.escape(key)}:[ \t]*(-?[\d.]+)', text, re.M)
    if not m:
        raise KeyError(f'在 {PARAMS.name} 裡找不到未註解的 `{key}:`')
    return float(m.group(2))


def set_scalar(text, key, value, comment=None):
    """就地改寫 `  key: <number>`,保留原本的縮排和行尾註解。"""
    pat = re.compile(rf'^([ \t]*){re.escape(key)}:[ \t]*(-?[\d.]+)([ \t]*)(#.*)?$', re.M)
    if not pat.search(text):
        raise KeyError(f'在 {PARAMS.name} 裡找不到未註解的 `{key}:`')

    def repl(m):
        indent, _old, gap, old_comment = m.groups()
        text_comment = comment or old_comment
        if not text_comment:
            return f'{indent}{key}: {fmt(value)}'
        # 指定了新註解就重算對齊空白(原本的 gap 是對著舊數字算的,會黏住或歪掉)
        pad = gap if (old_comment and not comment) else '    '
        return f'{indent}{key}: {fmt(value)}{pad}{text_comment}'

    return pat.sub(repl, text, count=1)


def set_triple(text, key, values):
    """改寫 `  key: [a, b, c]` 這種三元素速度向量。"""
    pat = re.compile(rf'^([ \t]*){re.escape(key)}:[ \t]*\[[^\]]*\]([ \t]*)(#.*)?$', re.M)
    if not pat.search(text):
        raise KeyError(f'在 {PARAMS.name} 裡找不到未註解的 `{key}:`')
    body = ', '.join(fmt(v) for v in values)

    def repl(m):
        indent, gap, comment = m.groups()
        return f'{indent}{key}: [{body}]{gap}{comment or ""}'.rstrip()

    return pat.sub(repl, text, count=1)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lin-scale', type=float, required=True,
                    help='vel_sweep --axis x 印出的 cmd_linear_scale')
    ap.add_argument('--ang-scale', type=float, required=True,
                    help='vel_sweep --axis yaw 印出的 cmd_angular_scale')
    ap.add_argument('--max-vx', type=float, default=FIRST_RUN_VX,
                    help=f'要跑的真實前進速度 m/s (預設 {FIRST_RUN_VX},第一趟刻意保守)')
    ap.add_argument('--max-vy', type=float, default=None,
                    help='真實橫移速度 m/s (預設 = max-vx × %.2f;量過 --axis y 就填實測)'
                         % STRAFE_EFFICIENCY)
    ap.add_argument('--max-vtheta', type=float, default=FIRST_RUN_VTHETA,
                    help=f'真實自轉速度 rad/s (預設 {FIRST_RUN_VTHETA})')
    ap.add_argument('--odom-linear-scale', type=float, default=DEFAULT_ODOM_LINEAR_SCALE,
                    help=f'算 /odom 飽和天花板用 (預設 {DEFAULT_ODOM_LINEAR_SCALE})')
    ap.add_argument('--params', type=Path, default=PARAMS, help='nav2 參數檔路徑')
    ap.add_argument('--apply', action='store_true',
                    help='真的寫檔。不給這個旗標就只印 diff。')
    args = ap.parse_args()

    if args.lin_scale <= 0 or args.ang_scale <= 0:
        sys.exit('✗ scale 必須是正數。vel_sweep 如果印出負斜率,那是方向反了 —— '
                 '要去修 driver 的 linear_x_sign / angular_z_sign,不是在這裡補負號。')

    max_vx = args.max_vx
    max_vy = args.max_vy if args.max_vy is not None else max_vx * STRAFE_EFFICIENCY
    max_vt = args.max_vtheta
    ceiling = ODOM_SAT_RAW * args.odom_linear_scale

    original = args.params.read_text(encoding='utf-8')
    text = original

    # 驗證要用到的既有值,從檔案本身讀 —— 不要在這裡寫死一份會過期的副本。
    decel_x = abs(read_scalar(text, 'decel_lim_x'))
    goal_tol = read_scalar(text, 'xy_goal_tolerance')
    sim_time = read_scalar(text, 'sim_time')

    # --- 1/3 FollowPath:DWB 產生候選軌跡的範圍 ---
    text = set_scalar(text, 'max_vel_x', max_vx)
    text = set_scalar(text, 'min_vel_x', -max_vx)
    text = set_scalar(text, 'max_vel_y', max_vy)
    text = set_scalar(text, 'min_vel_y', -max_vy)
    text = set_scalar(text, 'max_vel_theta', max_vt)
    text = set_scalar(text, 'max_speed_xy', round(math.hypot(max_vx, max_vy) + 0.05, 2))

    # 速度下限整個關掉(見檔頭:這是「到定點抖動」的直接原因)。
    text = set_scalar(text, 'min_speed_xy', -1.0,
                      comment='# 負值 = 關閉(見上面的說明)')
    text = set_scalar(text, 'min_speed_theta', 0.0,
                      comment='# 0.0 = 關閉(見上面的說明)')

    # --- 2/3 velocity_smoother:最後一關會無聲把指令夾回它自己的上限 ---
    text = set_triple(text, 'max_velocity', [max_vx, max_vy, max_vt])
    text = set_triple(text, 'min_velocity', [-max_vx, -max_vy, -max_vt])

    # --- 3/3 behavior_server:recovery 自轉時才用到,但漏掉就會和 controller 打架 ---
    text = set_scalar(text, 'max_rotational_vel', max_vt)

    # ---------------- 驗算 ----------------
    problems, notes = [], []

    brake = max_vx ** 2 / (2 * decel_x)
    if brake >= goal_tol:
        problems.append(
            f'煞停距離 {brake:.3f} m ≥ xy_goal_tolerance {goal_tol:.2f} m —— 會衝過終點再倒回來。'
            f'\n     修法:降 --max-vx,或把 decel_lim_x 加大(目前 -{decel_x:g})。')
    else:
        notes.append(f'煞停距離 {brake:.3f} m < xy_goal_tolerance {goal_tol:.2f} m ✓')

    if max_vx > ceiling:
        problems.append(
            f'max_vx {max_vx:.2f} 超過 /odom 飽和天花板 {ceiling:.2f} m/s —— '
            f'超過之後里程計會無聲少報,SLAM/AMCL 會開始說謊。')
    elif max_vx > 0.85 * ceiling:
        notes.append(f'⚠ max_vx {max_vx:.2f} 已達天花板 {ceiling:.2f} 的 '
                     f'{max_vx / ceiling * 100:.0f}%,沒剩多少餘裕。')
    else:
        notes.append(f'max_vx {max_vx:.2f} / 天花板 {ceiling:.2f} m/s '
                     f'({max_vx / ceiling * 100:.0f}%) ✓')

    notes.append(f'DWB 前視距離 = sim_time {sim_time:g}s × {max_vx:.2f} m/s '
                 f'= {sim_time * max_vx:.2f} m(走道一格 0.46 m)')

    # ---------------- 輸出 ----------------
    diff = list(difflib.unified_diff(
        original.splitlines(keepends=True), text.splitlines(keepends=True),
        fromfile=f'a/{args.params.name}', tofile=f'b/{args.params.name}'))
    print(''.join(diff) if diff else '(沒有任何改動)')

    print('=' * 68)
    for n in notes:
        print(f'  {n}')
    for p in problems:
        print(f'  ✗ {p}')
    print('=' * 68)

    if problems:
        sys.exit('\n✗ 有約束沒過,沒有寫檔。先解決上面的問題。')

    if args.apply:
        args.params.write_text(text, encoding='utf-8')
        print(f'✓ 已寫入 {args.params}')
    else:
        print('(dry-run,沒有寫檔。確認 diff 沒問題後加 --apply)')

    print(f"""
還有兩個地方不在這個檔案裡,要自己下:

  1) driver 的補償倍率(launch arg,改完要重啟 bringup):

       ./robotctl args "cmd_linear_scale:={args.lin_scale:.2f} cmd_angular_scale:={args.ang_scale:.2f}"
       ./robotctl restart bringup

     確認有吃到:
       ros2 param get /ominibot_driver cmd_linear_scale

     跑順了再把這兩個數字寫回 driver_node.py 和 robot_bringup.launch.py 的預設值,
     否則下次忘了帶 args 就會變回未校正狀態。

  2) teleop 的預設速度(ominibot_driver/teleop_node.py:69-70):

       linear_speed   0.6  ->  {max_vx:.2f}
       angular_speed  1.5  ->  {max_vt:.2f}

     現在的 0.6 / 1.5 是當初手工補償 8× 衰減加上去的,校正後會變成真實 0.6 m/s,
     **超過天花板 {ceiling:.2f}**。改完要 colcon build(ament_python 也一樣):
       colcon build --packages-select ominibot_driver --symlink-install

重啟 nav2 讓新參數生效:./gcs.sh --down && ./gcs.sh --nav <map>
""")


if __name__ == '__main__':
    main()
