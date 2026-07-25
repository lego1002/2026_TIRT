#!/usr/bin/env python3
"""逐顆輪子單獨測試 —— 找出「有沒有哪一顆輪子沒在出力」。

為什麼需要這支(2026-07-25):
`tools/vel_sweep.py` 掃出來的命令→實際曲線散得一塌糊塗(R² 只有 0.63 / 0.70),
而且正轉反轉不對稱、命令加大實際卻不跟著加大:

    命令 +0.300 → 實際 0.0246        命令 -0.400 → 實際 -0.0259
    命令 +0.500 → 實際 0.0249  ← 沒變  命令 -0.600 → 實際 -0.0908  ← 暴增

固定的標定誤差不會長這樣(那會是一條直線)。這種**又飄又前後不對稱**的曲線,
在麥克納姆底盤上最典型的成因就是四顆輪子出力不平均 —— 某顆馬達弱掉、
編碼器線接觸不良、輪轂固定螺絲鬆了在打滑,或齒輪箱卡住。
四顆輪子出力不均時,車體的合成運動會變得很難預測,這也完全符合「控制很怪」。

--- 原理 ---

用板子的 `motor_speed(m1,m2,m3,m4)` 指令,一次只轉一顆輪子。
板子回報的車體速度(lx/ly/az)是從**四顆編碼器**算出來的,所以只驅動第 N 顆時:

    該輪正常  → 回報速度明顯非零
    該輪不動  → 回報速度 ≈ 0(編碼器沒轉)

⚠️  但這隻眼睛也要用:如果馬達有轉、編碼器線斷了,回報一樣是 0。
    所以請**同時用眼睛看**輪子有沒有轉、用手感覺有沒有異常阻力或異音。
    程式只負責量編碼器,分辨「馬達沒轉」和「編碼器沒訊號」要靠你看。

--- 用法 ---

⚠️  **一定要把車子抬離地面**,四顆輪子懸空。單顆輪子驅動時車子會亂跑。
⚠️  這支會**直接開序列埠**,所以要先把驅動關掉(埠是 exclusive 的,
    沒關的話會直接報 port busy)。

    ./run_robot.sh 那個終端按 Ctrl-C,或
    pkill -f ominibot_driver

    然後:
    python3 tools/wheel_test.py

不需要 ROS,也不需要 DDS —— 這支不經過 ROS,直接跟板子講話。

--- 怎麼判讀 ---

八個「輪子 × 方向」的組合應該**大小相近**。要逐一比,**不要**把同一顆輪子的
兩個方向合併成一個數字 —— 方向不對稱正是這裡最重要的訊號(第一版判讀用了
max(正轉,反轉),把一台每顆輪子各壞一個方向的車判成全部正常)。

單顆單方向偏小/為 0  → 電氣或韌體問題。機械原因(齒輪箱、軸承、鬆掉的輪轂
                      螺絲)都是**雙向對稱**的,不會只挑一個方向壞。
單顆雙向都偏小       → 這才是機械。檢查輪轂固定螺絲、齒輪箱、麥輪滾子。
八個都很小           → 不是單顆的問題,回去看供電或板子設定。
八個都正常且相近      → 輪子沒問題,問題在車體層級(重心、輪子沒同時著地)。
                      把車放回地面用手壓四個角,感覺有沒有一角浮空。
"""

import argparse
import statistics
import sys
import time

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0] + '/ominibot_driver')

from ominibot_driver.ominibot_hv import OminiBotHV  # noqa: E402

PORT = '/dev/serial0'
WHEEL_NAMES = ['M1 (左前)', 'M2 (右前)', 'M3 (左後)', 'M4 (右後)']
# 預設 0.4 而不是 1.0:1.0 rev/s 時板子回報的車體速度會撞到 ~2.77 的天花板,
# 量到的變成「花多久爬到飽和」,結果每跑一次故障輪就換一顆(2026-07-25 實測)。
REV_PER_S = 0.4     # 單輪測試轉速 (rev/s)
SPIN = 1.5          # 收資料的時間
SETTLE = 0.8        # 開始收資料前先轉多久(過渡期,不計入)


def measure(bot, idx, rev, spin=SPIN):
    """只驅動第 idx 顆輪子,回傳這段期間車體回報速度的**中位數**振幅。

    2026-07-25 的第一版在這裡有兩個會製造假故障的問題,連續誤導了兩輪除錯:

    1. 沒清序列埠的輸入緩衝。板子是持續串流的,上一顆輪子(甚至上一次停車)
       的舊訊框還躺在緩衝裡,一開始讀到的是過期資料。
    2. 用平均值。回報值在 ~2.77 就飽和了,所以量到的其實是「輪子花多久爬到
       飽和」—— 起步慢一點的那次,平均就被拉低,看起來像出力不足。哪一顆剛好
       慢每次都不同,於是「故障輪」每跑一次就換一顆。

    現在:先 reset_input_buffer(),過渡期拉長,並取中位數(對起步殘留的低值
    不敏感)。真正的修法還是把轉速降到飽和點以下 —— 見 --rev。
    """
    cmd = [0.0, 0.0, 0.0, 0.0]
    cmd[idx] = rev
    # 先把馬達轉起來並丟掉過渡期,再開始收資料。
    t_go = time.monotonic() + SETTLE
    while time.monotonic() < t_go:
        bot.motor_speed(*cmd)
        time.sleep(0.02)
    try:
        bot.ser.reset_input_buffer()    # 丟掉過渡期堆積的舊訊框
    except Exception:  # noqa: BLE001
        pass

    mags = []
    t_end = time.monotonic() + spin
    while time.monotonic() < t_end:
        bot.motor_speed(*cmd)
        data = bot.read_feedback()
        if data:
            # 單輪驅動時車體會同時有 x/y/yaw 分量,取合成振幅當「這顆有沒有在轉」。
            mags.append(abs(data['lx']) + abs(data['ly']) + abs(data['az']))
        time.sleep(0.02)
    bot.motor_speed(0, 0, 0, 0)
    time.sleep(0.8)     # 等慣性停下來,不要污染下一顆
    try:
        bot.ser.reset_input_buffer()
    except Exception:  # noqa: BLE001
        pass
    return statistics.median(mags) if len(mags) >= 3 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--rev', type=float, default=REV_PER_S,
                    help=f'單輪測試轉速 rev/s (預設 {REV_PER_S})。'
                         '讀數全部貼在同一個值時,調小這個再測一次。')
    ap.add_argument('--repeat', type=int, default=3,
                    help='每個方向重複量幾次 (預設 3)。重複量測是用來抓「量不準」的,'
                         '不要降到 1。')
    ap.add_argument('--port', default=PORT, help=f'序列埠 (預設 {PORT})')
    args = ap.parse_args()

    print(__doc__.split('--- 用法 ---')[0])
    print('=' * 62)
    print('⚠️  車子抬離地面了嗎?四顆輪子要能自由空轉。')
    print('⚠️  ./run_robot.sh 關掉了嗎?(序列埠是獨占的)')
    print('=' * 62)
    try:
        input('確認好按 Enter 開始,Ctrl-C 取消 … ')
    except (KeyboardInterrupt, EOFError):
        print('\n取消。')
        return 1

    try:
        bot = OminiBotHV(port=args.port)
    except Exception as exc:  # noqa: BLE001
        print(f'\n✗ 開不了 {args.port}: {exc}')
        print('  如果是 "Device or resource busy",代表驅動還在跑:')
        print('      pkill -f ominibot_driver')
        return 1

    # 每個「輪子 × 方向」重複量 N 次。**可重現性是這裡的核心** —— 單次抽樣
    # 在 2026-07-25 連續兩次讓「故障輪」每跑一次就換一顆,對硬體下了兩次錯誤
    # 的判決。重複量之後,如果 N 次自己就對不起來,工具會說「量不準」而不是
    # 報一個假故障。
    trials = {}     # (輪子, 方向) -> [n 次的值]
    try:
        print()
        print(f'每個方向重複量 {args.repeat} 次(檢查可重現性)。眼睛也要看!')
        print()
        hdr = ' '.join(f'{"#"+str(k+1):>8}' for k in range(args.repeat))
        print(f'{"輪子":>10} {"方向":>6} {hdr} {"中位":>8} {"離散":>7}')
        print('-' * (18 + 9 * args.repeat + 17))
        for i, name in enumerate(WHEEL_NAMES):
            for label, sign in (('正轉', 1.0), ('反轉', -1.0)):
                vals = [measure(bot, i, sign * args.rev)
                        for _ in range(args.repeat)]
                trials[(name, label)] = vals
                med = statistics.median(vals)
                spread = (statistics.pstdev(vals) / med) if med > 1e-9 else float('inf')
                cells = ' '.join(f'{v:>8.4f}' for v in vals)
                flag = '  ← 不穩!' if spread > 0.15 else ''
                print(f'{name:>10} {label:>6} {cells} {med:>8.4f} '
                      f'{spread*100:>6.0f}%{flag}')
    except KeyboardInterrupt:
        print('\n中斷 —— 停車。')
        return 1
    finally:
        try:
            bot.close()
        except Exception:  # noqa: BLE001
            pass

    results = {}
    unstable = []
    for name in WHEEL_NAMES:
        pair = []
        for label in ('正轉', '反轉'):
            vals = trials[(name, label)]
            med = statistics.median(vals)
            if med > 1e-9 and statistics.pstdev(vals) / med > 0.15:
                unstable.append(f'{name} {label}')
            pair.append(med)
        results[name] = tuple(pair)

    if unstable:
        print()
        print('=' * 62)
        print('✗ 這些組合**重複量測自己就對不起來**:')
        for u in unstable:
            print(f'     {u}')
        print()
        print('  同一顆輪子、同一個方向、連續量幾次卻給出差很多的值 ——')
        print('  真實的硬體故障不會這樣(壞掉的 H 橋或斷掉的編碼器相位是穩定的)。')
        print('  這代表**量測本身不可信**,下面的判讀不要當真。')
        print()
        print('  最可能的原因:轉速太高,回報值撞到天花板(~2.77),')
        print('  量到的其實是「花多久爬到飽和」而不是轉速。降轉速重測:')
        print(f'      python3 {sys.argv[0]} --rev 0.4')
        print('=' * 62)

    # -- 判讀 --------------------------------------------------------------
    # ⚠️ 每個方向要**分開**比。2026-07-25 的第一版用 max(正轉, 反轉) 把兩個方向
    # 壓成一個數字,結果一台「每顆輪子各有一個方向壞掉」的車被判成四顆全正常 ——
    # 方向不對稱正是這裡唯一有意義的訊號,取 max 剛好把它抹掉。
    print()
    print('=' * 62)
    vals = [v for pair in results.values() for v in pair]
    best = max(vals) if vals else 0.0
    if best < 1e-4:
        print('✗ 四顆輪子的編碼器全部沒有回報。')
        print('  不是單顆輪子的問題 —— 檢查馬達電源(板子邏輯電有電不代表馬達電有電)、')
        print('  以及板子的馬達輸出接線。')
        return 0

    bad = []       # (輪子, 方向, 相對百分比)
    for name, (f, r) in results.items():
        for label, v in (('正轉', f), ('反轉', r)):
            pct = v / best * 100
            mark = '✗' if v < 0.6 * best else '✓'
            if v < 0.6 * best:
                bad.append((name, label, pct))
            print(f'  {mark}  {name:<10} {label}  {v:>8.4f}  ({pct:>3.0f}%)')

    # 所有「正常」讀數貼在同一個值上,有兩種完全不同的成因,要分清楚:
    #   (a) 回報值飽和 —— 撞到天花板,輪子之間的差異被壓平看不出來
    #   (b) 空轉無負載下閉環 PID 完美追到設定值 —— 這是正常的
    # 分辨方法:換一個 --rev 再測,看讀數是否成正比。
    # 2026-07-25 實測 rev=1.0 → 2.766、rev=0.4 → 1.106,比值 2.765 一致,
    # 所以這台是 (b),不是飽和。當初誤判成 (a) 是因為沒有做這個比例檢查。
    good = [v for v in vals if v >= 0.6 * best]
    if len(good) >= 3 and statistics.pstdev(good) / max(statistics.mean(good), 1e-9) < 0.02:
        mean_good = statistics.mean(good)
        print()
        print(f'ℹ  「正常」的讀數全部貼在 {mean_good:.3f} 附近,'
              f'每 rev/s 約 {mean_good / max(args.rev, 1e-9):.3f}。')
        print('   這通常是正常的:輪子空轉沒有負載,閉環 PID 會精準追到設定值。')
        print(f'   要確認不是回報值撞到天花板,換個轉速再測一次,看讀數是否成正比:')
        print(f'       python3 {sys.argv[0]} --rev {args.rev * 2:.1f}')
        print(f'   讀數約變成 {mean_good * 2:.3f} → 正比,沒有飽和問題。')

    print()
    if not bad:
        print('✓ 八個方向都正常。')
        print()
        print('  ⚠ 但這支測的是**空轉**,沒有負載。四顆輪子都輕鬆追到設定值是意料中的,')
        print('    所以這個結果只能排除「馬達完全不動 / 編碼器完全沒訊號」這種硬故障,')
        print('    **不能**證明四顆輪子在真實負載下的出力是平均的。')
        print()
        print('  那 vel_sweep 的散亂就要往車體層級找:')
        print('  - 把車放回地面,用手壓四個角,看是不是有一角浮空')
        print('    (麥輪車只要一輪沒確實著地,運動就會完全跑掉)')
        print('  - 重心是否過度偏向一側')
        print('  - 地面材質(麥輪在地毯/不平整地面上會嚴重打滑)')
        return 0

    print('⚠  找到異常的「輪子+方向」組合:')
    for name, label, pct in bad:
        print(f'     {name} {label} ({pct:.0f}%)')
    print()
    # 同一顆輪子只有單一方向壞掉,和整顆都壞掉,病因完全不同。
    per_wheel = {}
    for name, label, _ in bad:
        per_wheel.setdefault(name, []).append(label)
    one_way = [n for n, dirs in per_wheel.items() if len(dirs) == 1]
    if one_way:
        print('   ★ 這些輪子是**只有一個方向**有問題:' + '、'.join(one_way))
        print('     單方向失效基本上排除機械原因 —— 齒輪箱、軸承、鬆掉的輪轂螺絲')
        print('     都是雙向對稱的,不會挑方向。要往電氣/韌體找:')
        print()
        print('     1. 馬達驅動 H 橋有半邊掛掉(該顆馬達只能單向出力)')
        print('     2. 編碼器只有一相有訊號 —— 缺一相就判不出方向,')
        print('        板子在某個方向會算成 0。檢查該顆的編碼器 A/B 兩條線。')
        print('     3. 板子的 encoder_direct / motor_direct 設定與實際接線不合')
        print()
    print('   ※ 關鍵的分辨動作(程式做不到,要你的眼睛):')
    print('     重跑一次,**盯著讀到 0 或偏低的那顆輪子看**。')
    print('       輪子有在轉,程式卻讀到 0  → 編碼器問題(訊號),馬達是好的')
    print('       輪子根本沒轉              → 馬達/H 橋問題(出力)')
    print('     這兩種修法完全不同,一定要先分清楚。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
