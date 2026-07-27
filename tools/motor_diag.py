#!/usr/bin/env python3
"""底盤控制問題的總合診斷 —— 一次同時量「序列鏈路」「車體抖動」「四輪同步」。

為什麼需要這支(2026-07-26):
症狀是「上禮拜還好好的,現在車子一頓一頓」+「輪子懸空測都正常,放到地板加了
摩擦力就開始怪」。這兩句話指向的病因完全不同,而現有工具**沒有一支能分辨**:

    tools/wheel_test.py   只在空中測單顆輪子(無負載),測不到負載下的行為
    tools/vel_sweep.py    走 ROS /odom,量的是車體層級,看不到四顆輪子
    tools/odom_check.py   量累積位移,專門校 scale,不看抖動

而且最關鍵的數字整個系統都沒人收:`read_feedback()` 對 timeout、desync、
BCC 錯**一律回傳 None**,呼叫端直接重試 —— 所以「鏈路很乾淨」和「鏈路一直在
掉訊框」長得一模一樣。2026-07-19 板子從 USB(FTDI) 改接 Pi 的 GPIO UART 之後,
這件事變得很要緊:git 考古顯示從「還好好的」到現在,命令路徑的軟體**實質上沒
有變過**(只有 port 名稱、exclusive=True、輪距 110/110→115/96 的 4% 差異,
cmd_*_scale 預設 1.0 是 no-op),唯一真正換掉的就是那條實體線。

--- 這支在量什麼(三個互相獨立的軸) ---

1. **鏈路健康**(ominibot_hv.stats):good / bcc_fail / desync / timeout。
   *靜止時*就在掉訊框 → 接線、鮑率、或兩個驅動源搶同一條線(板子上的 USB
   晶片若還在驅動 STM32 的 USART 腳,和 Pi 的 GPIO 會對打)。
   *只有馬達出力時*才掉 → 電氣雜訊(地回路/走線)。

2. **車體抖動**:命令固定不變時,回報速度掉到接近 0 的比例(dropout)。
   一頓一頓的物理量化 —— 若板子收不到有效命令訊框,它自己的 watchdog 會把馬達
   歸零,下一個好訊框又動,於是每秒抖幾下。dropout 高 + bcc_fail 高 = 同一件事。

3. **四輪同步**(0x36 readback):四顆輪子各自的編碼器速率。
   純前進(或純旋轉)時四顆的**大小應該相等**,離散度就是不同步的程度。

--- 為什麼非要 0x36 才看得到同步 ---

串流訊框回報的是車體速度,韌體已經用麥輪正運動學(PDF p.33)把**四顆編碼器
壓成三個數字**。那個投影剛好丟掉 (V1-V2-V3+V4) 這一個自由度 —— 也就是不同步
/打滑模式。所以「四顆輪子有沒有跟上彼此」從 /odom 再怎麼後處理都算不出來,
這是自由度不夠,不是分析方法的問題。

⚠️  0x36 在這塊板子上**還沒驗證過**(廠商範例只實作 0x33/0x34/0x50)。板子不
    回應時這支會說「不支援」然後繼續跑另外兩軸 —— 不要把「沒讀到」當成故障。

--- 用法 ---

⚠️  這支**直接開序列埠**,要先把驅動關掉(埠是 exclusive 的):
        pkill -f ominibot_driver          # 或在 ./run_robot.sh 那個終端按 Ctrl-C

    **一定要跑兩次做對照**,這是整支工具的重點:

        python3 tools/motor_diag.py --label air     # 車子抬高,四輪懸空
        python3 tools/motor_diag.py --label floor   # 車子放地上,留 1.5m 淨空

⚠️  floor 那次車子會前後移動。Ctrl-C 立刻停車。

--- 怎麼判讀(兩次的差異才是答案) ---

    air 就在掉訊框            → 鏈路/接線問題,和負載無關。先修線再談控制。
    air 乾淨、floor 掉訊框    → 電氣雜訊(馬達電流耦合進 UART)。
    兩次都乾淨但 floor 抖     → 不是鏈路。往扭矩餘裕/PID/機械找:
                                板子 motor_pwm_max=3600(duty 上限 50%),12V 供電
                                下馬達最多吃到 6V —— 12V 馬達的扭矩餘裕被砍半,
                                負載一重 PID 就飽和。試 motor_pwm_max:=6800。
    floor 四輪離散度大        → 該顆吃不到設定值(飽和或抓地不良)。
    dropout 高但 bcc_fail=0   → 板子有收到命令卻自己停,不是鏈路 → 看電池電壓。
"""

import argparse
import statistics
import sys
import time

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0] + '/ominibot_driver')

from ominibot_driver.ominibot_hv import OminiBotHV, REPLY_MOTOR_VEL  # noqa: E402

PORT = '/dev/serial0'
# 預設掃這幾個命令值。板子的內部標定和這台差 ~6.5x,所以這裡的數字是「板子單位」,
# 不是真實 m/s —— 0.15 大約只會跑 0.02 m/s。要的是涵蓋「幾乎不動」到「明顯在跑」。
SPEEDS = [0.0, 0.2, 0.4, 0.8]
HOLD = 3.0          # 每個速度收多久
SETTLE = 0.7        # 換速度後先丟掉的過渡期
CMD_HZ = 20.0       # 命令重送頻率(和 driver_node 的 cmd_rate 一致)
WHEEL_NAMES = ('M1', 'M2', 'M3', 'M4')


def phase(bot, axis, speed, hold, want_wheels):
    """固定命令跑一段,回傳這段的鏈路統計、車體速度樣本、四輪速率樣本。"""
    def send():
        if axis == 'yaw':
            bot.robot_speed(0.0, 0.0, speed)
        elif axis == 'y':
            bot.robot_speed(0.0, speed, 0.0)
        else:
            bot.robot_speed(speed, 0.0, 0.0)

    # 過渡期:先把馬達轉起來,並把緩衝裡的舊訊框丟掉。板子是持續串流的,不清
    # 緩衝的話一開始讀到的是上一個速度的殘留(wheel_test.py 曾被這個坑過兩輪)。
    t_go = time.monotonic() + SETTLE
    while time.monotonic() < t_go:
        send()
        time.sleep(1.0 / CMD_HZ)
    try:
        bot.ser.reset_input_buffer()
    except Exception:  # noqa: BLE001
        pass
    bot.reset_stats()

    vel = []            # 車體速度(這個軸)
    wheels = []         # (m1, m2, m3, m4)
    last_cmd = 0.0
    last_req = 0.0
    t_end = time.monotonic() + hold
    while time.monotonic() < t_end:
        now = time.monotonic()
        if now - last_cmd >= 1.0 / CMD_HZ:
            send()
            last_cmd = now
        # 每 0.25s 問一次四輪速率。回覆是 read_feedback() 順手收進 last_reply 的,
        # 所以這裡不會為了等回覆而漏掉串流樣本。
        if want_wheels and now - last_req >= 0.25:
            bot.request_readback(REPLY_MOTOR_VEL)
            last_req = now

        data = bot.read_feedback()
        if data:
            vel.append(data['az'] if axis == 'yaw'
                       else (data['ly'] if axis == 'y' else data['lx']))
        reply = bot.last_reply.get(REPLY_MOTOR_VEL)
        if reply and reply.get('_taken') is None and reply['bcc_ok']:
            reply['_taken'] = True
            p = reply['payload']
            wheels.append(tuple(
                int.from_bytes(p[2 + 2 * i:4 + 2 * i], 'big', signed=True) / 1000.0
                for i in range(4)))

    return dict(stats=dict(bot.stats), vel=vel, wheels=wheels)


def dropout_rate(vel):
    """回報速度掉到中位數 20% 以下的樣本比例 —— 「一頓一頓」的量化。

    用中位數當基準而不是命令值,因為命令和實際差了 ~6.5x 的固定倍率;這裡要問的
    是「速度有沒有間歇性崩到 0」,不是「速度對不對」。
    """
    if len(vel) < 5:
        return float('nan')
    mag = [abs(v) for v in vel]
    med = statistics.median(mag)
    if med < 1e-6:
        return float('nan')     # 根本沒動,dropout 沒有意義
    return sum(1 for m in mag if m < 0.2 * med) / len(mag)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--label', default='(未命名)',
                    help='這次的情境標籤,例如 air / floor。**一定要跑兩次做對照**。')
    ap.add_argument('--axis', default='x', choices=('x', 'y', 'yaw'),
                    help='測哪個軸(預設 x 前進;純前進時四輪速率應該相等)')
    ap.add_argument('--speeds', type=float, nargs='+', default=SPEEDS,
                    help=f'要掃的命令值,板子單位 (預設 {SPEEDS})')
    ap.add_argument('--hold', type=float, default=HOLD, help=f'每個速度收多久 (預設 {HOLD}s)')
    ap.add_argument('--port', default=PORT, help=f'序列埠 (預設 {PORT})')
    args = ap.parse_args()

    print(__doc__.split('--- 用法 ---')[0])
    print('=' * 70)
    print(f'情境標籤: {args.label}    軸: {args.axis}')
    if args.label == 'air':
        print('⚠️  車子抬離地面了嗎?四顆輪子要能自由空轉。')
    else:
        print('⚠️  前方留 1.5m 以上淨空(車子會前後跑)。')
    print('⚠️  ./run_robot.sh / ominibot_driver 關掉了嗎?(序列埠是獨占的)')
    print('=' * 70)
    try:
        input('確認好按 Enter 開始,Ctrl-C 取消 … ')
    except (KeyboardInterrupt, EOFError):
        print('\n取消。')
        return 1

    try:
        bot = OminiBotHV(port=args.port)
    except Exception as exc:  # noqa: BLE001
        print(f'\n✗ 開不了 {args.port}: {exc}')
        print('  "Device or resource busy" = 驅動還在跑:  pkill -f ominibot_driver')
        return 1

    # 先探一次 0x36 支不支援。不支援就只跑鏈路+抖動兩軸,不要讓整支掛掉。
    print('\n探測 0x36 (四輪編碼器讀回) …', end=' ', flush=True)
    probe = bot.read_motor_speeds(timeout=1.5)
    want_wheels = probe is not None
    print(f'✓ 支援,讀到 {tuple(round(v, 3) for v in probe)}' if want_wheels
          else '✗ 板子沒回應 → 這次沒有四輪同步資料(其餘照跑)')

    results = []
    try:
        print()
        print(f'{"命令":>6} {"速度中位":>9} {"抖動CV":>7} {"dropout":>8} '
              f'{"good/s":>7} {"bcc錯":>6} {"desync":>7} {"timeout":>8}'
              + (f' {"四輪離散":>9}' if want_wheels else ''))
        print('-' * (56 + (10 if want_wheels else 0)))
        for spd in args.speeds:
            r = phase(bot, args.axis, spd, args.hold, want_wheels)
            r['cmd'] = spd
            st, vel = r['stats'], r['vel']
            mag = [abs(v) for v in vel]
            med = statistics.median(mag) if mag else 0.0
            cv = (statistics.pstdev(mag) / med) if med > 1e-6 else float('nan')
            drop = dropout_rate(vel)
            r['med'], r['cv'], r['drop'] = med, cv, drop

            # 四輪離散度:同一時刻四顆的大小差多少(用四顆的絕對值,因為某些軸
            # 上左右輪轉向相反)。scale 無關,所以板子單位沒校正也能比。
            spread = float('nan')
            if r['wheels']:
                per_sample = []
                for w in r['wheels']:
                    a = [abs(x) for x in w]
                    m = statistics.mean(a)
                    if m > 1e-6:
                        per_sample.append(statistics.pstdev(a) / m)
                if per_sample:
                    spread = statistics.median(per_sample)
            r['spread'] = spread

            print(f'{spd:>6.2f} {med:>9.4f} {cv:>7.2f} '
                  f'{drop * 100 if drop == drop else float("nan"):>7.1f}% '
                  f'{st["good"] / args.hold:>7.1f} {st["bcc_fail"]:>6} '
                  f'{st["desync"]:>7} {st["timeout"]:>8}'
                  + (f' {spread * 100 if spread == spread else float("nan"):>8.1f}%'
                     if want_wheels else ''))
            results.append(r)
    except KeyboardInterrupt:
        print('\n中斷 —— 停車。')
        return 1
    finally:
        try:
            bot.close()
        except Exception:  # noqa: BLE001
            pass

    # -- 判讀 -------------------------------------------------------------
    print()
    print('=' * 70)
    print(f'判讀({args.label}):')
    idle = results[0] if results and abs(results[0]['cmd']) < 1e-9 else None
    moving = [r for r in results if abs(r['cmd']) > 1e-9]

    def link_bad(r):
        st = r['stats']
        total = st['good'] + st['bcc_fail'] + st['desync'] + st['short']
        return (st['bcc_fail'] + st['desync'] + st['short']) / max(total, 1)

    if idle is not None:
        rate = link_bad(idle)
        print(f'  · 靜止時鏈路壞訊框比例: {rate * 100:.1f}%')
        if rate > 0.02:
            print('    ✗ 靜止就在掉訊框 —— **和負載無關**,這是接線/電氣層問題。')
            print('      馬達沒出力也錯,就不可能是馬達雜訊。要查:')
            print('      1. 板子上的 USB 轉序列晶片是否還在驅動 STM32 的 USART 腳')
            print('         → 兩個驅動源對打。這是 USB 改 UART 後最容易中的。')
            print('      2. TX/RX 有沒有接反或虛接、跳線是否過長')
            print('      3. 鮑率:兩邊都必須是 115200 8N1')
        else:
            print('    ✓ 靜止時鏈路乾淨。')

    if moving:
        worst = max(moving, key=link_bad)
        idle_rate = link_bad(idle) if idle is not None else 0.0
        worst_rate = link_bad(worst)
        print(f'  · 出力時最差的壞訊框比例: {worst_rate * 100:.1f}% (命令 {worst["cmd"]:.2f})')
        if worst_rate > 0.02 and worst_rate > 3 * max(idle_rate, 0.002):
            print('    ✗ 只有馬達出力時才掉訊框 → **電氣雜訊**(馬達電流耦合進 UART)。')
            print('      地線已經接了還是這樣的話:UART 跳線遠離馬達線、TX/RX 各自和')
            print('      GND 絞在一起、或在板子端 RX 對 GND 加 100nF。')
        elif worst_rate <= 0.02:
            print('    ✓ 出力時鏈路也乾淨 → **一頓一頓不是鏈路造成的**,往下看抖動。')

        drops = [r for r in moving if r['drop'] == r['drop'] and r['drop'] > 0.1]
        if drops:
            print(f'  · 有 {len(drops)} 個速度出現明顯 dropout(速度間歇崩到 0)。')
            if worst_rate <= 0.02:
                print('    → 鏈路乾淨卻還是崩,表示板子**收到命令了卻自己停**。查:')
                print('      1. 電池電壓(/battery_voltage 或這支印的);壓降會讓扭矩掉')
                print('      2. 扭矩飽和:motor_pwm_max=3600 = duty 上限 50%,12V 供電下')
                print('         12V 馬達只吃到 6V。試 ./run_robot.sh motor_pwm_max:=6800')
                print('      3. PID 過衝:vel_ki:=0 vel_kp:=1500 看抖動是否變小')
        else:
            print('  · 沒有明顯 dropout。')

        cvs = [r['cv'] for r in moving if r['cv'] == r['cv']]
        if cvs and max(cvs) > 0.5:
            print(f'  · 速度抖動很大(CV 最高 {max(cvs):.2f})→ 閉環在振盪或 stick-slip。')
            print('    先試 vel_ki:=0(去掉積分,積分在死區會 windup 然後暴衝)。')

        spreads = [r['spread'] for r in moving if r['spread'] == r['spread']]
        if spreads:
            print(f'  · 四輪離散度中位數最高 {max(spreads) * 100:.1f}%')
            if max(spreads) > 0.15:
                print('    ✗ 四顆輪子沒同步 —— 有輪子追不到設定值。')
                print('      在 floor 才出現 → 扭矩餘裕不足或該角抓地不良(壓四個角看是否浮空)')
                print('      在 air 就出現   → 電氣/機械,回去跑 tools/wheel_test.py')
            else:
                print('    ✓ 四顆輪子彼此跟得上。')
        elif want_wheels:
            print('  · 沒收到足夠的四輪樣本(0x36 回覆太少)。')

    print()
    print(f'  ⚠ 單獨一次沒有結論。請再跑另一個情境並比較:')
    other = 'floor' if args.label == 'air' else 'air'
    print(f'      python3 {sys.argv[0]} --label {other}')
    print('=' * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())
