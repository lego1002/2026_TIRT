# SLAM 學習筆記(本專案實戰版)

寫給 2026 TIRT 迷宮機器人專案。前半是 SLAM 的核心觀念,後半對照本專案的實際檔案、
參數、以及 2026-07-14 實測遇到的四個問題(紅箭頭滿地、地圖破碎、停車後模型倒退、
模型看起來太小)的成因與修法。

---

## 一、SLAM 是什麼:雞生蛋、蛋生雞

SLAM = **S**imultaneous **L**ocalization **A**nd **M**apping(同時定位與建圖)。

它要同時解兩個互相依賴的問題:

- **定位(Localization)**:「我在地圖的哪裡?」→ 需要一張地圖才能回答。
- **建圖(Mapping)**:「牆壁在哪裡?」→ 光達只量得到「牆離**我**多遠」,
  要把牆畫到地圖上,得先知道**我自己**在地圖的哪裡。

兩個問題互為前提,這就是 SLAM 的雞生蛋問題。解法的直覺:

1. 開機那一刻,把當下位置定為原點,把第一幀光達掃描直接畫成初始地圖。
2. 機器人移動一小段(由**里程計**粗估移動量)。
3. 拿新的光達掃描去跟已建好的地圖比對(**scan matching**),
   修正里程計的誤差,得到「比較準的位置」。
4. 用修正後的位置把新掃描畫進地圖。
5. 回到 2,反覆進行。

只要每一步的誤差都被 scan matching 壓住,地圖就能越長越大而不散掉。

---

## 二、三個座標系:map → odom → base_link(REP-105)

ROS 的標準 TF 樹(本專案完全遵循):

```
map ──(slam_toolbox 發布)──► odom ──(ominibot_driver 發布)──► base_link
                                                                ├─► lidar_link      (robot_state_publisher,固定)
                                                                └─► 四顆輪子 link    (robot_state_publisher)
```

| Frame | 誰發布 | 特性 |
|---|---|---|
| `odom → base_link` | `ominibot_driver`(輪式里程計積分) | **平滑、連續**,但誤差會隨時間**累積漂移** |
| `map → odom` | `slam_toolbox`(scan matching 修正量) | **準確、不漂移**,但修正時會**跳動** |

這個「雙層設計」是刻意的:

- 里程計短時間很準(幾公分內)、更新快(本板 ~30Hz)、絕不跳動 —— 適合給控制迴路用。
- 但它是**純積分**(dead reckoning),輪徑誤差、打滑、幾何參數錯,誤差只進不出。
- slam_toolbox 不直接去改 `odom → base_link`(那是 driver 的),而是把「里程計到目前為止
  累積了多少誤差」放進 `map → odom` 這一段。所以**每次 scan matching 修正,
  你在 RViz(Fixed Frame=map)看到的就是機器人模型「跳」一下** —— 跳的量 = 這段時間
  里程計積累的誤差量。跳得越大,代表里程計越不準。

> **這直接解釋了「按停止鍵後模型往後退」**:見第六節問題 3。

---

## 三、里程計(Odometry):一切誤差的源頭

### 3.1 本專案的里程計怎麼來的

OminiBotHV 板子上每顆 N20 馬達有編碼器。板子韌體做的事:

```
編碼器脈衝 ──(encoder_ppr=165、gear_ratio=55)──► 輪子轉速 (rev/s)
輪子轉速 ──(wheel_diameter,輪徑)──► 輪緣線速度 (m/s)
四輪線速度 ──(麥輪運動學逆解,用 wheel_space + axle_space)──► 車體速度 (vx, vy, ωz)
```

板子把 `(vx, vy, ωz)` 以 ~30Hz 回傳,`driver_node.py` 的 `_publish()` 再做**中點積分**
(midpoint integration)累積成位置 `(x, y, θ)`,發布 `/odom` 和 `odom→base_link` TF。

### 3.2 麥克納姆輪的先天弱點

麥輪靠 45° 滾子的側向滑動實現全向移動 —— 「滑動」是它工作原理的一部分,所以:

- **橫移(strafe)時里程計最不準**(y 方向誤差可達 10–20%);
- **原地旋轉**也比差速輪容易打滑;
- 前進/後退相對最準。

結論:麥輪車的 odom 品質天生比差速車差,**幾何參數必須校到準,而且建圖時要開慢**,
把剩下的誤差留給 scan matching 收拾。

### 3.3 幾何參數錯誤 → 誤差的三種型態

| 參數錯誤 | 效果 | 地圖症狀 |
|---|---|---|
| `wheel_diameter` 錯(例:板子預設 60mm,實際 48mm) | 回報速度被放大 60/48 = **1.25 倍** → odom 距離多報 25% | 直走時模型超前實際 → scan matching 每次把它**拉回來**(模型倒退跳);牆壁前後重影 |
| `wheel_space` / `axle_space` 錯 | ωz(自轉角速度)被等比例縮放 → **航向角 θ 積分錯** | 轉彎後走廊「彎掉」、本來 90° 的牆變成銳角/鈍角、整張地圖扇形散開 |
| `encoder_ppr` / `gear_ratio` 錯 | 和輪徑一樣是純比例誤差 | 同輪徑錯 |

**角度誤差比距離誤差毒 10 倍**:距離錯 5cm 就是錯 5cm;航向錯 5°,走 2 公尺後
位置就錯了 2 × sin(5°) ≈ 17cm,而且越走越歪。地圖「破碎、亂」十之八九是**角度**出問題。

---

## 四、Scan Matching 與 Pose Graph:slam_toolbox 的內部運作

### 4.1 Scan Matching(相關性掃描匹配)

每當機器人移動超過 `minimum_travel_distance`(或轉超過 `minimum_travel_heading`),
slam_toolbox 取一幀掃描,做這件事:

1. 以 odom 給的預測位置為中心,開一個搜索窗
   (大小 = `correlation_search_space_dimension`,本專案 0.5m);
2. 在窗內平移 + 旋轉這幀掃描,找「和已建地圖重合度最高」的位姿;
3. 重合度分數夠高(`link_match_minimum_response_fine`)就採用,
   把 odom 預測和匹配結果的差值更新進 `map → odom`。

**重要推論**:如果 odom 誤差大到真實位置跑出 0.5m 搜索窗外,scan matching 會
匹配失敗或**匹配到錯的地方** → 地圖疊歪、破碎。odom 越爛,越需要加大搜索窗,
但窗越大越慢、也越容易在「長得很像的地方」(迷宮走廊!)配錯。**治本是把 odom 校準。**

### 4.2 Pose Graph(位姿圖)

slam_toolbox 不是只維護「一張圖」,而是維護一張**圖(graph)**:

- **節點(node)**= 某時刻的機器人位姿 + 那一刻的雷射掃描;
- **邊(edge)**= 兩節點間的相對位姿約束(來自 odom 或 scan matching),帶不確定度。

地圖只是這張圖的「渲染結果」:把每個節點的掃描按節點位姿畫到佔據格上。
所以當圖被優化、節點位姿被改動,**整張地圖會重畫** —— 這就是為什麼建圖中
地圖有時會整片「動一下」,那是正常的全域修正,不是壞掉。

### 4.3 Loop Closure(迴環閉合)

走一圈回到出發點時,累積誤差可能讓「同一個地方」在圖上出現兩次。slam_toolbox 會:

1. 發現目前掃描和**很久以前**的節點附近的掃描很像
   (搜索半徑 `loop_search_maximum_distance`,本專案 3m);
2. 匹配成功(分數 > `loop_match_minimum_response_coarse/fine`)就在兩節點間加一條邊;
3. 交給 **Ceres solver** 做全域優化:把整條軌跡「橡皮筋式」拉直,讓所有約束同時最滿足。

迷宮場景迴環特別多(走廊繞圈),loop closure 是把迷宮圖收乾淨的關鍵。
但反面是:迷宮走廊長得都一樣,**假迴環**(配錯地方)風險也高 ——
`loop_match_minimum_chain_size: 10` 就是防假迴環的門檻(要連續 10 個節點都對得上才算)。

### 4.4 佔據柵格地圖(Occupancy Grid)

`/map` 是一張格子圖,每格 `resolution`(本專案 0.05m = 5cm)見方,值有三種:
- **佔據**(黑,100):有牆;
- **空閒**(白,0):雷射穿過去了,確定沒東西;
- **未知**(灰,-1):還沒看過。

每幀掃描做 ray casting:雷射打到的那格 +佔據證據,途經的格子 +空閒證據,
多幀累積後取閾值。這就是為什麼**同一面牆掃越多次越清晰**,
也是為什麼 odom 不準時牆會「糊開」—— 同一面牆被畫在略不同的位置,證據互相抵消。

---

## 五、本專案的完整資料流

```
[OminiBotHV 板] ─串列─► ominibot_driver ──► /odom + odom→base_link TF (~30Hz)
                              ▲    │
        /cmd_vel (teleop) ────┘    └─► /imu(目前只有 quaternion,未融合)

[RPLidar C1] ─串列─► sllidar_ros2 ──► /scan (~10Hz)

/scan + odom→base_link ──► slam_toolbox ──► /map + map→odom TF

URDF ──► robot_state_publisher ──► base_link→lidar_link、輪子 TF

RViz(PC 端,Fixed Frame = map)訂閱 /map、/scan、/robot_description、TF、/odom
```

相關檔案:
- 驅動與幾何參數:`ominibot_driver/ominibot_driver/driver_node.py`(ROS 參數)、
  `ominibot_hv.py`(送給韌體的設定幀)
- SLAM 參數:`~/ros2_ws/src/my_robot_lidar/config/mapper_params_online_async.yaml`(**不在本 repo**)
- RViz 顯示:`car_assemble_description/rviz/view_robot.rviz`

---

## 六、實測問題對照表(2026-07-14)

### 問題 1:地圖上一堆紅色箭頭

**這不是 bug,是 RViz 的 Odometry 顯示元件。** `view_robot.rviz` 原本設
`Keep: 50` —— 沿路每隔 0.1m(Position Tolerance)留一支箭頭、最多留 50 支,
本意是畫出「里程計軌跡麵包屑」。兩個因素讓它變災難:

1. 箭頭尺寸(Shaft 0.3 + Head 0.1 = **0.4m**)比整台車(0.152m)還長 2.6 倍;
2. odom 有誤差時,箭頭軌跡會和 scan matching 修正後的機器人位置**分岔**,
   看起來就是一堆紅箭頭亂指。

**修法(已改進 `view_robot.rviz`,PC 端 `git pull` 後生效)**:`Keep: 50 → 1`
(只顯示當前位姿)、箭頭縮小到 Shaft 0.08 / Head 0.03。
其實那 50 支箭頭的「分岔量」正是里程計誤差的可視化 —— 校 odom 時可以暫時調回
`Keep: 50` 當診斷工具:**箭頭軌跡和機器人實際軌跡岔多開,odom 就有多不準**。

### 問題 2:模型看起來太小,想放大去配合障礙物

**不要放大模型 —— 模型是對的,地圖才是錯的。** 已實際解析 STL 驗證:
`base_link` 包圍盒 152 × 103 × 93 mm、輪子直徑 48.5mm,和實車完全一致
(SolidWorks 匯出單位就是公尺,RViz 裡 1 格 Grid = 1m,車佔 0.15 格,本來就該這麼小)。

「模型和障礙物比例不對」的真正原因:

1. **0.4m 的紅色大箭頭**貼在 0.15m 的車上,視覺上把車比小了(問題 1 已修);
2. **地圖被里程計誤差撐糊**:牆壁重影、糊開後,walls 之間的「空地」看起來比實際寬,
   車就顯得小。把第 3 題的 odom 校準修好,地圖收緊後比例就對了。

如果想確認:在 RViz 用 Measure 工具量地圖上一段已知長度的牆(例如迷宮一格),
和捲尺量實物比 —— 光達的距離是公制真值,正常情況誤差應 < 2–3cm。

### 問題 3:按停止鍵後模型往後退 ⭐ 最重要

這是**里程計距離高報 + scan matching 回拉**的教科書症狀:

1. 板子韌體出廠預設 `wheel_diameter=60`(mm),實輪是 **48mm**。
   韌體用 60mm 換算輪速 → 回報速度放大 60/48 = **1.25 倍**;
2. 前進 1m,odom 說走了 1.25m → RViz 裡模型跑得比實車遠;
3. 移動中 scan matching 每 0.2m(`minimum_travel_distance`)修正一次,每次往回拉一點;
   **停車瞬間**最後一次修正把積欠的誤差一次結清 → 模型明顯「倒退嚕」。

**修法**:`driver_node.py` 已加 `wheel_diameter_mm` 參數並預設 48,開機時經
`ominibot_hv.py` 的 `\x7b\x24` 設定幀寫進韌體。**重啟 bringup(`./run_robot.sh`)才生效。**
修完做第七節的直線校準驗證。

次要成因(修完輪徑後若還有小幅倒退再查):
- 停止時 PID(`vel_kp=3000` 相當硬)煞車過衝,輪子真的短暫反轉,odom 如實積分 —— 屬實動作,無妨;
- `wheel_space_mm`/`axle_space_mm` 仍是廠設 110/110,**還沒對實車量過**,影響旋轉刻度(見問題 4)。

### 問題 4:地圖破碎、雜亂

綜合症,按優先順序排查:

1. **輪徑錯(同問題 3)** —— 距離刻度錯 25%,直走的牆前後重影。已修,重啟生效。
2. **旋轉刻度錯** —— `wheel_space_mm`/`axle_space_mm`(110/110)是廠設值,沒量過實車。
   麥輪運動學中 ωz 的換算含 `(wheel_space + axle_space)/2`,錯了 → 每次轉彎航向角
   積分錯 → 走廊彎折、地圖扇形展開。**做第七節的旋轉校準。**
3. **建圖開太快** —— 麥輪打滑 + C1 只有 10Hz(轉太快時一幀掃描本身就被「拖糊」)。
   建圖時:直線 ≤ 0.15 m/s,**旋轉 ≤ 0.3 rad/s**,轉彎前先停一拍。
4. **橫移建圖** —— 麥輪橫移的 odom 最爛,建圖階段盡量只用前進+原地轉,少用斜移/橫移。
5. 以上都修完地圖還糊,才動 slam_toolbox 參數(第八節)。

---

## 七、校準程序(修完參數必做)

### 7.1 直線刻度(驗 wheel_diameter)

```bash
# 終端 A:盯住 odom
ros2 topic echo /odom --field pose.pose.position
# 終端 B:teleop 慢速直線前進,實地用捲尺量 1.00m 後停
```

- odom 的 x 位移應為 1.00m ± 3%。
- 高報(如 1.25m)→ 韌體用的輪徑比實輪大,`wheel_diameter_mm` 調小;低報則調大。
- 修正公式:`新輪徑 = 舊輪徑 × 實際距離 / odom距離`。

### 7.2 旋轉刻度(驗 wheel_space / axle_space)

```bash
ros2 topic echo /odom --field pose.pose.orientation
# teleop 原地慢轉,對地板記號精確轉 2 圈(720°)回到原朝向
```

- quaternion 應回到起始值(z、w 同號同值;轉 2 圈而不是 1 圈,把誤差放大一倍好觀察)。
- 沒轉夠 → 韌體以為車比實際小 → `wheel_space_mm + axle_space_mm` 要調大;轉過頭則調小。
- 兩者只影響「和」,先按實車量:輪距 = 左右輪**中心**距,軸距 = 前後軸距,
  量完還差再微調其中一個。

### 7.3 驗收

兩項都過後,再跑一次 SLAM 繞房間一圈:
- 停車時模型不再倒退(或只剩 1–2cm 的微跳);
- 同一面牆只有一條線,90° 牆角是直角;
- 回到出發點時 loop closure 不需要大幅拉扯地圖。

---

## 八、關鍵參數速查表

### ominibot_driver(launch 參數,`./run_robot.sh xxx:=yyy` 直接帶)

| 參數 | 現值 | 作用 / 調法 |
|---|---|---|
| `wheel_diameter_mm` | 48 | **距離刻度**。錯 → 停車倒退、牆重影。用 7.1 校 |
| `wheel_space_mm` | 110(廠設,未量) | **旋轉刻度**(和 axle_space 之和)。錯 → 地圖彎折。用 7.2 校 |
| `axle_space_mm` | 110(廠設,未量) | 同上 |
| `vx_sign`/`vy_sign`/`wz_sign` | 1 / -1 / -1 | 軸向正負(已實機驗證,勿動;動了 odom 和指令一起反,地圖直接鏡像) |
| `cmd_vel_timeout` | 0.5s | watchdog:斷線自動停車 |

### slam_toolbox(`mapper_params_online_async.yaml`)

| 參數 | 現值 | 作用 / 什麼時候動它 |
|---|---|---|
| `resolution` | 0.05 | 地圖格 5cm。迷宮牆薄想更細可到 0.03(Pi CPU 變重),**odom 沒校準前調細只會更糊** |
| `minimum_travel_distance` | 0.2 | 每走 20cm 收一個節點。調小 → 修正更頻繁、模型跳動更細碎但地圖更緊;小迷宮可 0.1 |
| `minimum_travel_heading` | 0.17 (≈10°) | 每轉 10° 收一個節點。旋轉是誤差大戶,可調小到 0.1 |
| `correlation_search_space_dimension` | 0.5 | scan matching 搜索窗(m)。**odom 校準後不用動**;odom 爛時的止痛藥(加大),副作用是慢+迷宮易配錯 |
| `max_laser_range` | 12.0 | C1 標稱極限。迷宮內牆近,不是瓶頸 |
| `map_update_interval` | 2.0 | /map 重渲染週期(秒)。純顯示頻率,不影響精度 |
| `do_loop_closing` | true | 迷宮必開 |
| `loop_search_maximum_distance` | 3.0 | 迴環搜索半徑。迷宮小,夠用 |
| `loop_match_minimum_chain_size` | 10 | 防假迴環門檻。迷宮走廊長很像,若出現「地圖被亂拉」考慮調大到 12–15 |
| `minimum_time_interval` | 0.2 | 節點最小時間間隔,配 10Hz 光達合理 |

### RViz(`view_robot.rviz`,已改)

| 設定 | 舊 → 新 | 理由 |
|---|---|---|
| Odometry → Keep | 50 → 1 | 不留箭頭軌跡(除錯時可調回 50 看 odom 漂移量) |
| Odometry → Shaft/Head Length | 0.3/0.1 → 0.08/0.03 | 箭頭原本比車長 2.6 倍 |

---

## 九、建圖操作守則(TL;DR)

1. 改完參數**重啟 bringup** 才生效(參數是開機時寫進韌體的)。
2. 先校直線(7.1)再校旋轉(7.2),都過了才開始建正式地圖。
3. 建圖時**慢**:直線 ≤ 0.15 m/s、旋轉 ≤ 0.3 rad/s,轉彎前停一拍,少橫移。
4. 路線刻意繞回走過的地方(餵 loop closure)。
5. Fixed Frame 用 `map` 建圖;模型「跳一下」= scan matching 在修正,是好事,
   跳太大 = odom 還不準,回去校。
6. 存圖:`./save_map.sh <名字>`。
