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

`/map` 是一張格子圖,每格 `resolution`(本專案 0.025m = 2.5cm,2026-09-12 前是 0.05)見方,值有三種:
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
- SLAM 參數:`car_assemble_description/config/mapper_params_online_async.yaml`
  (2026-07-21 從 `my_robot_lidar` 搬進本 repo,Pi 與 PC 兩邊讀同一份)
- 光達驅動與外參:`car_assemble_description/launch/robot_bringup.launch.py` 的 3a / 3b 段
  (2026-07-25 從 `my_robot_lidar/launch/lidar_start.launch.py` 搬進本 repo)
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
5. **光達外參填錯(見下面問題 5)** —— 2026-07-25 新發現,和旋轉量成正比,優先序其實排在第 2 之後。
6. 以上都修完地圖還糊,才動 slam_toolbox 參數(第八節)。

### 問題 5:光達外參 `base_link -> laser_frame` 是全 0 的佔位值(2026-07-25)

原本這個 TF 由 repo 外的 `~/ros2_ws/src/my_robot_lidar/launch/lidar_start.launch.py` 發布,
內容是六個 0,註解自己寫著「假設雷達安裝在機器人中心上方」—— **那只是假設,而且是錯的**:

- URDF 的 `lidar_joint` 說光達在 base_link 的 `(0.0088, -0.061, 0.0427)`;
- 四顆輪子 joint 原點算出的**運動學中心**(板子回報車體速度、odom 實際追蹤的那個點)
  在 base_link 的 `(-0.0047, -0.047)` —— 也就是 **base_link 本身就不是車子的旋轉中心**
  (SolidWorks 匯出的原點是任意的);
- 兩者相減 → 光達相對旋轉中心約 `(+0.014, -0.014)`,不是 `(0, 0)`。

**後果**:原地旋轉時光達實際在繞一個小圓走,SLAM 卻以為它釘在旋轉中心不動 →
每轉一次,同一面牆被畫到兩個位置 → 雙線牆、走廊彎折、扇形塗抹。
這是**與旋轉量成正比**的誤差:直走時幾乎看不出來,一轉彎就爆,和「/map 對不上 /scan」的症狀吻合。

**目視快篩**:RViz 裡 `/scan` 的發射原點畫在車體正中央,但 URDF 的 `lidar_link` 模型偏在一邊 ——
兩者對不起來就是這個 bug。

**修法**:2026-07-25 已把光達 launch 收進本 repo(`robot_bringup.launch.py` 直接起
`sllidar_node` + 自己的 static TF),外參改成四個 launch arg:
`laser_x` / `laser_y` / `laser_z` / `laser_yaw`(預設 `0.014` / `-0.014` / `0.109` / `0.0`)。
**預設值只是上面 CAD 推算的起點,不是量測值**,務必用 §7.3 校準後把結果寫回 launch 的
`default_value`。launch 檔是 symlink-install,改值不必 rebuild:

```bash
./run_robot.sh laser_x:=0.02 laser_y:=-0.01
```

---

### 問題 6:陀螺儀零點漂移 —— 「前面好好的、後面才歪」的直接成因(2026-08-30)⭐

**症狀**:同一間房間、同樣走法,建圖**有時候**好、有時候後半段整個轉掉一個角度。
前段牆壁乾淨筆直,愈到後面愈歪,而且**每次歪的量不一樣**。

**成因**:`use_gyro_heading=true` 時 odom 的朝向是把板子的原始 gyro-Z 積分出來的,
但 MEMS 陀螺靜止時**不會讀到 0**,永遠有一個零點偏移(bias),而且這個偏移
**每次開機都不同、還會隨晶片溫度變**。舊版 driver 直接積分原始讀值,等於把這個
偏移當成真的轉動一路累加。

**實測(2026-08-30,車完全靜止、馬達停住)**:

| 這次開機 | 靜止時 raw `gyro_z` 平均 | 積出來的假轉動 |
| --- | --- | --- |
| 第 1 次 | −0.000447 rad/s | **−1.6 °/分** → 5 分鐘建圖 −7.8° |
| 第 2 次 | −0.000350 rad/s | −1.2 °/分 |
| 第 3 次 | −0.000225 rad/s | −0.8 °/分 |

三次數值差快兩倍 —— 這就是「有機率會歪」的來源。

**為什麼它專門毀後半段**:這個誤差**和「經過多久」成正比,和「走多遠」無關**。
車停在原地讓你思考、調整、等 RViz 的那幾分鐘,odom 的朝向都在偷偷轉。
開場前 30 秒幾乎看不出來,第 8 分鐘就是好幾度。而 slam_toolbox 的
`minimum_travel_heading: 0.17`(≈9.7°)是拿 odom 算的,所以站著不動久了
甚至會被誤判成「車轉了」而插入一張新 scan。

**修法(已進 code)**:`driver_node.py` 新增 `gyro_auto_bias`(預設 `true`)——
啟動時趁車靜止量 40 筆求平均當零點,之後只要車是停的就用 30 秒時間常數的
指數平均慢慢跟著溫度修。朝向積分會**等零點量完才開始**,不然等於把偏移永久烙進去。

**驗收(修完實測)**:靜止 121 秒,odom 朝向漂移從 −2.5° 降到 **±0.15° 以內**,
而且不再是單向的斜坡,是有界的隨機遊走。

要自己重量一次:

```bash
# 車放著別動,起 driver,盯 60 秒
ros2 run ominibot_driver ominibot_driver_node
ros2 topic echo /odom --field pose.pose.orientation   # yaw 應該幾乎不動
```
啟動時 driver 會直接印出量到的零點,例如
`gyro zero offset = -0.000350 rad/s (-1.2 deg/min of heading drift removed; 40 samples)`。
想比對舊行為就 `./run_robot.sh gyro_auto_bias:=false`。

---

### 問題 7:序列埠 desync 造成 /odom 凍結數秒(2026-08-30)⭐

**症狀**:建圖時偶爾一段路的牆壁整片錯位、或轉彎處出現硬折角,但 CPU、WiFi 都正常。

**成因**:板子的回授串流會在 driver **寫指令**時掉幾張 frame(實測 cmd_rate 20 Hz 下
約 5%,`bcc` 全 0 —— 是分幀跑掉,不是位元被干擾)。舊的 `read_feedback()` 遇到
非 `0x7b` 的位元組只丟掉**一個 byte** 就 return,回到 Python 迴圈再讀下一個。
純腳本裡這樣還追得上,但在 ROS 節點裡這個迴圈要跟 executor 搶 GIL,
追不上 640 B/s 的串流 → kernel buffer 愈積愈多 → 變成**正回饋的雪崩**。

實測到最糟的一次:**5 秒內 455 次 desync、只有 4 張好 frame** —— 等於
`/odom` 和 `odom->base_link` TF 凍結了五秒。車還在走,SLAM 卻拿到一個不動的 odom prior,
折角就是這樣來的。

**修法(已進 code)**:`ominibot_hv.py` 的 `read_feedback()` 改成用
`ser.read_until(b'\x7b')` **一次跳到下一個起始位元組**。`stats['desync']` 現在
是「事件數」不是「壞掉的 byte 數」,另外新增 `stats['resync_bytes']` 記丟掉幾個 byte。

**驗收**:同樣條件下 desync 從 ~40 次/5 秒降到 **~4 次/5 秒**,好 frame 維持 94/5 秒
(≈19 Hz),121 秒內**不再出現任何 >3 秒的 /odom 空窗**。

**殘留**:ROS 節點下仍會偶發一段 5~10 秒的 frame rate 減半(20 Hz → 9 Hz),
時間點隨機。→ **2026-09-11 找到成因了,見問題 8:指令 timer 和板子回饋的拍頻。**

### 問題 8:指令寫入撞到板子回饋 TX → 拍頻式 desync → odom 把那段位移整段丟掉(2026-09-11)⭐

**症狀**:bringup 才兩分鐘、車一開始走,RViz 裡的 scan 就離牆 ~20 cm、有一整段掃描點
落在地圖外,「一開始就漂」。車停著量卻什麼都正常:`/odom` 20 Hz、`/scan` 10 Hz、
`map->odom` 50 Hz 零斷流,靜止 30 s 朝向只漂 0.007°,Pi/PC 時鐘同步。

**成因鏈**:
1. **拍頻(beat)**:driver 用自己的 20 Hz timer 寫指令,板子用自己的 20 Hz 送回饋。板子 TX 到一半
   收到指令就把那張 frame 截斷(所以 `desync` 一堆、`bcc=0`)。兩個 20 Hz 差一點點,相位慢慢滑過去,
   對齊的那幾秒每張都撞 → **每 90~110 s 爆一次、一次 5~10 s、最糟 90.7% bad**,車停著也一樣。
   證據:driver 自己 fd 讀 `TIOCGICOUNT`,81% bad 的視窗裡 kernel `uart overrun=0` —— byte 不是在
   Pi 裡丟的,是板子根本沒送完。(問題 7 說的「driver 寫指令時掉 ~5% frame」就是同一件事的平均值。)
2. **odom**:5 s 只剩 7 張好 frame → 兩筆之間 dt ≈ 0.7 s,超過 `_publish()` 寫死的 0.5 s 上限
   → **那段位移直接歸零不積分**。車在走、odom 不動 → slam_toolbox 看不到 travel 不插 scan、
   RViz 把「現在的 scan」畫在「舊的 pose」上,就是截圖那個錯位。

**修法(已進 code)**:
- **`cmd_sync_to_feedback`(預設 true)**:指令改成在 read thread 收到一張好 frame 之後立刻寫,
  永遠落在兩張 frame 之間 47 ms 的空檔;原本的 `cmd_rate` timer 降為備援(回饋斷兩個週期才接手,
  watchdog 停車指令照樣送得出去)。**驗收:重啟後 4 分鐘 `/odom` 每 10 s 剛好 200 筆、最大間隔 0.116 s、
  零 `degraded` WARN**(改之前同樣時間內會爆 2~3 次)。
- **`odom_max_gap`(預設 1.0 s)** 取代寫死的 0.5;間隔內用前後兩筆速度**平均**積分(梯形法),
  超過才丟;車在動而回饋斷超過 0.25 s 會 WARN `feedback gap X.XXs while moving`。
- **診斷**:`serial link degraded` WARN 現在附帶 kernel 的 UART 計數(`uart overrun=… buf_overrun=…
  frame=…`,免 sudo)和 `vcgencmd get_throttled` 的旗標。`overrun>0` = byte 在 Pi 裡丟的(電源/負載);
  `overrun=0` 而 desync 一堆 = 板子端。沒 sudo 想看累計值:`sudo cat /proc/tty/driver/ttyAMA` 的 `oe:`。

**順便抓到的另一件事(還沒修,硬體)**:`vcgencmd get_throttled` = `0x50000`,kernel 15 分鐘 38 次
`hwmon: Undervoltage detected!`,每 20~40 s 一次、車停著也一樣 —— Pi 的 5 V 長期掉到 4.63 V 以下,
韌體反覆把 CPU 砍到 600 MHz。這次證明它**不是** desync 的原因,但它會拖慢 Pi、有機會讓 SD 卡壞掉或
直接重開機。要 5.1 V ≥ 3 A 的 buck、線粗且短;光達 C1 從 USB 抽 ~0.5 A,考慮獨立供電。驗收:跑完一趟
`vcgencmd get_throttled` 回 `0x0`。driver 啟動時 since-boot 旗標已亮會 WARN 一次,之後每分鐘最多提醒一次。
建圖時也**不要在 Pi 上開 VS Code remote / Claude Code**(當天量到合計吃掉 ~半顆核心)。

---

## 七、校準程序(修完參數必做)

> **順序不可顛倒:7.1 直線 → 7.2 旋轉 → 7.3 光達外參。**
> 7.3 的疊圖法是用 `odom` 當固定座標系,odom 刻度本身還是錯的時候,疊出來的糊是 odom
> 造成的,不是外參造成的,校了也是白校。
>
> **工具:`tools/odom_check.py`** —— 即時印出累積位移/轉角(度),比 `ros2 topic echo /odom`
> 的四元數好讀太多,結束時還直接幫你算出該填的倍率。先把倍率歸 1 再量:
> ```bash
> ./run_robot.sh odom_linear_scale:=1.0 gyro_scale:=1.0   # Pi
> python3 tools/odom_check.py                             # 另一個終端
> ```
>
> **2026-07-25 實測結果(已寫回預設值)**:
> - 靜止 30 秒累積轉角只飄 +0.1° → **陀螺儀零偏乾淨**,航向問題不是 drift 是刻度。
> - 直線 1 m:原始輸出報 6.54 m → **`odom_linear_scale = 0.153`**(原本 0.16,只差 4%)。
> - 旋轉 360°:原始輸出報 335° → **`gyro_scale = 1.075`**(原本 1.0,只差 7%)。
> - 也就是說 **odom 刻度一直都大致正確**,「地圖不長」不是 odom 低報造成的,
>   別再往那個方向查(2026-07-25 曾誤判過一次)。
> - `/odom` 實測 19 Hz(不是舊筆記寫的 30 Hz)。

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

### 7.3 光達外參校準

> **2026-07-25 已用 `tools/analyze_bag.py` 數值校出 `laser_yaw = +167.0°`(2.9146 rad)——
> 光達幾乎是反裝的。** 先前填 0.0 是地圖扇形塗抹的元凶:odom 說車往前,光達看到的世界
> 卻幾乎反向流動,scan matching 每一步都在對抗一個近乎顛倒的運動模型。
> 同一份 bag 也修正了 `gyro_scale` 1.075 → 1.014(odom 報 145.3° vs scan 真值 137.1°),
> 並確認 **`gyro_z_sign` 是對的**(第一版分析工具符號寫反,一度誤判成要翻轉)。
>
> **數值法(推薦,取代下面的目視疊圖法):**
> ```bash
> python3 tools/record_diag.py ~/bags/diagN   # Pi;靜止→原地轉360°→靜止→直行1m→靜止
> python3 tools/analyze_bag.py ~/bags/diagN   # 直接印出該填的 gyro_scale 與 laser_yaw
> ```
> 原理是拿 `/scan` 當真值反驗 `/odom`:旋轉段用連續兩幀的環形互相關求真實轉角;
> 直線段擬合 `Δr(θ) ≈ -d·cos(θ-φ)` 求車在光達座標系的行進方向,與 odom 的方向相減即為 yaw。
> 改動這些數學後務必跑 `python3 tools/selftest_analyze.py`(合成資料釘死符號約定)。
>
> **為什麼目視法會漏掉這個 bug**:如果你是「朝牆直行」來測,180° 的誤差會**保留**
> 「牆垂直於行進方向」這個關係 —— 看起來完全正常。只有平行走牆、或用數值法才抓得到。

### 7.3-舊 光達外參校準(疊圖法,不用尺;保留作為無工具時的備案)

校 `laser_x` / `laser_y` / `laser_yaw`(問題 5)。核心觀念:**把 SLAM 完全排除在外**,
只驗「odom + 光達 TF」這條鏈,所以 Fixed Frame 用 `odom` 而**不是** `map`。

**RViz 設定(PC 端)**
- Fixed Frame → `odom`
- LaserScan → **Decay Time = 20**(讓 20 秒內的 scan 疊在一起顯示)、Size 調到 0.01–0.02
- 開車速度照第九節:直線 ≤ 0.15 m/s、旋轉 ≤ 0.3 rad/s

兩個測試**互相獨立**(純平移不會因 yaw 錯而糊、純旋轉不會因 yaw 錯而糊),所以可以一個一個校。

**7.3a 校 `laser_yaw` —— 直線測試**

沿著一面直牆平行慢慢前進約 1 m。

- yaw 對 → 疊出來的牆是一條直線,且**與車子在 odom 裡的行進方向平行**。
- yaw 差 θ → 牆仍是一條乾淨直線,但相對行進方向**傾斜 θ**(每張 scan 都繞光達原點轉了同樣角度,
  所以不糊,只是整體歪掉)。量出這個傾角加到 `laser_yaw`,重跑再確認。

**7.3b 校 `laser_x` / `laser_y` —— 原地旋轉測試**

在牆角(兩面互相垂直的牆)前**原地**慢轉 360°。

- 偏移對 → 疊出來的牆收斂成單一清晰的線。
- 偏移差 e → 牆糊成寬約 **2e** 的帶狀(光達繞半徑 e 的圓走,SLAM 以為它不動)。
- 用座標下降法:`laser_x` 各試 ±0.01,留比較清晰的那個;再對 `laser_y` 做同樣的事。
  通常 2–3 輪就收斂。每輪只要重跑 `./run_robot.sh laser_x:=... laser_y:=...`,不用 rebuild。

**順便驗到的東西**:若 360° 轉完牆線**沒有閉合**(起點與終點的同一面牆錯開一個角度),
那是 `gyro_scale` 沒校準 —— 回去做 §7.2,不是外參的問題。

校完把收斂值寫回 `car_assemble_description/launch/robot_bringup.launch.py` 裡
`laser_x`/`laser_y`/`laser_yaw` 的 `default_value`,並註明是實驗校準值與日期。

### 7.4 驗收

三項都過後,再跑一次 SLAM 繞房間一圈:
- 停車時模型不再倒退(或只剩 1–2cm 的微跳);
- 同一面牆只有一條線,90° 牆角是直角;
- 回到出發點時 loop closure 不需要大幅拉扯地圖。

---

## 八、關鍵參數速查表

### slam_toolbox 的四種 mode(2026-07-25 補)

常被問「是不是該換 mode」。本專案用的是 **online asynchronous**,那已經是即時建圖的標準選擇,
**換 mode 不會修好破圖**。四種對照:

| 模式 | 節點 / 設定 | 什麼時候用 |
|---|---|---|
| **online async** | `async_slam_toolbox_node` | ✅ **本專案現用**。跟不上時丟舊 scan 保即時性 |
| online sync | `sync_slam_toolbox_node` | 不丟 scan、照順序全處理,但會落後真實時間。優勢只在「CPU 算不動」;本專案的 scan 遺失發生在 tf2 MessageFilter(WiFi jitter 等不到 TF,見 `0721_net_issue_plan.md`),sync 一樣救不了 |
| **offline** | `offline_slam_toolbox_node` / `ros2 bag play` | 🔧 **除錯神器**,見下面 |
| localization | async + `mode: localization` | 比賽當天:載入 `maps/` 存好的地圖,只定位不建圖 |

**rosbag 離線除錯迴圈**(強烈建議,調參效率差非常多):

```bash
# Pi 端:錄一趟(走一圈迷宮)
ros2 bag record /scan /odom /tf /tf_static -o ~/bags/maze_run1
# PC 端:拉回來反覆重播,不必再開一次車
ros2 bag play ~/bags/maze_run1 --clock
```

好處是**把 WiFi jitter 完全移出等式**(重播是本機讀檔,不會再出現 "queue is full"),
同一份資料可以反覆試不同參數做 A/B。要拿來掃光達外參(§7.3)時,錄的時候**排除 `/tf_static`**,
重播時自己發不同的 static TF,一份 bag 就能試完所有偏移組合。

### ominibot_driver(launch 參數,`./run_robot.sh xxx:=yyy` 直接帶)

| 參數 | 現值 | 作用 / 調法 |
|---|---|---|
| `wheel_diameter_mm` | 48 | **距離刻度**。錯 → 停車倒退、牆重影。用 7.1 校 |
| `wheel_space_mm` | 115(實車量測) | **旋轉刻度**(和 axle_space 之和)。錯 → 地圖彎折。用 7.2 校 |
| `axle_space_mm` | 96(實車量測) | 同上 |
| `laser_x` / `laser_y` | 0.014 / -0.014(**CAD 推算,未校**) | **光達外參**平移。錯 → 一轉彎牆就變雙線(問題 5)。影響量級僅 1.4cm |
| `laser_yaw` | **2.9146(=167°,實測)** | 光達幾乎反裝。填 0 會讓地圖扇形塗抹 —— 這是 2026-07-25 破圖的元凶 |
| `vx_sign`/`vy_sign`/`wz_sign` | 1 / -1 / -1 | 軸向正負(已實機驗證,勿動;動了 odom 和指令一起反,地圖直接鏡像) |
| `cmd_vel_timeout` | 1.0s | watchdog:斷線自動停車 |
| `odom_max_gap` | 1.0s | 回饋兩筆間隔多久以內仍積分(前後速度平均);超過就丟並 WARN(問題 8) |
| `cmd_sync_to_feedback` | true | 指令緊跟在每張回饋 frame 後面寫,消除拍頻 desync(問題 8)。false=舊的獨立 timer |

### slam_toolbox(`mapper_params_online_async.yaml`)

| 參數 | 現值 | 作用 / 什麼時候動它 |
|---|---|---|
| `resolution` | **0.025**(2026-09-12,原 0.05) | 地圖格 2.5cm。44 cm 迷宮格配 5 cm 格牆像積木、看不出 scan 貼不貼牆;SLAM 在 PC 跑所以 CPU 不是問題。**odom 沒校準前調細只會更糊** |
| `minimum_travel_distance` | 0.2 | 每走 20cm 收一個節點。調小 → 修正更頻繁、模型跳動更細碎但地圖更緊;小迷宮可 0.1 |
| `minimum_travel_heading` | 0.17 (≈10°) | 每轉 10° 收一個節點。旋轉是誤差大戶,可調小到 0.1 |
| `correlation_search_space_dimension` | 0.5 | scan matching 搜索窗(m)。**odom 校準後不用動**;odom 爛時的止痛藥(加大),副作用是慢+迷宮易配錯 |
| `max_laser_range` | 12.0 | C1 標稱極限。迷宮內牆近,不是瓶頸 |
| `map_update_interval` | 1.0(2026-09-12,原 2.0) | /map 重渲染週期(秒)。純顯示頻率,不影響精度 |
| `do_loop_closing` | true | 迷宮必開 |
| `loop_search_maximum_distance` | 3.0 | 迴環搜索半徑。迷宮小,夠用 |
| `loop_match_minimum_chain_size` | 10 | 防假迴環門檻。迷宮走廊長很像,若出現「地圖被亂拉」考慮調大到 12–15 |
| `minimum_time_interval` | 0.2 | 節點最小時間間隔,配 10Hz 光達合理 |

### RViz(`view_robot.rviz`,已改)

| 設定 | 舊 → 新 | 理由 |
|---|---|---|
| Odometry → Keep | 50 → 1 | 不留箭頭軌跡(除錯時可調回 50 看 odom 漂移量) |
| Odometry → Shaft/Head Length | 0.3/0.1 → 0.08/0.03 | 箭頭原本比車長 2.6 倍 |
| TF → Show Names / Show Arrows / Marker Scale | true/true/1 → false/false/0.3 | 六個 frame 的名字和 1 m 的軸把 15 cm 的車整個蓋住(2026-09-12) |
| LaserScan → Size | 0.01 → 0.02 | 配合 0.025 m 地圖,點太小看不出貼不貼牆 |

---

## 九、建圖操作守則(TL;DR)

1. 改完參數**重啟 bringup** 才生效(參數是開機時寫進韌體的)。
2. 校準順序:直線(7.1)→ 旋轉(7.2)→ 光達外參(7.3),三項都過了才開始建正式地圖。
3. 建圖時**慢**:直線 ≤ 0.15 m/s、旋轉 ≤ 0.3 rad/s,轉彎前停一拍,少橫移。
4. 路線刻意繞回走過的地方(餵 loop closure)。
5. Fixed Frame 用 `map` 建圖;模型「跳一下」= scan matching 在修正,是好事,
   跳太大 = odom 還不準,回去校。
6. 存圖:`./save_map.sh <名字>`。
