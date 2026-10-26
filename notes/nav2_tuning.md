# Nav2 調參手冊（2026 TIRT 迷宮車）

寫給「已經跑得動、要開始調」的階段。所有參數都在 **`config/nav2_params.yaml`**（repo 根目錄的
`config/`，2026-07-28 起集中在那裡），改完**不用重 build**——`colcon --symlink-install` 裝的是 symlink，
但**要重啟 nav2**（`Ctrl-C` 掉 `run_nav2.sh` 再跑一次），因為這些值是節點 configure 時讀進去的。

先讀 `CLAUDE.md` 的「Autonomous navigation (Nav2)」一節了解架構，再看這裡。

---

## 0. 一分鐘版：我想改 X，去動哪裡

| 想做的事 | 改哪裡 | 注意 |
| --- | --- | --- |
| 跑快一點 | `FollowPath.max_vel_x` + **`velocity_smoother.max_velocity`** + `behavior_server.max_rotational_vel` | 三個要一起改，見 §1 |
| 低速卡住／打滑 | `FollowPath.min_speed_xy` + `min_speed_theta`（成對） | 單獨設 min_speed_xy 無效，見 §2 |
| 起步/煞車在打滑 | `acc_lim_*` / `decel_lim_*`（降低） | 不要先降最高速，見 §2 |
| planner 說找不到路 | `robot_radius`、`inflation_radius`（兩個 costmap 都要） | 幾乎都是這個，不是 planner，見 §3 |
| 貼牆太近／撞牆 | `inflation_radius`（提高）、`cost_scaling_factor` | 見 §3 |
| 不肯橫移、老是轉頭 | `PathAlign.scale` / `GoalAlign.scale`（降低） | 見 §4 |
| 停不準／終點附近爬行 | `xy_goal_tolerance`、`RotateToGoal.slowing_factor` | 見 §4 |
| 換 planner / controller 演算法 | `planner_server.GridBased.plugin` / `FollowPath.plugin` | 見 §5、§6 |
| 定位飄掉 | `amcl.alpha1..5`、`update_min_d/a`、`laser_*` 外參 | 見 §7 |

---

## 1. 速度：★ 三個地方要一起改

這是最容易踩的坑。**只改 `FollowPath` 的 `max_vel_x` 完全不會變快，而且不會有任何警告。**

指令的路徑是：

```
planner → controller_server(FollowPath/DWB) → /cmd_vel_nav → velocity_smoother → /cmd_vel → 車
                          ↑ 第 1 個上限                        ↑ 第 2 個上限（會靜靜夾掉）
```

| # | 位置 | 參數 | 目前值 |
| --- | --- | --- | --- |
| 1 | `controller_server.FollowPath` | `max_vel_x` / `max_vel_y` / `max_vel_theta` / `max_speed_xy` | 0.35 / 0.25 / 1.5 / 0.40 |
| 2 | `velocity_smoother` | `max_velocity: [x, y, theta]`、`min_velocity`、`max_accel`、`max_decel` | [0.35, 0.25, 1.5] |
| 3 | `behavior_server` | `max_rotational_vel`（只有 recovery 自轉會用到） | 1.5 |

再快的話，往上調之前先確認兩件事：

- **車子本身的極速**：teleop 的 `linear_speed` 預設 0.6 m/s、`linear_max` 1.5，所以硬體遠不止 0.35。
  用 `ros2 run ominibot_driver mecanum_teleop` 在 Pi 上按 `w` 加速，找出「還能直線走、不打滑」的速度，
  nav2 的上限設在那之下。
- **`sim_time` 要跟著**：DWB 每條候選軌跡往前模擬 `sim_time` 秒（現在 1.5s）。速度提高但 sim_time 不變，
  等於「看得更遠」——1.5s × 0.35 ≈ 0.5m 對走道剛好，到 0.6 m/s 就變成 0.9m，會開始「看過轉角」而選出
  撞牆的軌跡。速度加倍就把 `sim_time` 砍半（0.8–1.0）。

**驗證真的變快了**（不要用眼睛判斷）：

```bash
ros2 topic echo /cmd_vel --once          # smoother 之後、真正送到車上的
ros2 topic echo /cmd_vel_nav --once      # DWB 的原始輸出
```
兩者的 `linear.x` 不一樣，就是被 smoother 夾掉了 → 你漏改第 2 個地方。

---

## 2. 打滑 / 低速走不動

兩件不同的事，別混在一起調。

### 2a. 低速走不動（→ 用速度下限）

`min_speed_xy: 0.06` + `min_speed_theta: 0.15`。**必須成對設定**：DWB 的判斷式是
`(平移慢於 min_speed_xy) AND (自轉慢於 min_speed_theta) → 丟掉這條軌跡`，
`min_speed_theta` 留 0.0 的話「自轉慢於 0.0」永遠不成立，整條判斷失效，`min_speed_xy` 寫多少都沒用。

副作用：窄走道裡如果所有候選軌跡都被刷掉，DWB 會回報找不到可行軌跡 → controller 失敗 → recovery。
真的發生就把兩個一起降（0.04 / 0.10）或都設 0.0 退回原本行為。

還是不動的話，問題在**驅動層而不是 nav2**，可調的是 `ominibot_driver` 的參數（透過
`./robotctl args "..."` 傳給 `robot_bringup.launch.py`，然後 `./robotctl restart bringup`）：

- `motor_pwm_min`（預設 2100）— 板子的 PWM 下限。馬達有靜摩擦死區，低指令時扭力不夠就是這裡。
- `vel_kp` / `vel_ki`（預設 3000 / 1050）— CircusPi 原廠值是給更重的參考車，對這台輕的 N20 太硬。
  2026-07-25 已記錄「調低有幫助」（見 memory / `notes/SLAM_learning_note.md`）。

### 2b. 高速打滑（→ 降加速度，不是降極速）

麥克納姆的滾子一打滑里程計就毀了（這台的航向已經因此改用 gyro 積分，見
`notes/SLAM_learning_note.md` §7）。打滑幾乎都是**加速度**造成的，不是最高速：

- 先降 `acc_lim_x` / `acc_lim_y`（現在 1.5）和 `decel_lim_*`（現在 -2.0）。
- `velocity_smoother` 的 `max_accel` / `max_decel` 要跟著降，否則 smoother 允許的斜率比 DWB 規劃的還陡。
- 轉彎打滑就降 `acc_lim_theta` / `rotational_acc_lim`。

診斷用 `tools/odom_check.py`：讓車直線走 1 m，如果 `/odom` 報的距離明顯偏長，就是打滑（輪子轉了車沒走）。

---

## 3. Costmap 幾何：「planner 說找不到路」幾乎都是這裡

**先看這裡，不要動 `planner_server`。** 兩個 costmap（`local_costmap` / `global_costmap`）各有一份，
改的時候**兩邊都要改**。

| 參數 | 目前 | 意義 |
| --- | --- | --- |
| `robot_radius` | **0.09** | 車體圓形半徑。障礙物這個距離內 = 內切致命區（代價 253），planner 直接當成牆 |
| `inflation_radius` | 0.18 | 代價膨脹範圍。比 `robot_radius` 大的那一段是「可以進但不鼓勵」的緩衝 |
| `cost_scaling_factor` | 5.0 | 代價衰減陡度。越大 → 離牆一點點就變便宜 → 越敢貼牆走 |

走道寬 W 時，車心可走的自由帶 ≈ `W - 2 × robot_radius`。

### 不要猜，量它：`python3 tools/costmap_check.py`

**2026-07-28 這個參數害整套走不了一次，值得記住。** 原本設 `robot_radius: 0.11`（幾何上正確：車體
0.16×0.14 m，對角半徑 0.108），結果 goal 一送出去就 aborted、車完全不動、而 nav2 閒置時**不噴任何警告**。
在 `201_self_test` 上量出來的連通性斷崖極陡：

| `robot_radius` | 最大連通區 |
| --- | --- |
| 0.12 | 3.74 m² |
| **0.11** | **4.07 m²** ← 原設定：地圖被切成互不相通的小塊 |
| 0.10 | 39.10 m² ← 整張圖連通 |
| 0.09 | 39.10 m² |

差 1 cm，可走面積差 10 倍。所以現在設 0.09。**代價**：0.09 小於真實對角半徑 0.108，車角可能擦牆約
1.8 cm —— 但 0.11 的替代方案不是「比較安全」而是「完全不能導航」。真的常擦牆，正解是改用矩形
`footprint`（見 `config/nav2_params.yaml` 該處註解），不是把半徑調回去。

**找不到路的調整順序**：先 `python3 tools/costmap_check.py` 看斷崖在哪，再降 `robot_radius`；
`inflation_radius` 只是梯度，不會造成「不通」，不必先動它。

### 讀 costmap 數值的陷阱

`/global_costmap/costmap` 這個 topic 上的 `OccupancyGrid` 是**重新縮放成 0..100** 的版本，不是內部的
raw 0..255 代價。對照表：

| 發布值 | raw | 意義 |
| --- | --- | --- |
| 100 | 254 | `LETHAL_OBSTACLE` |
| **99** | **253** | `INSCRIBED_INFLATED_OBSTACLE` — **planner 視為不可走** |
| 1..98 | 1..252 | inflation 梯度，可走但不鼓勵 |
| 0 | 0 | 完全空 |
| -1 | 255 | `NO_INFORMATION`；`allow_unknown: false` 時也不可走 |

拿 253/254 去比對這張圖永遠不會命中，會得到「整張地圖都是空的」的錯誤結論（我第一次就是這樣誤判的）。

另外兩個會造成「找不到路」的：

- `planner_server.GridBased.allow_unknown: false` — 刻意的。地圖上未知的格子不准穿。目標點落在沒掃到的
  區域就是規劃不出來，**這是設計而不是 bug**；要走那裡就先把地圖建完整。
- `global_costmap.track_unknown_space: true` — 同上，未知≠可走。

## 3b. 速度提高之後要跟著看的 costmap 參數

- `local_costmap.update_frequency`（現在 10.0）— 感測到障礙物的反應速度。跑快了要更高（15–20），
  但這是 PC 的 CPU 成本。
- `local_costmap.width` / `height`（現在 3）— **必須是整數公尺**，寫 2.5 會讓 `controller_server` 建構
  時丟型別錯誤、整個 `nav2_container` 載不起來（實機踩過）。跑快時要比 `sim_time × max_vel` 大，
  否則軌跡會超出局部地圖。

---

## 4. DWB 的評分項（critics）：行為長相在這裡調

`critics` 是一組評分函數，DWB 對每條候選軌跡加總分數後挑最好的。改權重就是改「行為偏好」。

| critic | scale | 作用 / 什麼時候調 |
| --- | --- | --- |
| `PathDist` | 32.0 | 車**位置**貼齊全域路徑。迷宮裡要高，不然會自己抄捷徑 |
| `GoalDist` | 24.0 | 往終點靠近 |
| `PathAlign` | 12.0 | 車**朝向**對齊路徑方向。原版 32 → 降到 12 |
| `GoalAlign` | 8.0 | 車朝向對齊終點方向。原版 24 → 降到 8 |
| `RotateToGoal` | 32.0 | 最後原地轉到目標朝向 |
| `BaseObstacle` | 0.02 | 避障（軌跡經過的 costmap 代價） |
| `Oscillation` | 1.0 | 抑制來回振盪 |

**麥克納姆專屬重點**：`PathAlign` / `GoalAlign` 是「轉頭去對齊」的評分。對差速輪這是必要的（不轉頭走不動），
但這台可以直接橫移過去。**還是老愛先轉頭再走 → 再降 `PathAlign.scale`（12 → 6）。**
反之如果它斜著走得很怪、希望車頭朝前，就往回調高。

其他：

- `xy_goal_tolerance`（0.10，在 `general_goal_checker` 和 `FollowPath` 各有一份）、`yaw_goal_tolerance`（0.20）
  ——「算到了」的判定。設得比車身還大就變成「到附近就算到」。
- `RotateToGoal.slowing_factor`（原版 5.0 → 現在 2.0）— 接近終點的減速倍率。太大會在終點前變成長距離爬行，
  正好撞上 §2a 的走不動區間。
- `Oscillation.oscillation_reset_dist`（0.03）— 「移動多遠算脫離振盪」。對 0.15 m 的小車，原版 0.05 偏大。
- `progress_checker.required_movement_radius`（0.10）/ `movement_time_allowance`（15.0）—「多久沒移動算卡住」。
  誤觸發 recovery 就放寬。

---

## 5. 換 controller（局部規劃器）演算法

改 `controller_server.FollowPath.plugin` 一行，加上那個 plugin 自己的參數。都已安裝，不用另外 apt。

| plugin | 適合 | 對這台的評估 |
| --- | --- | --- |
| `dwb_core::DWBLocalPlanner` | 通用、好懂、純 C++ | **目前用的**。開了 y 軸就會橫移，但 critic 結構偏好轉頭 |
| `nav2_mppi_controller::MPPIController` | 全向 / 窄空間 | 理論上**最適合**（`motion_model: "Omni"`）。⚠ arm64 binary 在 Pi 4 的 Cortex-A72 上一載入就 SIGILL（container exit -4），只能在 x86 PC 上用。完整設定已寫在 `nav2_params.yaml` 檔尾註解，解註解即可 |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 循跡、參數少、最穩 | **差速輪模型，不會橫移**。想先排除「controller 太複雜」的可能時很好用（只有 lookahead 距離和速度幾個參數） |
| `nav2_rotation_shim_controller::RotationShimController` | 包在別的 controller 外面 | 起步時先原地轉向路徑方向再交給主 controller。對麥克納姆意義不大（可以直接橫移） |

換的時候記得：每個 controller 的速度限制參數**名字不一樣**（DWB 是 `max_vel_x`，MPPI 是 `vx_max`），
但 `velocity_smoother` 那一關永遠都在，別忘了同步（§1）。

## 6. 換 planner（全域規劃器）演算法

改 `planner_server.GridBased.plugin`。

| plugin | 特性 | 對迷宮的評估 |
| --- | --- | --- |
| `nav2_navfn_planner/NavfnPlanner` | 網格 Dijkstra/A*，最簡單 | **目前用的**。地圖只有 168×104 格，`use_astar: true` 幾乎瞬間完成 |
| `nav2_smac_planner/SmacPlanner2D` | 網格 A*，含路徑平滑 | 路徑比 NavFn 平順（少階梯狀鋸齒），適合想跑快的時候。要調 `cost_travel_multiplier` |
| `nav2_theta_star_planner/ThetaStarPlanner` | any-angle A* | 會走斜線捷徑，路徑最短。迷宮走道窄時斜線容易貼牆 |
| `nav2_smac_planner/SmacPlannerHybrid` | Hybrid-A*，含車輛運動學 | 給 Ackermann/差速用的，**全向車不需要**，只是白花 CPU |

共通參數：`tolerance`（0.15，目標點無法精確到達時允許的偏差）、`allow_unknown`（false，見 §3）。

**迷宮解題演算法（右手法則、DFS、Flood-fill…）不屬於這一層。** 那是「決定下一個目標點是哪裡」，
應該寫成一個發 `NavigateToPose` action 的節點，把 Nav2 當成「幫我走到 (x, y)」的服務用。
入口是 `nav2_simple_commander`（Python，已安裝）：

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
nav = BasicNavigator()
nav.waitUntilNav2Active()
nav.goToPose(pose)          # 然後 while not nav.isTaskComplete(): ...
```

多點連走可以用 `waypoint_follower`（設定已在 `nav2_params.yaml` 裡）。

---

## 7. 定位（AMCL）

| 參數 | 目前 | 什麼時候調 |
| --- | --- | --- |
| `alpha1..alpha5` | 0.3 / 0.3 / 0.3 / 0.3 / 0.2 | odom 雜訊模型。粒子雲太緊、修不動 → 調大；抖動亂飄 → 調小 |
| `update_min_d` / `update_min_a` | **0.05 / 0.05** | 走多遠 / 轉多少才做一次濾波更新。**沒達到門檻時 AMCL 完全不修正**，所以兩次更新之間的誤差會累積到接近門檻值 |
| `max_beams` | 120 | 每次匹配用幾條光束。房間小、牆多所以給得比原版 60 多 |
| `transform_tolerance` | 1.0 | 跨 WiFi 的 TF 容忍，**不要降**（見 `0721_net_issue_plan.md`） |
| `set_initial_pose` / `initial_pose` | true / (0,0,0) | 只有「車放回建圖起跑點」時才正確；放別處要在 RViz 點 2D Pose Estimate |
| `robot_model_type` | `nav2_amcl::OmniMotionModel` | **不要改回 Differential**，那會把橫移當成不可能的運動 |

### 點雲和地圖疊不起來 → `python3 tools/scan_match_check.py`

不要用眼睛判斷歪多少。這支把 `/scan` 用當前 TF 打到 map 座標，算平均離牆距離，再暴搜附近的
`(dx, dy, dyaw)` 找最合的位姿。三種結論它會直接講：**定位收斂錯了**（有明顯更好的位姿 → 地圖沒問題）、
**已經對得夠好**、或 **地圖和現場不符**（連最佳位姿都爛 → 要重建圖或查外參）。

**2026-07-28 實測的坑：不能只看平均離牆距離的絕對值。** 窄迷宮裡大多數光點打在 0.2–0.5 m 的近牆上，
角度誤差在近處只造成一兩格偏移，平均值被近點稀釋 —— 實測 8–10° 的航向誤差平均分數只有 0.039 m
（小於一格 0.05 m），看起來「還不錯」，但同樣的 8° 在 3 m 遠處是 `3·sin8° ≈ 0.42 m`。
**近牆貼合、遠牆整片歪掉**就是這個成因，所以判斷要看「有沒有明顯更好的位姿」而不是絕對分數。

那次量到的是純旋轉誤差（`dx=dy=0`、`dyaw=+8~10°`），而 `update_min_a` 當時正是 0.15 rad ≈ 8.6° ——
誤差量級和更新門檻一樣大並非巧合，因此把 `update_min_d`/`update_min_a` 降到 0.05/0.05。

定位飄掉時，先確認不是**里程計或光達外參**的問題（那是上游，AMCL 再怎麼調都救不了）：
`laser_x` / `laser_y` / `laser_yaw`、`odom_linear_scale`、`gyro_scale` 的校正程序在
`notes/SLAM_learning_note.md` §7，工具是 `tools/odom_check.py` 和 `tools/analyze_bag.py`。
順便記一筆已經排除的嫌疑：靜止時 `odom->base_link` 的航向漂移實測只有 **−0.1°/分**，
所以陀螺零偏不是這次的原因（曾經懷疑過）。

---

## 8. 除錯：先看哪裡

```bash
ros2 topic echo /cmd_vel          # 真正送到車上的速度（smoother 之後）
ros2 topic echo /cmd_vel_nav      # DWB 原始輸出。和上面不同 = 被 smoother 夾掉
ros2 topic echo /plan --once      # 全域路徑（規劃得出來嗎）
ros2 topic echo /local_plan --once
ros2 run tf2_tools view_frames    # TF 樹：map->odom->base_link->laser_frame 都在嗎
python3 tools/cmd_vel_check.py    # 在 Pi 上跑,量 /cmd_vel 到達間隔與 watchdog 逾時次數
```

RViz 裡最有用的三個 display（`rviz/view_nav2.rviz` 都已經開好）：
**Global Costmap**（planner 眼中的世界，看走道有沒有被膨脹封死）、
**Local Costmap**、**Amcl Particle Swarm**（粒子雲散開＝定位沒信心）。

常見症狀對照：

| 症狀 | 先看 |
| --- | --- |
| **goal 一送就 aborted、車完全不動、閒置時毫無警告** | **`python3 tools/costmap_check.py`**。最常見的是車停得離牆太近 → 所在格是 INSCRIBED → 連「起點」都不合法，planner 根本不會開始規劃。其次是 `robot_radius` 太大把走道封死（§3） |
| 完全不動、也沒報錯 | Pi 上的 teleop 還在跑嗎（`./robotctl down teleop`）？它閒著會以 20Hz 發零速度 |
| 車體模型不見 | `/robot_description` 是 latched，RViz 的 durability 要 Transient Local |
| 一頓一頓 | `tools/cmd_vel_check.py` 看 watchdog 逾時次數；不是網路就換 `tools/motor_diag.py` |
| 地圖和光達對不上 | `python3 tools/scan_match_check.py`（§7），這是定位問題，不是導航問題 |
| 「no valid trajectories」 | §2a 的 `min_speed_*` 太高，或 §3 的膨脹把走道封死了 |
| 定位莫名其妙地爛，但完全沒有錯誤訊息 | **另一台機器上還開著 `run_slam.sh`。** DDS 是跨機的：它的即時 `/map` 會和 `map_server` 的存檔地圖同時餵給 AMCL，兩邊還都在發 `map->odom`。`run_nav2.sh` 現在會在啟動前檢查並警告 |
| 啟動後很久都沒有任何 log | 有殘留的 `nav2_container` 孤兒在吃 CPU。它**不理 SIGTERM**，要 `pkill -9 -f "component_container_isolated.*nav2_container"` |

---

## §9 變更紀錄

改參數時請在這裡補一筆:哪天、改了什麼、為什麼、有沒有在硬體上驗證過。
「為什麼」比「改成多少」重要 —— 數字在 YAML 裡看得到,理由只有這裡有。

### 2026-08-15 — 針對三個回報症狀的第一批修正(**尚未在硬體上驗證**)

使用者回報:**(1) 整體速度太慢、(2) 轉彎和接近終點卡頓、(3) 路徑很奇怪不像最短的**。
定位是「自主導航到指定點」,風險取向是「先能穩定完跑再往上推速度」。

改了 `config/nav2_params.yaml` 七個值,全部只動參數,沒動程式:

| 參數 | 原值 | 新值 | 針對 |
| --- | --- | --- | --- |
| `critics`(移除 `RotateToGoal`) | 含 RotateToGoal | 不含 | 終點卡頓 |
| `general_goal_checker.yaw_goal_tolerance` | 0.20 | **3.15** | 終點卡頓 |
| `FollowPath.PathAlign.scale` | 12.0 | **6.0** | 轉彎卡頓 |
| `FollowPath.GoalAlign.scale` | 8.0 | **4.0** | 轉彎卡頓 |
| `GridBased.use_astar` | true | **false** | 路徑品質 |
| `FollowPath.BaseObstacle.scale` | 0.02 | **0.5** | 擦牆風險 |
| `planner_server.expected_planner_frequency` | 20.0 | **1.0** | 假警報 |

三件當時查出來、值得記住的事:

1. **`RotateToGoal` 不是評分項,是一道閘門。** 車心一進 `xy_goal_tolerance`,它的
   `prepare()` 就把所有還在平移的軌跡判為 illegal,強迫先完全停下、再原地轉到
   `yaw_goal_tolerance` 之內。配上被衰減約 8 倍的角速度指令,那段幾乎轉不完 ——
   這就是「快到終點在磨」的完整解釋。**注意 `PathAlign`/`GoalAlign` 必須留著**:
   三個轉向評分項如果一起拿掉,DWB 就沒有任何一項懲罰旋轉,轉與不轉分數相同,
   結果是隨機亂轉,比原本更糟。

2. **`expected_planner_frequency` 不是重規劃頻率。** 在 Humble 的 `planner_server` 裡它
   只是 `max_planner_duration_`,也就是印
   `Planner loop missed its desired rate` 的門檻。真正的重規劃節奏由行為樹決定:
   `navigate_to_pose_w_replanning_and_recovery.xml:11` 的 `<RateController hz="1.0">`。
   所以 20 → 1 **不會變快**,是讓真正的超時不被 19 次/秒的假警報淹掉。

3. **矩形 `footprint` 不要換 —— YAML 第 119-126 行那段建議的安全方向是反的。**
   `nav2_costmap_2d` 取短邊當 inscribed 半徑(0.072),比現在的圓形 0.09 **更小**,
   等於讓 2D planner **更敢**規劃過窄縫;而 NavFn/SmacPlanner2D 都不考慮車體朝向。
   同時 DWB 的 `BaseObstacleCritic` 只取軌跡中心格的代價,**完全不看 footprint** ——
   要看得換成貴很多的 `ObstacleFootprintCritic`。角落外伸 1.8cm 是**轉彎時**才發生的
   (對角 0.216m),所以「少轉彎」本身就是那個問題的解,不是換 footprint。

**尚未處理、下次要接的(依重要性):**

- **速度太慢的真正原因沒有修掉。** `cmd_linear_scale`/`cmd_angular_scale` 仍是 1.0,
  命令路徑被衰減約 8 倍(`driver_node.py:130-142`:命令 0.15 m/s → 實際 0.019)。
  現在 `max_vel_x: 0.35` 只是名目值,實際約 0.04。要跑 `tools/vel_sweep.py` 量。
  ⚠️ 校正後車子會比看過的快 8 倍,**要先把 `max_vel_*` 和 `teleop_node.py` 的
  `linear_speed`(目前 0.6,是當初手工補償這個衰減加上去的)一起壓低**。
  天花板:板子回授在 raw 2.77 飽和 ≈ **0.42 m/s**,超過 `/odom` 會無聲少報。
- **`min_speed_xy: 0.06` 的意義會在校正當下改變。** 現在它是「命令空間」的門檻,
  等於真實 0.0075 m/s;校正後變成真實 0.06 m/s,嚴格 8 倍,在窄走道容易觸發
  `no valid trajectories`。必須和 `vel_sweep` 量到的死區一起重新推導,不能放著不管。
- **`robot_radius: 0.09` 是為 201 那間雜亂測試房調的,不是為比賽迷宮。** 比賽走道
  內徑 44cm,自由帶 = 0.44 − 2R,即使 R=0.12 也還有 20cm。要拿**真正的比賽地圖**
  重跑 `tools/costmap_check.py` 再決定,不要把測試房的權宜值帶進賽場。

### 2026-10-09 — 「到定點一直抖動」查到根因;把標定做成一次性可套用的工具

使用者回報:用 `./gcs.sh --nav` 跑,**走起來很卡,到定點會一直抖動,不實用**。
因為是 `gcs.sh --nav`(`gcs.sh:156` 會自動 `robotctl down teleop`),所以
「teleop 閒著發零速度搶 `/cmd_vel`」這條已排除 —— `/cmd_vel` 只有 nav2 一個發布者。

**抖動的根因:DWB 沒有「停住」這個選項。** nav2 humble
`dwb_plugins/src/kinematic_parameters.cpp`:

```cpp
if (min_speed_xy_ >= 0.0 && vmag_sq + EPSILON < min_speed_xy_sq_ &&
    min_speed_theta_ >= 0.0 && fabs(theta) + EPSILON < min_speed_theta_) {return false;}
```

兩個條件是 **AND**。配上當時的 `min_speed_xy: 0.06` + `min_speed_theta: 0.15`,
速度 `(0,0,0)` 同時低於兩個門檻 → 被判 illegal → **「停住」這條軌跡根本不在候選
名單裡**。只要 goal checker 還沒 latch,DWB 每個 cycle 都被迫挑一條非零軌跡。
終點附近全域路徑只剩幾公分,`PathDist` / `PathAlign` 幾乎分不出高下,就變成在一堆
同分的非零軌跡之間亂挑 —— 看起來就是原地抖。

這和 2026-08-15 拿掉 `RotateToGoal` 是**同一類**問題的兩個實例:終點附近的評分
已經退化,任何「強迫它動」的約束都會變成抖動產生器。

**「卡」是另一回事,不是參數能解的。** `max_vel_x: 0.35` 的真實值約 0.04 m/s
(cmd 路徑 8× 衰減,2026-08-15 就列為待辦第一項、一直沒做)。在那個速度帶靜摩擦
大於驅動力的時間佔比很高,就是黏滑(stick-slip)。任何 critic 權重都改不動它。

**兩件事必須同一次做完**,沒有「先做哪個比較安全」的選項:

| 只做一半 | 後果 |
| --- | --- |
| 只校正、不關下限 | `min_speed_xy` 語意從命令空間 0.0075 m/s 一夕變成真實 0.06 m/s(嚴格 8 倍),44cm 走道會開始噴 `no valid trajectories` |
| 只關下限、不校正 | 退回 2026-07-28 的原始症狀:DWB 挑 0.02-0.03 m/s 軌跡,輪子打滑車不動,`progress_checker` 判定卡住 |

所以 `min_speed_xy: 0.06` 從來就不是解法,它是 8× 衰減的 OK 繃,而這塊 OK 繃自己
會製造抖動。

**這次交付的是骨架,不是數字** —— 真實倍率只能在硬體上量:

- `tools/vel_sweep.py` 加了 **`--axis y`**。x / yaw 是校正(各自對應一個參數),
  y 是量測:`ly` 和 `lx` 乘的是**同一個** `cmd_linear_scale`
  (`driver_node.py:383-384`),y 軸沒有自己的參數可調,它要回答的是「橫移的真實
  上限在哪、有沒有自己的死區」。兩軸斜率差 >15% 就不能共用一個 scale,要改成壓
  `max_vel_y`。鎖頭橫移的跑法會讓一半路程走 y 軸,所以這個數字和 x 一樣重要。
  ⚠ `--axis y` 要留的是**左右**淨空,不是前後。
- `tools/apply_calibration.py`(新增)把所有互相牽連的地方一次改完:FollowPath、
  `velocity_smoother`、`behavior_server`、以及那道速度下限。預設 dry-run 只印 diff,
  `--apply` 才寫檔。改完會重驗三件事:煞停距離 `v²/2|decel|` < `xy_goal_tolerance`、
  `max_vx` 沒超過 `/odom` 飽和天花板(raw 2.77 × `odom_linear_scale` ≈ 0.42 m/s,
  超過之後里程計會無聲少報)、`max_speed_xy` ≥ 兩軸合成。驗算沒過就不寫檔、exit 1。
  冪等,可以重複跑。
- `config/nav2_params.yaml` 的速度區塊和下限區塊註解重寫過,把上面的耦合關係寫進去,
  並標明**不要手動改那兩區**。

踩到的一個坑記在這裡:這些速度參數 nav2 都是以 **double** 宣告的,YAML 裡寫成
`-1` 而不是 `-1.0`,rclcpp 會當成 integer 型別的 parameter override,
`declare_parameter` 當場拋 `InvalidParameterTypeException`,整個 `nav2_container`
起不來 —— 而錯誤訊息只會說某個參數型別不對,不會告訴你是少了 `.0`。
`apply_calibration.py` 的 `fmt()` 因此固定輸出兩位小數。

**下一步(依序,不能跳)**:

1. 量:`./robotctl args "cmd_linear_scale:=1.0 cmd_angular_scale:=1.0"` →
   `./robotctl restart bringup` → `vel_sweep.py` 跑 x / yaw / y 三軸。
2. 套:`apply_calibration.py --lin-scale <x> --ang-scale <yaw>`,**第一趟用預設的
   保守值**(`max_vx 0.20` / `max_vtheta 0.80`)確認「會走、會停、不抖」。
3. 兩個不在 YAML 裡的地方:driver 的 `cmd_*_scale`(launch arg,跑順了要寫回
   `driver_node.py` 和 `robot_bringup.launch.py` 的預設值)、`teleop_node.py:69-70`
   的 `linear_speed: 0.6` / `angular_speed: 1.5`(當初手工補償 8× 加上去的,校正後
   會變成真實 0.6 m/s,**超過 0.42 的天花板**)。腳本最後會把這兩段印出來。
4. 都穩了之後才談「鎖頭橫移」(`max_vel_theta` 壓到 0.4 左右、`max_vel_y` 拉到接近
   `max_vel_x`)—— 省掉每個 90° 轉彎約 1.5 s,一趟約 10 個轉彎。
5. AMCL 那條線還沒查:`update_min_a: 0.05` + `resample_interval: 1` 是否在餵抖動,
   以及 2026-09-12 截圖裡粒子雲明顯沒收斂。和上面是獨立的兩件事。

### 2026-10-10 — 標定完成、兩趟 Nav2 實跑錄檔,以及下次要測什麼

**標定(已完成,捲尺驗過)** —— `vel_sweep.py` 三軸:x 斜率 0.1530 → `cmd_linear_scale 6.54`、
yaw 0.1346 → `cmd_angular_scale 7.43`、y 0.1531(和 x 到小數第四位都一樣 —— 因為板子 PID
把輪速鎖在目標值,回饋是輪速反推,**`/odom` 的 x/y 看不到打滑**,只能用捲尺驗)。
捲尺:直走 1 m 讀 0.998、橫移 1 m 讀 1.074(滾子滑移,AMCL 會修)、自轉 360° 讀 352.6°。
目前只存在 Pi 的 `~/.tirt_robot_args`,**還沒寫回 `driver_node.py` / `robot_bringup.launch.py` 預設值**。
直走 1 m 車頭偏 +8~11°、橫移 1 m 偏 ~14°:陀螺儀量到的真實車身偏轉(使用者認為是馬達座晃,要修機構)。

**AMCL 收緊**(odom 準了):alpha1-4 0.3→0.1、alpha5 0.2→0.15、sigma_hit 0.2→0.1、
z_hit 0.5→0.8、z_rand 0.5→0.2。結果:粒子雲從散到地圖外幾公尺 → std x 5.7cm / y 4.3cm / yaw 5°。✅

**Run 1**(`notes/nav_runs/2026-10-10_run1_swing.jsonl`,參數 = apply_calibration 保守值):
目標正前方 43cm。DWB 從第一個 cycle 就下 `vy=-0.14`(**max_vel_y 上限,往右**),wz 一路爬到
0.8(上限,往左),車頭轉 55° 甩尾進終點;`yaw_goal_tolerance 3.15` 所以朝向不管。
→ 加了 `Twirling.scale 5.0`、加回 `RotateToGoal`、`yaw_goal_tolerance` 3.15→0.15。

**Run 2**(`..._run2_twirling.jsonl`,加 Twirling 之後)—— 更糟,兩段不同的失敗:
1. **起點規劃不出路徑**:目標 47cm 外空地,`GridBased: failed to create plan` ×14,recovery 的
   spin / backup 都立刻 `Collision Ahead`。→ 車子自己的 footprint 在 costmap 裡壓到致命格。
   0912_room 很小、牆近,也可能雷射把人腳或車上的線材標成障礙。**下次失敗當下跑
   `tools/costmap_check.py`**,不要猜。
2. 終於有路徑後:又是 `vx 0.2 / vy -0.14(上限)/ wz -0.69`,而且 `vy=-0.14` 連續 3.6 秒
   一模一樣不變,車子往右前斜衝 ~50cm 偏離目標後 `Goal failed`。

**最強的線索:輪速飽和。** 兩趟裡 odom 量到的速度都只有命令的 ~60%
(cmd `0.2/-0.14` → odom `0.133/-0.078`),而 vel_sweep 單軸時是 1:1。麥克納姆每顆輪的速度
是 `|vx|+|vy|+k|wz|`;`0.2+0.14+0.105×0.7 ≈ 0.41 m/s`,算回去**輪速天花板約 0.21 m/s**。
注意 vel_sweep 只在 scale=1.0 下量到名目 0.6 = **真實 0.09 m/s**,校正後的速度範圍從來沒量過
線性。DWB 的運動模型以為下多少就走多少,實際只走六成 → 預測軌跡全錯 → 累積側向誤差 →
它加碼要更多 vy → 頂到上限卡死。這同時解釋 Run 1 和 Run 2 的 vy 黏在 -0.14。

**下次的測試順序(一次一項,前一項過了才做下一項):**
1. **量真實天花板**(Pi,scale 已是 6.54,所以斜率應該 ≈1.0,彎下去的地方就是天花板):
   `python3 tools/vel_sweep.py --axis x --max 0.35`、`--axis y --max 0.35`、`--axis yaw --max 1.5`
2. **把 Nav2 速度壓進天花板**,讓 `max_vel_x + max_vel_y + 0.105×max_vel_theta` ≤ 天花板的 80%。
   天花板若真是 ~0.21:`apply_calibration.py --lin-scale 6.54 --ang-scale 7.43 --max-vx 0.12
   --max-vy 0.06 --max-vtheta 0.4 --apply`(再手動把 `max_speed_xy` 設 0.12)。
3. 換一個**大一點、空一點的地方**,用校正後的車**重建地圖**(0912_room 是 9/12 未校正時建的,
   房間又小到 costmap 幾乎全紅),再跑 1 m 直線目標。
4. 起點規劃失敗時當下跑 `tools/costmap_check.py`。
5. 跑順了才把 6.54 / 7.43 寫回 driver 預設值並 commit。
