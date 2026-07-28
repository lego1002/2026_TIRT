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
| `robot_radius` | 0.11 | 車體圓形半徑。障礙物 0.11 m 內 = 內切致命區，planner 不會讓車心進去 |
| `inflation_radius` | 0.18 | 代價膨脹範圍。比 `robot_radius` 大的那一段是「可以進但不鼓勵」的緩衝 |
| `cost_scaling_factor` | 5.0 | 代價衰減陡度。越大 → 離牆一點點就變便宜 → 越敢貼牆走 |

走道寬 W 時，車心可走的自由帶 ≈ `W - 2 × robot_radius`。走道 0.5 m → 0.28 m（夠）；0.4 m → 0.18 m（很擠）。

**找不到路的調整順序**：`inflation_radius` 0.18 → 0.13 → 再考慮 `robot_radius` 0.11 → 0.09。
`robot_radius` 是安全邊界，最後才動。

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
| `update_min_d` / `update_min_a` | 0.10 / 0.15 | 走多遠 / 轉多少才做一次更新。跑快了可以維持，CPU 吃緊才放大 |
| `max_beams` | 120 | 每次匹配用幾條光束。房間小、牆多所以給得比原版 60 多 |
| `transform_tolerance` | 1.0 | 跨 WiFi 的 TF 容忍，**不要降**（見 `0721_net_issue_plan.md`） |
| `set_initial_pose` / `initial_pose` | true / (0,0,0) | 只有「車放回建圖起跑點」時才正確；放別處要在 RViz 點 2D Pose Estimate |
| `robot_model_type` | `nav2_amcl::OmniMotionModel` | **不要改回 Differential**，那會把橫移當成不可能的運動 |

定位飄掉時，先確認不是**里程計或光達外參**的問題（那是上游，AMCL 再怎麼調都救不了）：
`laser_x` / `laser_y` / `laser_yaw`、`odom_linear_scale`、`gyro_scale` 的校正程序在
`notes/SLAM_learning_note.md` §7，工具是 `tools/odom_check.py` 和 `tools/analyze_bag.py`。

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
| 完全不動、也沒報錯 | Pi 上的 teleop 還在跑嗎（`./robotctl down teleop`）？它閒著會以 20Hz 發零速度 |
| 車體模型不見 | `/robot_description` 是 latched，RViz 的 durability 要 Transient Local |
| 一頓一頓 | `tools/cmd_vel_check.py` 看 watchdog 逾時次數；不是網路就換 `tools/motor_diag.py` |
| 地圖和光達對不上 | 定位問題（§7），不是導航問題 |
| 「no valid trajectories」 | §2a 的 `min_speed_*` 太高，或 §3 的膨脹把走道封死了 |
| 定位莫名其妙地爛，但完全沒有錯誤訊息 | **另一台機器上還開著 `run_slam.sh`。** DDS 是跨機的：它的即時 `/map` 會和 `map_server` 的存檔地圖同時餵給 AMCL，兩邊還都在發 `map->odom`。`run_nav2.sh` 現在會在啟動前檢查並警告 |
| 啟動後很久都沒有任何 log | 有殘留的 `nav2_container` 孤兒在吃 CPU。它**不理 SIGTERM**，要 `pkill -9 -f "component_container_isolated.*nav2_container"` |
