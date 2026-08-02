# 模擬環境 — 在沒有硬體的情況下把整套系統學會

## 這是什麼

一個 **2D 假物理**的迷宮模擬器。它取代真車上的「底盤 + 光達」,發出**位元級相同**的
`/scan`、`/odom`、TF 和 `/cmd_vel` 介面 —— 所以 slam_toolbox、Nav2、RViz、teleop
**完全分辨不出自己接的是模擬還是真車**,`run_slam.sh`、`run_nav2.sh`、`save_map.sh`
一個字都不用改。

你在這裡練的每一個指令、每一個除錯手法,比賽當天都是同一套。

## 為什麼不用 Gazebo

規則四.4 明訂只准用光達,而牆是 20cm 高的平面 —— 這台車能感知到的世界**本來就是 2D 的**。
用 3D 物理引擎算重力、摩擦、輪胎接觸,算完再切一刀投影回 2D,對「學會這套系統怎麼運作」
沒有多給任何資訊,卻要付出裝 Gazebo、寫 SDF、調 mecanum plugin、以及在沒有 GPU 的
虛擬機裡跑 3D 算繪的代價。

而且**假物理是刻意的**。真實的打滑、PID、電池垂降本來就模擬不準,做得半像不像只會給你
「模擬過了就沒問題」的錯覺。這裡只保留會**改變上層行為**的那幾件事:加速度上限、
里程計刻度誤差、麥輪打滑、指令延遲/掉包、光達雜訊 —— 也就是 `notes/` 裡那些
歷史 bug 的真正成因。

## 安裝(一次就好)

```bash
./sim/setup_sim.sh
```

它會偵測 ROS 發行版、列出缺少的 apt 套件、把三個套件 symlink 進 `~/ros2_ws/src` 並 build。
缺套件時它只會**印出**安裝指令不會自己 sudo,在 Claude Code 裡可以用 `!` 開頭直接執行。

## 開始用

```bash
./sim/gcs_sim.sh              # 建圖模式:遙控開一圈,把地圖畫出來
./sim/gcs_sim.sh --nav        # 導航模式:Nav2 讀地圖自己走
./sim/gcs_sim.sh --down       # 收工
```

操作方式和實機的 `./gcs.sh` 一模一樣,包括 `Ctrl-b m` 存地圖。

## 和實機的對應關係

| | 實機(Pi + 筆電) | 模擬(單機) |
|---|---|---|
| 入口 | `./gcs.sh` | `./sim/gcs_sim.sh` |
| bringup | `./run_robot.sh`(Pi) | `./sim/run_sim.sh` |
| 底盤 | `ominibot_driver`(序列埠) | `fake_base` |
| 光達 | `sllidar_node`(RPLidar C1) | `fake_lidar` |
| 車體 TF | `robot_state_publisher` | **同一個,同一份 URDF** |
| 建圖 | `./run_slam.sh` | **同一支腳本** |
| 導航 | `./run_nav2.sh` | **同一支腳本** |
| 存圖 | `./save_map.sh` | **同一支腳本** |
| 遙控 | `mecanum_teleop` | **同一個節點** |

模擬多出三個真車**不可能有**的「上帝視角」topic,那正是拿來學習的:

```bash
ros2 topic echo /sim/ground_truth   # 車子真正在哪(odom 是「車以為自己在哪」)
ros2 topic echo /sim/odom_error     # 兩者的差 —— 里程計漂移一眼看穿
ros2 topic echo /sim/collision      # 有沒有撞牆(規則五.5:碰牆即當次失敗)
```

## 學習路線

照順序做,每一步都有明確的「你應該看到什麼」。

**第 1 步 — 看懂 TF 樹**(30 分鐘)
```bash
./sim/gcs_sim.sh --no-rviz
# 另開終端:
# 注意 jazzy 的 view_frames 產生的是「帶時間戳的檔名」(frames_2026-08-02_22.55.06.pdf),
# 不是 humble 的 frames.pdf —— 舊寫法 `&& xdg-open frames.pdf` 在這裡一定會說找不到檔案。
ros2 run tf2_tools view_frames && xdg-open "$(ls -t frames*.pdf | head -1)"
ros2 run tf2_ros tf2_echo odom base_link
```
★ 開跑後**先等 10 秒再看**。slam_toolbox 要收到第一張 scan、完成第一次匹配,才會開始發
`map -> odom`;在那之前 TF 是「兩棵不相連的樹」,view_frames 會畫成 `map -> sim_world`
(模擬的真值分支)和 `odom -> base_link -> ...` 各自獨立,tf2_echo 則說
"they are not part of the same tree"。這是**正常的啟動過程,不是故障** —— 太早按下去
會以為 SLAM 壞了。真的一直不接起來,才去看 `/scan` 有沒有在跑(`ros2 topic hz /scan`
應該是 10Hz)。

確認你能講清楚 `map -> odom -> base_link -> laser_frame` 每一段**由誰發布、代表什麼**。
這是整套系統的骨架,`notes/SLAM_learning_note.md` 有完整說明。搞不清楚這個,
後面所有症狀你都只能猜。

**第 2 步 — 遙控建圖**(30 分鐘)
```bash
./sim/gcs_sim.sh
```
切到 `teleop` 視窗開車。看著 RViz 裡地圖長出來,然後 `Ctrl-b m` 存檔。
觀察:轉彎時地圖有沒有抖?迴路閉合發生時整張圖會抽動一下,那是正常的。

**第 3 步 — 用完美地圖跑 Nav2**(30 分鐘)
```bash
./sim/gcs_sim.sh --down && ./sim/gcs_sim.sh --nav
```
第一次會自動用場地檔產生一張**幾何完美**的地圖。在 RViz 用 `Nav2 Goal` 點目標。
這一步的意義:先在「地圖絕對正確」的前提下確認 Nav2 參數是對的。

**第 4 步 — 跑完整任務**
```bash
python3 tools/run_mission.py --world sim/worlds/tirt_maze.yaml
```
起點 → 中繼區 → 終點,和比賽當天一樣。看總耗時、看撞牆次數。

**第 5 步 — 用自己掃的地圖跑一次**
```bash
./sim/gcs_sim.sh --nav <你在第 2 步存的地圖名>
```
和第 3 步比較。SLAM 掃出來的地圖一定比完美地圖差,差多少?會不會導致撞牆?
**這一步是整個模擬最重要的一課** —— 它量化了「地圖品質」對成績的影響。

**第 6 步 — 故障注入**(最重要,慢慢做)

`sim/faults.md` 有 8 個處方,每一個都對應真車上真的發生過的 bug。
做法:先跑處方 → **不要看答案** → 自己觀察並寫下推論 → 用診斷工具驗證 → 對答案。

## RViz 一片空白怎麼查

按這個順序,每一步都有明確的判準:

```bash
# 1. 光達有在發嗎?(應該 ~10Hz)
ros2 topic hz /scan

# 2. SLAM 有在發地圖嗎?(publisher 應該是 1;是 0 就往下看)
ros2 topic info /map

# 3. slam_toolbox 的 lifecycle 狀態(應該 active)
ros2 lifecycle get /slam_toolbox

# 4. 地圖是真的在長,還是只是重複發同一張?
python3 tools/map_check.py
```

**已知坑 1 — slam_toolbox 停在 `unconfigured`(jazzy 以後)。**
從 slam_toolbox 2.8(jazzy)起它變成 **lifecycle 節點**,而且不會自己啟動。
節點起得來、`ros2 node list` 看得到、log 印完 "Node using stack size" 就沒下文,
但它**完全沒有訂閱 `/scan`,也不發 `/map`** —— RViz 就只是一片空白,沒有任何錯誤訊息。
`run_slam.sh` 現在會自動幫它 configure + activate;手動的話:

```bash
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
```

humble 的 slam_toolbox 是普通節點,沒有這個問題 —— 這是**發行版差異**,不是模擬的問題。

**已知坑 2 — QoS 不相容,兩邊都不報錯。**
「RELIABLE 的訂閱者」配「BEST_EFFORT 的發布者」是不相容的,DDS 直接不配對,
畫面安靜地什麼都沒有。查法:

```bash
ros2 topic info /scan --verbose     # 看每一端的 Reliability
```

**已知坑 3 — 遙控在搶 `/cmd_vel`。**
`mecanum_teleop` 為了餵 watchdog,閒著時也會以 20Hz 發零速度。用腳本送指令時它會
把你的指令蓋掉,現象是「車子完全不動、地圖當然也不長」。先關掉 teleop 視窗(`Ctrl-C`)。
Nav2 模式下 `gcs_sim.sh` 已經不啟動 teleop,就是這個原因。

## 模擬做不到的事(不要有錯覺)

| 只有真車才會有的問題 | 為什麼模擬不出來 |
|---|---|
| 序列埠通訊、BCC 校驗失敗、掉封包 | 沒有硬體。用 `tools/motor_diag.py` 在實機上測 |
| 板子的命令/回授標定誤差(6.5 倍) | 那是那塊板子韌體的特性,不是通用行為 |
| 馬達 PID 震盪、扭矩不足、電池垂降 | 假物理。用 `tools/step_response.py` 在實機上測 |
| 四輪出力不均、齒輪背隙 | 同上,用 `tools/wheel_test.py` |
| WiFi 真實的丟包/分片(tailscale MTU) | 模擬是單機。網路故障是用參數**近似**注入的 |
| Pi 4 的 CPU 是否跑得動 | 筆電比 Pi 快太多 —— **這點特別危險,見下** |

⚠️ **最需要警惕的一項是 CPU**。模擬跑在筆電上,一切都很順;但 `notes/` 記載
Pi 4 跑不動 async slam_toolbox(scan queue 塞爆 → 丟 scan → 地圖旋轉塗抹),
而且 arm64 的 `nav2_mppi_controller` 在 Cortex-A72 上直接 SIGILL 當掉。
**模擬能驗證演算法和參數是對的,不能驗證 Pi 跑不跑得動。**

## 換成真正的比賽場地

規則四.3:「迷宮之場地路徑圖,主辦單位會先行提供予參賽隊伍」。

拿到那張圖之後,編輯 `sim/worlds/tirt_maze.yaml` 的 `maze:` 區塊,照著重畫 ASCII 即可 ——
尺寸(44cm 格、46cm 間距)已經是規則的真值,不用動。畫法:

```
+--+--+--+     奇數列是橫牆,'--' = 有牆
|        |     偶數列是格子列,'|' = 有牆
+  +--+  +     每格佔 3 個字元寬
|  |     |
+--+--+--+
```

畫完驗證:
```bash
python3 sim/tirt_sim/make_map.py sim/worlds/tirt_maze.yaml /tmp/check
```
它會印出尺寸。要目視確認牆的位置,直接跑 `./sim/gcs_sim.sh --nav` 在 RViz 裡看。
