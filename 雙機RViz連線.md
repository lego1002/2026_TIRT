# 雙機視覺化:Pi 純後端跑,Ubuntu PC 看 RViz

架構:**樹莓派(headless server)= 機器人本體**,跑光達 + SLAM + 車體模型;
**Ubuntu PC = 純觀看端**,只跑 RViz2。兩台同一個 Wi-Fi/網段,靠 ROS 2 DDS 自動探索,
資料走網路傳輸,不是 `ssh -X` 把視窗轉過來(效能好很多,也不佔 Pi 的 GPU)。

SSH 只是用來「登入 Pi 下指令啟動後端」,RViz 本身跑在 PC 上。

```
┌─────────────── 樹莓派 (headless) ───────────────┐        ┌──────── Ubuntu PC ────────┐
│ ros2 launch car_assemble_description             │  DDS   │ rviz2 -d view_robot.rviz  │
│   robot_bringup.launch.py                        │◄──────►│ (看車體 + 光達 + 地圖)     │
│   = robot_state_publisher + joint_state_publisher│  網路  │                           │
│   + 光達驅動 + SLAM + (暫時)fake odom            │        │                           │
└──────────────────────────────────────────────────┘        └───────────────────────────┘
```

---

## 〇、Pi 端一次性前置(只做一次)

這兩個套件 Pi 上還沒裝,bringup 需要它們:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-slam-toolbox ros-humble-joint-state-publisher
```

`car_assemble_description` 已經連進 `~/ros2_ws/src/` 並 build 好了(用 symlink,以後改 launch/rviz 檔不用重 build;
改 `package.xml`/`CMakeLists.txt` 才要重 build)。

---

## 一、兩台都要設的網路環境變數

在 **Pi 和 PC 各自的 `~/.bashrc` 最後**都加上這幾行(`ROS_DOMAIN_ID` 兩台要一模一樣):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=69      # 隨便挑 0~101,但兩台必須相同
export ROS_LOCALHOST_ONLY=0  # 一定要 0,才能跨機器通訊
# 只走 LAN、排除 tailscale/其他介面(見下方「多介面」說明)。
# 這行會自動偵測本機當下 LAN IP 並設好 FASTRTPS_DEFAULT_PROFILES_FILE,換場地免手改;
# 兩台共用同一支腳本(各自偵測自己的 IP)。介面猜錯時:DDS_IFACE=eth0 source .../setup_dds.sh
source /home/lego/2026_TIRT/dds/setup_dds.sh
```

Pi 端還要多 source 工作區:

```bash
source ~/ros2_ws/install/setup.bash
```

改完各自 `source ~/.bashrc` 或重開 terminal。

> 確認兩台在同一網段:各自 `hostname -I` 看 IP,前三段要一樣(例如都是 `192.168.50.x`)。

### ⚠ 多介面(wlan0 + tailscale0)—— 本專案踩過的雷

Pi 同時有 `wlan0`(192.168.50.x)和 `tailscale0`(100.x)兩個介面。預設 Fast DDS 會把**兩個 IP 都當 locator 公告**。結果:

- `ros2 topic list` 這種小 discovery 封包兩條路都通 → **看得到 topic**;
- 但 `/robot_description`、`/tf`、`/map` 這類**大樣本**一旦被對方走 tailscale(MTU 只有 1280)傳,就分片掉包 → **RViz 收得到訂閱卻沒資料 → 整個畫面空白**。

修法:把 DDS 的 `interfaceWhiteList` 綁死在 LAN IP + 127.0.0.1。`dds/fastdds_lan.xml` 現在是**範本**
(`<address>` 是佔位符 `@LAN_IP@`,別直接指它,會解析失敗)—— 改 `source dds/setup_dds.sh`,它會偵測
本機當下 LAN IP、渲染成 `$XDG_RUNTIME_DIR/fastdds_active.xml` 並匯出 `FASTRTPS_DEFAULT_PROFILES_FILE`。
**兩台都要 source**(各自偵測自己的 IP);Fast DDS 2.6 的白名單只吃 IP 不吃介面名,所以才需要這支腳本
自動代入。換場地 / wlan0 換 IP 只要重開 terminal 或重新 `source` 即可,不必手改 xml,也不必綁固定 IP。
前提仍是**兩台要在同一區網**(各自 `hostname -I` 前三段相同)。

---

## 二、Pi 端:啟動機器人後端(headless,不開任何視窗)

SSH 進 Pi 後:

```bash
ros2 launch car_assemble_description robot_bringup.launch.py
```

這一行會拉起:車體模型(`/robot_description` + TF)、四顆輪子 TF、光達 `/scan`、SLAM `/map`、
以及暫時的 `odom->base_link` 假里程計。**Pi 上不會、也不該開 RViz。**

常用選項:

```bash
# 只看光達 + 車體,先不建圖
ros2 launch car_assemble_description robot_bringup.launch.py use_slam:=false

# 之後 OminiBotHV 底盤驅動寫好、會自己發 odom 時,關掉假里程計
ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=false
```

啟動後另開一個 SSH 視窗驗證 topic 有出來:

```bash
ros2 topic hz /scan          # 應該有頻率(C1 約 10Hz)
ros2 topic echo /robot_description --once | head   # 應印出 URDF
ros2 topic list | grep -E "scan|map|robot_description|tf"
```

---

## 三、PC 端:一次性準備(只做一次)

PC 要能顯示車體 mesh,必須有這個描述套件(RViz 會在**本機**解析 `package://car_assemble_description/meshes/...`)。

```bash
# 1) 先裝 git-lfs 再 clone,否則 STL 只會是指標檔、車體顯示不出來
sudo apt-get install -y git-lfs && git lfs install
git clone git@github.com:lego1002/2026_TIRT.git ~/2026_TIRT
cd ~/2026_TIRT && git lfs pull

# 2) 把描述套件放進 PC 自己的 ROS2 工作區並 build
mkdir -p ~/ros2_ws/src
ln -s ~/2026_TIRT/car_assemble_description ~/ros2_ws/src/car_assemble_description
cd ~/ros2_ws && colcon build --packages-select car_assemble_description --symlink-install
```

> PC 端**不需要**裝 slam_toolbox / sllidar_ros2 / my_robot_lidar —— 那些都在 Pi 上跑。PC 只負責 rviz2。

---

## 四、PC 端:開 RViz 看

每次要看時(Pi 那邊 bringup 已經在跑):

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/install/car_assemble_description/share/car_assemble_description/rviz/view_robot.rviz
```

`view_robot.rviz` 已經幫你設好:Fixed Frame = `map`、RobotModel 訂閱 `/robot_description`、
LaserScan = `/scan`、Map = `/map`(Durability 已改 **Transient Local** 才收得到 latched 地圖)、TF 全開。

> **第一次連、除錯時先把 Fixed Frame 改成 `base_link`(不要用 `map`)。** slam 要跑 ~10~15 秒才會建出
> `map` frame,在那之前 Fixed Frame=`map` 會顯示「Fixed Frame [map] does not exist」→ 整個畫面空白,
> 很容易誤判成「連不上」。用 `base_link` 車體 + 輪子 + 光達點雲會**立刻**出現(不依賴 slam);確認畫面
> OK 後再切回 `map` 看地圖。

驗證 PC 有看到 Pi 的節點:

```bash
ros2 node list      # 應該看得到 sllidar_node、slam_toolbox、robot_state_publisher 等
# 關鍵:topic list 有、不代表資料有進來。下面兩個要「真的印出東西」才算跨機資料通:
ros2 topic echo /robot_description --once | head   # 收不到 → 大樣本走錯介面(見多介面雷)
ros2 topic hz /tf                                   # 應持續有頻率
ros2 topic hz /scan # PC 這邊也收得到才代表跨機通訊 OK
```

---

## 五、連不到的排查(DDS 探索問題)

ROS 2 預設用 multicast 自動探索,大多數家用路由器 OK;若 `ros2 node list` 在 PC 看不到 Pi 的節點:

1. 先確認 **ROS_DOMAIN_ID 兩台相同、ROS_LOCALHOST_ONLY 都是 0**(最常見的錯)。
2. 確認同網段、防火牆沒擋:PC 上 `ping <Pi 的 IP>` 要通。
3. 有些 Wi-Fi AP 會擋 multicast。改用「指定對方 IP」的單播探索,兩台都設(把 IP 換成對方的):

   ```bash
   # 在 Pi 上,指向 PC 的 IP;在 PC 上,指向 Pi 的 IP
   export ROS_STATIC_PEERS=<對方的 IP>
   ```

   (Humble 較新版本支援 `ROS_STATIC_PEERS`;若無效,改用 Fast DDS 的 XML 設定檔指定 unicast peer。)

---

## 六、接上底盤後:開車 + 建圖 + 存地圖

底盤驅動(`ominibot_driver`)已完成(2026-07-13)。實機驗證過:板子以 ~30Hz 串流回授,
`/odom`、`odom->base_link` TF、`/imu` 都正常,電壓/四元數解析正確。

**Pi 端**改用真底盤啟動(關掉假里程計):

```bash
# udev 規則裝好後(/dev/ominibot 存在):
ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=false
# udev 還沒裝、想先用 /dev/ttyUSB1:
ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=false ominibot_port:=/dev/ttyUSB1
```

先裝底盤 udev 規則(和光達那條一樣做法):

```bash
sudo cp udev/99-ominibot.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
```

**PC 端**開 RViz(同前),再開一個終端機用鍵盤遙控發 `/cmd_vel`:

```bash
sudo apt-get install -y ros-humble-teleop-twist-keyboard   # 只裝一次
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # i/j/l/, 開車,先把速度調小
```

在房間裡**慢速**繞一圈(轉彎更要慢,麥輪原地平移也可以),看 RViz 的 `/map` 長出來。滿意後存地圖:

```bash
# 在 PC 或 Pi 任一台(要 source 到有 slam_toolbox 的環境;PC 沒裝就在 Pi 上存)
ros2 run nav2_map_server map_saver_cli -f ~/my_room_map   # 產生 my_room_map.pgm + .yaml
```

> 安全提醒:`ominibot_driver` 有 watchdog,`/cmd_vel` 超過 `cmd_vel_timeout`(預設 0.5s)沒更新就自動停車;
> 關掉 teleop 或斷線車子會停,不會暴衝。

---

## 待辦

### 下次要修(2026-07-14 記錄,實測後發現)

1. **旋轉方向相反** —— 用了 `vx_sign:=-1.0` 把前進修正後,轉彎(`wz`/`angular.z`)方向變成反的。
   下次確認後,大概是要再加 `wz_sign:=-1.0`(甚至可能連 `vy_sign` 也要翻);若三軸都固定了,
   可考慮把正確的預設值直接寫進 `ominibot_driver` 或 `robot_bringup.launch.py`,不用每次帶參數。
2. **自訂鍵盤遙控(不要按 Shift)** —— 現在用 `teleop_twist_keyboard`,平移(左右橫向漂移 `linear.y`)
   和斜向(X 形對角移動)要按住 Shift 才有,很不順手。下次寫一支自訂的 keyboard teleop node
   (可放進 `ominibot_driver`),把**左右橫移**和**四個對角(X 形)**都放到一般按鍵上,一鍵直接發
   對應的 `/cmd_vel`(麥輪本來就能全向移動),省掉 Shift。
3. **地圖會漂移(drift/shift)** —— 建圖時地圖會偏移/飄,原因還沒找到,先記錄。下次查方向:
   里程計累積誤差、`odom` 積分(`ominibot_hv` 的輪徑/輪距參數是否正確)、`/scan` 與 TF 時間戳對不上、
   或 slam_toolbox scan-matching 參數。可先比對 `ros2 topic echo /odom` 的位移和實際移動距離是否吻合。

### 其他(先前記錄)

- URDF 四顆輪子的 `<limit effort velocity>` 目前仍缺(見 `urdf閱讀方法.md`),接 `ros2_control` 前要補。
- `ominibot_driver` 的 `/imu` 目前只發四元數(orientation);accel/gyro 的 byte 排列與縮放還沒驗證,
  之後若要做 IMU/里程計融合(robot_localization)再補。
