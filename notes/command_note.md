SLAM 啟動筆記

  架構:Pi = 機器人本體(光達+SLAM+底盤),PC = 純 RViz 觀看端,兩台靠 ROS 2 DDS 網路連線,不是 X11 轉發。

  ---
  1. Pi 端(樹莓派,SSH 進去下指令)
  
  cd ~/2026_TIRT
  ./run_robot.sh

  這支 one-click 腳本會自動幫你 source /opt/ros/humble/setup.bash → ~/ros2_ws/install/setup.bash → dds/setup_dds.sh,然後跑
  robot_bringup.launch.py,一次拉起:車體模型 TF、四輪 TF、光達 /scan、SLAM /map、底盤驅動(ominibot_driver,預設真的底盤,不是假里程計)。

  常用參數(直接接在後面):
  ./run_robot.sh use_fake_odom:=true   # 沒接底盤板子,只看模型+光達
  ./run_robot.sh use_slam:=false       # 不建圖

  ▎ dds/setup_dds.sh 已經寫進 Pi 的 ~/.bashrc,但 run_robot.sh 自己也會 source 一次,不用擔心漏掉。

  2. PC 端(筆電,看畫面)

  cd ~/2026_TIRT   # 要先確保 car_assemble_description 已 build 過(package:// mesh 才解析得出來)
  ./run_rviz.sh

  會開 rviz2 並載入存好的 rviz/view_robot.rviz(Grid、RobotModel、LaserScan、Map、Odometry、TF 都設好了)。

  ⚠ 第一次開或剛啟動時,先把 Fixed Frame 從 map 切成 base_link。 SLAM 要跑 ~10–15 秒才會建出 map frame,在那之前用 map 會顯示「does not exist」→
  整個畫面空白,容易誤判成連線失敗。等看到車體+光達點雲正常後,再切回 map 看地圖。

  3. 兩台都要對的網路設定

  - ROS_DOMAIN_ID:兩台必須完全一樣(隨便挑 0~101)。
  - ROS_LOCALHOST_ONLY=0:兩台都要,否則跨機器探索不到彼此。
  - DDS LAN 白名單:Pi 同時有 wlan0 和 tailscale0,不做白名單的話大樣本(/robot_description、/tf、/map)會被路到 tailscale(MTU 1280)分片掉包 —— 這時候 ros2 topic 
  list 看得到 topic 名字,但 RViz 是空的,是最容易誤判的雷。run_robot.sh/run_rviz.sh 都已經自動 source dds/setup_dds.sh 處理這件事了,正常情況不用手動管。
  - 只有換場地/網路時才需要重新 source dds/setup_dds.sh(或重開 terminal),猜錯介面才需要手動 DDS_IFACE=eth0。

  4. 開車建圖

  另開一個 PC 終端機:
  source ~/ros2_ws/install/setup.bash   # 或直接 source ~/2026_TIRT 內對應 setup
  ros2 run ominibot_driver mecanum_teleop
  會開 rviz2 並載入存好的 rviz/view_robot.rviz(Grid、RobotModel、LaserScan、Map、Odometry、TF 都設好了)。

  ⚠ 第一次開或剛啟動時,先把 Fixed Frame 從 map 切成 base_link。 SLAM 要跑 ~10–15 秒才會建出 map frame,在那之前用 map 會顯示「does not exist」→
  整個畫面空白,容易誤判成連線失敗。等看到車體+光達點雲正常後,再切回 map 看地圖。

  3. 兩台都要對的網路設定

  - ROS_DOMAIN_ID:兩台必須完全一樣(隨便挑 0~101)。
  - ROS_LOCALHOST_ONLY=0:兩台都要,否則跨機器探索不到彼此。
  - DDS LAN 白名單:Pi 同時有 wlan0 和 tailscale0,不做白名單的話大樣本(/robot_description、/tf、/map)會被路到 tailscale(MTU 1280)分片掉包 —— 這時候 ros2 topic
  list 看得到 topic 名字,但 RViz 是空的,是最容易誤判的雷。run_robot.sh/run_rviz.sh 都已經自動 source dds/setup_dds.sh 處理這件事了,正常情況不用手動管。
  - 只有換場地/網路時才需要重新 source dds/setup_dds.sh(或重開 terminal),猜錯介面才需要手動 DDS_IFACE=eth0。

  4. 開車建圖

  另開一個 PC 終端機:
  source ~/ros2_ws/install/setup.bash   # 或直接 source ~/2026_TIRT 內對應 setup
  ros2 run ominibot_driver mecanum_teleop
  在房間慢速繞一圈(轉彎更慢),看 RViz 的 /map 長出來。

  5. 存地圖

  cd ~/2026_TIRT
  ./save_map.sh maze_01   # 存到 maps/maze_01.pgm + .yaml
  （這支比手動下 map_saver_cli 好,save_map_timeout 拉到 10 秒,避免預設 2 秒常常抓不到 latched /map 而報錯。）

  ---
  備註:你現有的 雙機RViz連線.md 內容大方向一樣,但裡面待辦提到的「轉彎方向相反」「自訂鍵盤遙控」看起來已經處理掉了(driver_node.py 已有校正好的 sign
  預設值,ominibot_driver 也已經有 mecanum_teleop),那份文件之後有空可以更新一下,避免之後查資料被舊資訊誤導。