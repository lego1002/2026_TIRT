"""Pi 端 headless 整合啟動檔（給樹莓派跑,不開任何 GUI）。

一次拉起:
  1. robot_state_publisher —— 用真正的 URDF 發布車體/輪子/lidar_link 的 TF 與 /robot_description
  2. joint_state_publisher —— headless(非 GUI)版,把四顆 continuous 輪子 joint 補 0,讓輪子 TF 存在
  3. sllidar_node + base_link->laser_frame 靜態 TF —— 2026-07-25 從 my_robot_lidar/
        lidar_start.launch.py 搬進本 repo(那邊的 TF 是「假設光達在車體中心」的全 0 佔位值,
        而且檔案在 repo 外不進版控)。外參改成 laser_x/laser_y/laser_z/laser_yaw 四個
        launch arg,現場可直接帶參數迭代校準,見 SLAM_learning_note.md §7.3。
  4. 底盤(預設 use_fake_odom=false):跑 ominibot_driver(收 /cmd_vel、發 /odom + 真 odom->base_link TF);
        use_fake_odom=true → 改發假的 odom->base_link 靜態 TF(無硬體純看模型時用)
  5. slam_toolbox(async)—— 讀本 repo config/ 的 mapper 參數(預設不啟動,SLAM 在 PC 端跑)
  6. oled_status —— 車上 SPI OLED 狀態顯示(預設不啟動)。比賽當天沒有筆電接著,
        電池電量/光達是否還活著只能靠這片螢幕看,見 use_oled。

RViz 一律不在這裡開;請在另一台 Ubuntu PC 上用相同 ROS_DOMAIN_ID 連過來看
(Pi 端一鍵用 repo 根目錄的 run_robot.sh;PC 端一鍵用 run_rviz.sh。
 見 car_assemble_description/rviz/view_robot.rviz 與 repo 內的雙機連線說明)。

用法:
  ros2 launch car_assemble_description robot_bringup.launch.py                       # 預設:真底盤+光達+SLAM
  ros2 launch car_assemble_description robot_bringup.launch.py use_slam:=false       # 只出光達+模型,不建圖
  ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=true   # 沒接底盤,只看模型/光達
  ros2 launch car_assemble_description robot_bringup.launch.py ominibot_port:=/dev/ttyS0    # 底盤改接到別的 UART 時
  ros2 launch car_assemble_description robot_bringup.launch.py laser_x:=0.02 laser_y:=-0.01 # 校光達外參時(不必 rebuild)
  ros2 launch car_assemble_description robot_bringup.launch.py use_oled:=true          # 車上 OLED 顯示電池/狀態
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')

    urdf_path = os.path.join(desc_share, 'urdf', 'CAR_ASSEMBLE_URDF.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # SLAM 設定檔改放本 repo(car_assemble_description/config),讓 PC 端只靠這個
    # repo 就能跑 slam,不必安裝 my_robot_lidar。Pi 端保留 use_slam:=true 的單機
    # fallback 時也讀同一份,避免兩份 config 漂移。
    slam_config = os.path.join(desc_share, 'config', 'mapper_params_online_async.yaml')

    use_slam = LaunchConfiguration('use_slam')
    use_fake_odom = LaunchConfiguration('use_fake_odom')
    use_oled = LaunchConfiguration('use_oled')
    oled_controller = LaunchConfiguration('oled_controller')
    oled_dc = LaunchConfiguration('oled_dc')
    oled_rst = LaunchConfiguration('oled_rst')
    batt_full_v = LaunchConfiguration('batt_full_v')
    batt_empty_v = LaunchConfiguration('batt_empty_v')
    ominibot_port = LaunchConfiguration('ominibot_port')
    vx_sign = LaunchConfiguration('vx_sign')
    vy_sign = LaunchConfiguration('vy_sign')
    wz_sign = LaunchConfiguration('wz_sign')
    wheel_diameter_mm = LaunchConfiguration('wheel_diameter_mm')
    wheel_space_mm = LaunchConfiguration('wheel_space_mm')
    axle_space_mm = LaunchConfiguration('axle_space_mm')
    encoder_ppr = LaunchConfiguration('encoder_ppr')
    gear_ratio = LaunchConfiguration('gear_ratio')
    motor_pwm_max = LaunchConfiguration('motor_pwm_max')
    motor_pwm_min = LaunchConfiguration('motor_pwm_min')
    pos_kp = LaunchConfiguration('pos_kp')
    pos_ki = LaunchConfiguration('pos_ki')
    pos_kd = LaunchConfiguration('pos_kd')
    vel_kp = LaunchConfiguration('vel_kp')
    vel_ki = LaunchConfiguration('vel_ki')
    odom_linear_scale = LaunchConfiguration('odom_linear_scale')
    cmd_linear_scale = LaunchConfiguration('cmd_linear_scale')
    cmd_angular_scale = LaunchConfiguration('cmd_angular_scale')
    motor_direct = LaunchConfiguration('motor_direct')
    encoder_direct = LaunchConfiguration('encoder_direct')
    odom_angular_scale = LaunchConfiguration('odom_angular_scale')
    use_gyro_heading = LaunchConfiguration('use_gyro_heading')
    gyro_z_sign = LaunchConfiguration('gyro_z_sign')
    gyro_scale = LaunchConfiguration('gyro_scale')
    gyro_auto_bias = LaunchConfiguration('gyro_auto_bias')
    odom_max_gap = LaunchConfiguration('odom_max_gap')
    cmd_sync_to_feedback = LaunchConfiguration('cmd_sync_to_feedback')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    cmd_vel_best_effort = LaunchConfiguration('cmd_vel_best_effort')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    return LaunchDescription([
        # 預設 false:SLAM 已改到 PC 端跑(async scan matching 太吃 CPU,Pi 4 追不上
        # 10Hz 掃描 → queue full 丟 scan → 地圖旋轉塗抹)。PC 端用 run_slam.sh /
        # slam_pc.launch.py。設 use_slam:=true 才會在 Pi 上單機建圖(fallback)。
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='是否在「Pi 本機」啟動 slam(預設 false,SLAM 改在 PC 跑;true=單機 fallback)'),
        DeclareLaunchArgument('use_fake_odom', default_value='false',
                              description='false=跑真底盤 ominibot_driver(預設,一鍵開底盤);true=只發假 odom 靜態 TF(無硬體純看模型時用)'),
        # 車上 SPI OLED 狀態顯示。預設 false:這片螢幕是選配硬體,沒插的時候
        # luma 開 /dev/spidev0.0 會直接丟例外把整個 bringup 拉掉。
        DeclareLaunchArgument('use_oled', default_value='false',
                              description='車上 SPI OLED 顯示電池/狀態(需接 OLED 模組)'),
        DeclareLaunchArgument('oled_controller', default_value='ssd1306',
                              description='OLED 控制器型號:ssd1306 或 sh1106'),
        DeclareLaunchArgument('oled_dc', default_value='24',
                              description='OLED DC 腳位(BCM 編號,實體 pin 18)'),
        DeclareLaunchArgument('oled_rst', default_value='25',
                              description='OLED RST 腳位(BCM 編號,實體 pin 22)'),
        DeclareLaunchArgument('batt_full_v', default_value='12.6',
                              description='電量條滿電電壓(預設 3S LiPo,請量測實際電池)'),
        DeclareLaunchArgument('batt_empty_v', default_value='10.5',
                              description='電量條沒電電壓(預設 3S LiPo 安全下限)'),
        DeclareLaunchArgument('ominibot_port', default_value='/dev/serial0',
                              description='OminiBotHV 底盤板序列埠(接 Pi GPIO UART=/dev/serial0;改接別的 UART 時可設 /dev/ttyS0)'),
        # 這三個預設值已對齊 driver_node.py 硬體實測後的正負號;若之後方向再有變,兩邊要一起改。
        DeclareLaunchArgument('vx_sign', default_value='1.0',
                              description='前後反了就設 -1.0(x 前進為正)'),
        DeclareLaunchArgument('vy_sign', default_value='-1.0',
                              description='左右平移(實測底盤與 REP-103 相反 → 預設 -1.0)'),
        DeclareLaunchArgument('wz_sign', default_value='-1.0',
                              description='旋轉方向(實測底盤與 REP-103 相反 → 預設 -1.0)'),
        # 車體幾何(mm),寫進韌體換算輪速/轉向刻度;校準流程見 SLAM_learning_note.md §7。
        DeclareLaunchArgument('wheel_diameter_mm', default_value='48',
                              description='輪徑(直線距離刻度)。錯了會造成 odom 距離等比例縮放。'),
        DeclareLaunchArgument('wheel_space_mm', default_value='115',
                              description='左右輪中心距(旋轉刻度,和 axle_space 之和決定 ωz 換算)。實測值。'),
        DeclareLaunchArgument('axle_space_mm', default_value='96',
                              description='前後軸中心距(旋轉刻度,同上)。實測值。'),
        # 馬達/編碼器刻度:CircusPi 原廠預設(165/55)是別台機器的,和 wheel_diameter
        # 連乘決定 odom 距離倍率;換 N20 後必須改。校準流程見 SLAM_learning_note.md §7。
        DeclareLaunchArgument('encoder_ppr', default_value='165',
                              description='編碼器每轉脈衝數(原廠 165 為別台機器,換馬達要改)。'),
        DeclareLaunchArgument('gear_ratio', default_value='55',
                              description='減速比(原廠 55 為別台機器 1:55,換馬達要改)。'),
        # PWM duty 上下限(寫進韌體,範圍 1~7199 = 0~100% duty)。原廠 3600/2100 來自
        # 廠商自己的註解 "motor range: 3v-6v" —— 那是 12V 供電下**保護 6V 馬達**的限制。
        # 這台用的是 12V 200rpm N20,所以 3600 = 50% duty = 6V,馬達只吃到額定的一半,
        # 扭矩餘裕直接砍半。空中無負載永遠碰不到上限(所以怎麼測都正常),放到地板上
        # 某顆輪子需要的扭矩超過 6V 能給的,PID 就飽和、掉隊 → 四輪不同步。
        # 往 6800(94%)調在馬達額定內,但**先確認供電真的是 12V**,並且逐步往上加。
        # ※ 不是這次「一頓一頓」的元凶:上禮拜好好的時候這兩個值一樣。這是餘裕問題。
        DeclareLaunchArgument('motor_pwm_max', default_value='3600',
                              description='PWM duty 上限(1~7199)。原廠 3600=50% duty,12V 供電下馬達只吃到 6V;12V 馬達可試 6800。'),
        DeclareLaunchArgument('motor_pwm_min', default_value='2100',
                              description='PWM duty 下限(1~7199)。原廠 2100=29% duty,低於此輪子不動(靜摩擦死區)。'),
        # 閉環 PID 增益:原廠為 1:55 重底盤調的,馬達不匹配可能過衝震盪。
        # 可從命令列調小(如 vel_kp:=1500)現場壓振動,不必重新 build。
        DeclareLaunchArgument('pos_kp', default_value='3000', description='位置環 Kp'),
        DeclareLaunchArgument('pos_ki', default_value='1050', description='位置環 Ki'),
        DeclareLaunchArgument('pos_kd', default_value='0', description='位置環 Kd'),
        DeclareLaunchArgument('vel_kp', default_value='3000', description='速度環 Kp'),
        DeclareLaunchArgument('vel_ki', default_value='1050', description='速度環 Ki'),
        # odom 刻度校正:板子回授速度用寫死的內部校正,不吃上面的幾何 config,
        # 實測 raw 灌水 6.54x → 乘 0.153 修回真實單位。這是唯一有效的 odom 校準手段。
        # 用 tools/odom_check.py 跑 §7.1 / §7.2 校準;設 1.0 看原始輸出。
        DeclareLaunchArgument('odom_linear_scale', default_value='0.153',
                              description='直線速度校正倍率(2026-07-25 實測 1m: raw 灌水 6.54x → 0.153)。'),
        DeclareLaunchArgument('odom_angular_scale', default_value='0.195',
                              description='輪速 az 的旋轉校正(只在 use_gyro_heading=false 時用)。'),
        # 命令刻度校正 —— 同一個標定誤差的**另一半**。板子收到 /cmd_vel 也用同一套
        # 寫死的內部校正去解讀,2026-07-25 實測命令 0.15 m/s 車子只跑 0.019 m/s。
        # 在此之前只修了回授那一半,所以 teleop 預設值才會被迫加到 0.6 m/s / 1.5 rad/s
        # 這種對 15cm 小車來說荒謬的數字。用 tools/vel_sweep.py 校(跑之前先設 1.0)。
        DeclareLaunchArgument('cmd_linear_scale', default_value='1.0',
                              description='送給板子的直線速度倍率(1.0=未校正,用 tools/vel_sweep.py 校)。'),
        DeclareLaunchArgument('cmd_angular_scale', default_value='1.0',
                              description='送給板子的角速度倍率(1.0=未校正,用 tools/vel_sweep.py --axis yaw 校)。'),
        # 馬達/編碼器方向 bitmask(原廠值,對應 CircusPi 參考車的接線,從未對這台驗證過)。
        # 2026-07-25 曾因 tools/wheel_test.py 報「三顆輪子只有單一方向能動」而懷疑這裡,
        # 但那個結果已作廢:重跑一次故障輪就換一顆,且操作者目視確認四顆輪子正反轉都正常。
        # 真正原因是回報值在 ~2.77 飽和,平均值量到的其實是起步時間。**沒有證據顯示接線有問題**,
        # 這兩個維持原廠值,開出來只是為了萬一日後又懷疑時能便宜地驗證。
        DeclareLaunchArgument('motor_direct', default_value='0',
                              description='馬達方向 bitmask(原廠 0,未驗證)。'),
        DeclareLaunchArgument('encoder_direct', default_value='10',
                              description='編碼器方向 bitmask(原廠 10=0b1010,未驗證;試 0 或 15)。'),
        # 航向來源:輪速 az 被麥輪打滑毀掉(實測轉 360° 輪速報 2270°),改用板子
        # 原始陀螺儀 Z 積分(實測 360° 準到 ~350°,不受打滑影響)。四元數無磁力計 yaw 凍結不能用。
        DeclareLaunchArgument('use_gyro_heading', default_value='true',
                              description='true=odom 朝向用陀螺儀(麥輪車正解);false=退回輪速 az。'),
        DeclareLaunchArgument('gyro_z_sign', default_value='1.0',
                              description='陀螺 Z 正負號(odom 轉向反了就設 -1.0)。'),
        DeclareLaunchArgument('gyro_scale', default_value='1.014',
                              description='陀螺積分倍率(2026-07-25 tools/analyze_bag.py:odom 報 145.3° vs scan 真值 137.1° → 1.075×0.9435)。'),
        # 陀螺零點漂移(bias)。2026-08-30 實測:車完全靜止 60 秒,raw gyro_z 平均 -0.000447 rad/s,
        # 積出來的 odom 朝向自己轉了 -1.6°/分鐘 —— 誤差跟「經過多久」成正比,跟「走多遠」無關,
        # 這正是「一開始好好的、後面才歪」而且「每次歪的程度不一樣」的成因(MEMS 零點每次開機、
        # 每個溫度都不同)。driver 現在會在啟動時量一次、之後只要車停著就慢慢跟著修。
        DeclareLaunchArgument('gyro_auto_bias', default_value='true',
                              description='true=自動量測並扣掉陀螺零點漂移(建議);false=用原始讀值。'),
        # 底盤回饋斷多久以內 odom 還照樣積分。2026-09-11 實測:Pi 欠壓(Undervoltage detected!)→ CPU 被砍到
        # 600 MHz → UART FIFO 溢位 → 5 秒內 90% 的回饋 frame 壞掉,兩筆好 frame 之間拉到 ~0.7 s;
        # 舊的寫死 0.5 s 上限會把那段位移整段丟掉 → 車在走、/odom 不動 → SLAM 不插 scan、RViz 的 scan
        # 離牆 20 cm。現在 1.0 s 以內用前後兩筆速度平均積分,超過才丟並且 WARN。
        DeclareLaunchArgument('odom_max_gap', default_value='1.0',
                              description='回饋兩筆之間隔多久以內 odom 仍積分(秒);超過就丟掉該段並 WARN。'),
        # 指令改成「收到一張回饋 frame 就馬上寫」而不是獨立 20 Hz timer。板子 TX 到一半收到指令會把那張
        # frame 截斷(desync、bcc=0);Pi 的 timer 和板子的 20 Hz 差一點點,相位慢慢滑過去就形成
        # 「每 ~100 s 爆 5~10 s、最糟 90% 壞 frame」的拍頻(2026-09-11 實測,車停著也一樣,
        # kernel UART overrun=0)。跟著回饋寫就永遠落在 frame 之間的空檔。
        DeclareLaunchArgument('cmd_sync_to_feedback', default_value='true',
                              description='true=指令緊跟在每張回饋 frame 之後送(消除拍頻 desync);false=獨立 cmd_rate timer。'),
        # /cmd_vel 的 watchdog 與 QoS —— 2026-07-27 為了「WiFi 抖動害底盤一頓一頓」開出來。
        # 舊的 0.5s 太短:命令從筆電經 WiFi 過來,傳輸卡個幾百毫秒 watchdog 就把底盤歸零、
        # 下一筆到了又衝出去,操作者看到的就是一頓一頓。根治手段是把 teleop 搬到 Pi 上跑
        # (gcs.sh / pi/robot_tmux.sh 已經這麼做,/cmd_vel 根本不過網路),這兩個參數是留給
        # nav2 或其他 PC 端發布者的餘裕。
        DeclareLaunchArgument('cmd_vel_timeout', default_value='1.0',
                              description='多久沒收到 /cmd_vel 就把底盤歸零(秒)。'),
        DeclareLaunchArgument('cmd_vel_best_effort', default_value='true',
                              description='true=/cmd_vel 用 BEST_EFFORT(丟包的 WiFi 上不重傳過期指令);false=RELIABLE。'),
        # 光達外參(base_link -> laser_frame)。舊的 my_robot_lidar/lidar_start.launch.py
        # 這裡填全 0(註解自承是「假設雷達安裝在機器人中心上方」),但光達並不在中心:
        # URDF 的 lidar_joint 在 (0.0088, -0.061, 0.0427),而四顆輪子 joint 原點算出的
        # 運動學中心(odom 實際追蹤的點)在 base_link 的 (-0.0047, -0.047) —— 兩者相減,
        # 光達相對運動學中心約 (+0.014, -0.014)。偏移填 0 的後果:原地旋轉時光達其實在
        # 繞一個小圓走,SLAM 卻以為它釘在原點 → 每轉一次牆就被畫歪 → 雙線牆、走廊彎折。
        # laser_yaw 已於 2026-07-25 用 tools/analyze_bag.py 實測校準(見下)。
        # laser_x/laser_y 仍是 CAD 推算值,尚未校(量級只有 1.4cm,遠小於 yaw 的影響)。
        DeclareLaunchArgument('laser_x', default_value='0.014',
                              description='光達相對運動學中心的前後偏移(m,前為正)。用 §7.3 原地旋轉疊圖法校。'),
        DeclareLaunchArgument('laser_y', default_value='-0.014',
                              description='光達相對運動學中心的左右偏移(m,左為正)。用 §7.3 原地旋轉疊圖法校。'),
        DeclareLaunchArgument('laser_z', default_value='0.109',
                              description='光達離地高度(m)。2D SLAM 用不到,只影響 RViz 立體顯示。'),
        # ★ 2026-07-25 實測:光達幾乎是反裝的。tools/analyze_bag.py 在直線段解出
        #   車在光達座標系的行進方向是 -167.0°,而 odom 說是 +0.0° → 兩者差 167°。
        #   品質:航向變化僅 0.5°、vy=0.0000、43 幀逐幀估計標準誤差 ±0.8°。
        #   與 CAD 互相印證(輪子座標顯示 base_link 整個轉了 180°),差的 13° 是實際安裝歪斜。
        #   先前填 0.0 是致命錯誤:odom 說往前、光達看到的世界卻幾乎反向流動,
        #   scan matching 每一步都在對抗一個近乎顛倒的運動模型 → 地圖扇形塗抹。
        DeclareLaunchArgument('laser_yaw', default_value='2.9146',
                              description='光達 0° 相對車頭的旋轉(rad)。2026-07-25 實測 +167.0°(近乎反裝)。'),

        # 1. 車體模型 TF + /robot_description(PC 端 RViz 的 RobotModel 會訂閱這個 topic)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2. headless joint_state_publisher(非 GUI),讓四顆輪子 joint 有 TF
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # 3a. RPLidar C1 驅動。參數沿用原本 my_robot_lidar/lidar_start.launch.py 的設定;
        #     serial_port 是 udev/99-rplidar.rules 綁出來的穩定名稱(不是 /dev/ttyUSB*)。
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 460800,   # C1 建議值;不穩可退回 115200
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # 3b. base_link -> laser_frame 外參。arguments 順序是 x y z yaw pitch roll parent child。
        #     值由上面的 laser_* launch arg 餵,現場校準不必改檔也不必 rebuild:
        #       ./run_robot.sh laser_x:=0.02 laser_y:=-0.01
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[laser_x, laser_y, laser_z, laser_yaw, '0', '0',
                       'base_link', 'laser_frame'],
            output='screen',
        ),

        # 4a. use_fake_odom=true:暫時的假里程計 odom->base_link 靜態 TF。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=IfCondition(use_fake_odom),
            output='screen',
        ),

        # 4b. use_fake_odom=false:真的 OminiBotHV 底盤驅動,收 /cmd_vel、發 /odom + odom->base_link TF。
        Node(
            package='ominibot_driver',
            executable='ominibot_driver_node',
            name='ominibot_driver',
            output='screen',
            parameters=[{
                'port': ominibot_port,
                'linear_x_sign': ParameterValue(vx_sign, value_type=float),
                'linear_y_sign': ParameterValue(vy_sign, value_type=float),
                'angular_z_sign': ParameterValue(wz_sign, value_type=float),
                'wheel_diameter_mm': ParameterValue(wheel_diameter_mm, value_type=int),
                'wheel_space_mm': ParameterValue(wheel_space_mm, value_type=int),
                'axle_space_mm': ParameterValue(axle_space_mm, value_type=int),
                'encoder_ppr': ParameterValue(encoder_ppr, value_type=int),
                'gear_ratio': ParameterValue(gear_ratio, value_type=int),
                'motor_pwm_max': ParameterValue(motor_pwm_max, value_type=int),
                'motor_pwm_min': ParameterValue(motor_pwm_min, value_type=int),
                'pos_kp': ParameterValue(pos_kp, value_type=int),
                'pos_ki': ParameterValue(pos_ki, value_type=int),
                'pos_kd': ParameterValue(pos_kd, value_type=int),
                'vel_kp': ParameterValue(vel_kp, value_type=int),
                'vel_ki': ParameterValue(vel_ki, value_type=int),
                'odom_linear_scale': ParameterValue(odom_linear_scale, value_type=float),
                'odom_angular_scale': ParameterValue(odom_angular_scale, value_type=float),
                'cmd_linear_scale': ParameterValue(cmd_linear_scale, value_type=float),
                'cmd_angular_scale': ParameterValue(cmd_angular_scale, value_type=float),
                'motor_direct': ParameterValue(motor_direct, value_type=int),
                'encoder_direct': ParameterValue(encoder_direct, value_type=int),
                'use_gyro_heading': ParameterValue(use_gyro_heading, value_type=bool),
                'gyro_z_sign': ParameterValue(gyro_z_sign, value_type=float),
                'gyro_scale': ParameterValue(gyro_scale, value_type=float),
                'gyro_auto_bias': ParameterValue(gyro_auto_bias, value_type=bool),
                'odom_max_gap': ParameterValue(odom_max_gap, value_type=float),
                'cmd_sync_to_feedback': ParameterValue(cmd_sync_to_feedback, value_type=bool),
                'cmd_vel_timeout': ParameterValue(cmd_vel_timeout, value_type=float),
                'cmd_vel_best_effort': ParameterValue(cmd_vel_best_effort, value_type=bool),
            }],
            condition=UnlessCondition(use_fake_odom),
        ),

        # 5. SLAM Toolbox(async),讀本 repo config/ 的參數
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': False}],
            condition=IfCondition(use_slam),
        ),

        # 6. 車上 SPI OLED 狀態顯示(選配)。訂 /battery_voltage + /odom + /scan,
        #    畫電池電壓/電量條、車體速度、以及 scan/odom 的到達頻率。
        #    比賽規則不准遠端運算,現場沒有筆電可以下 ros2 topic echo,
        #    「電池快沒電」和「光達掛了」只能靠這片螢幕在地板上看出來。
        Node(
            package='ominibot_driver',
            executable='oled_status',
            name='oled_status',
            output='screen',
            parameters=[{
                'controller': ParameterValue(oled_controller, value_type=str),
                'gpio_dc': ParameterValue(oled_dc, value_type=int),
                'gpio_rst': ParameterValue(oled_rst, value_type=int),
                'batt_full_v': ParameterValue(batt_full_v, value_type=float),
                'batt_empty_v': ParameterValue(batt_empty_v, value_type=float),
            }],
            condition=IfCondition(use_oled),
        ),
    ])
