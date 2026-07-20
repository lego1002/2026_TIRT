"""Pi 端 headless 整合啟動檔（給樹莓派跑,不開任何 GUI）。

一次拉起:
  1. robot_state_publisher —— 用真正的 URDF 發布車體/輪子/lidar_link 的 TF 與 /robot_description
  2. joint_state_publisher —— headless(非 GUI)版,把四顆 continuous 輪子 joint 補 0,讓輪子 TF 存在
  3. sllidar_node + base_link->laser_frame 靜態 TF —— 沿用 my_robot_lidar/lidar_start.launch.py
  4. 底盤(預設 use_fake_odom=false):跑 ominibot_driver(收 /cmd_vel、發 /odom + 真 odom->base_link TF);
        use_fake_odom=true → 改發假的 odom->base_link 靜態 TF(無硬體純看模型時用)
  5. slam_toolbox(async)—— 沿用 my_robot_lidar 的 mapper 參數

RViz 一律不在這裡開;請在另一台 Ubuntu PC 上用相同 ROS_DOMAIN_ID 連過來看
(Pi 端一鍵用 repo 根目錄的 run_robot.sh;PC 端一鍵用 run_rviz.sh。
 見 car_assemble_description/rviz/view_robot.rviz 與 repo 內的雙機連線說明)。

用法:
  ros2 launch car_assemble_description robot_bringup.launch.py                       # 預設:真底盤+光達+SLAM
  ros2 launch car_assemble_description robot_bringup.launch.py use_slam:=false       # 只出光達+模型,不建圖
  ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=true   # 沒接底盤,只看模型/光達
  ros2 launch car_assemble_description robot_bringup.launch.py ominibot_port:=/dev/ttyS0    # 底盤改接到別的 UART 時
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    lidar_share = get_package_share_directory('my_robot_lidar')

    urdf_path = os.path.join(desc_share, 'urdf', 'CAR_ASSEMBLE_URDF.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # SLAM 設定檔改放本 repo(car_assemble_description/config),讓 PC 端只靠這個
    # repo 就能跑 slam,不必安裝 my_robot_lidar。Pi 端保留 use_slam:=true 的單機
    # fallback 時也讀同一份,避免兩份 config 漂移。
    slam_config = os.path.join(desc_share, 'config', 'mapper_params_online_async.yaml')

    use_slam = LaunchConfiguration('use_slam')
    use_fake_odom = LaunchConfiguration('use_fake_odom')
    ominibot_port = LaunchConfiguration('ominibot_port')
    vx_sign = LaunchConfiguration('vx_sign')
    vy_sign = LaunchConfiguration('vy_sign')
    wz_sign = LaunchConfiguration('wz_sign')
    wheel_diameter_mm = LaunchConfiguration('wheel_diameter_mm')
    wheel_space_mm = LaunchConfiguration('wheel_space_mm')
    axle_space_mm = LaunchConfiguration('axle_space_mm')
    encoder_ppr = LaunchConfiguration('encoder_ppr')
    gear_ratio = LaunchConfiguration('gear_ratio')
    pos_kp = LaunchConfiguration('pos_kp')
    pos_ki = LaunchConfiguration('pos_ki')
    pos_kd = LaunchConfiguration('pos_kd')
    vel_kp = LaunchConfiguration('vel_kp')
    vel_ki = LaunchConfiguration('vel_ki')
    odom_linear_scale = LaunchConfiguration('odom_linear_scale')
    odom_angular_scale = LaunchConfiguration('odom_angular_scale')
    use_gyro_heading = LaunchConfiguration('use_gyro_heading')
    gyro_z_sign = LaunchConfiguration('gyro_z_sign')
    gyro_scale = LaunchConfiguration('gyro_scale')

    return LaunchDescription([
        # 預設 false:SLAM 已改到 PC 端跑(async scan matching 太吃 CPU,Pi 4 追不上
        # 10Hz 掃描 → queue full 丟 scan → 地圖旋轉塗抹)。PC 端用 run_slam.sh /
        # slam_pc.launch.py。設 use_slam:=true 才會在 Pi 上單機建圖(fallback)。
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='是否在「Pi 本機」啟動 slam(預設 false,SLAM 改在 PC 跑;true=單機 fallback)'),
        DeclareLaunchArgument('use_fake_odom', default_value='false',
                              description='false=跑真底盤 ominibot_driver(預設,一鍵開底盤);true=只發假 odom 靜態 TF(無硬體純看模型時用)'),
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
        # 閉環 PID 增益:原廠為 1:55 重底盤調的,馬達不匹配可能過衝震盪。
        # 可從命令列調小(如 vel_kp:=1500)現場壓振動,不必重新 build。
        DeclareLaunchArgument('pos_kp', default_value='3000', description='位置環 Kp'),
        DeclareLaunchArgument('pos_ki', default_value='1050', description='位置環 Ki'),
        DeclareLaunchArgument('pos_kd', default_value='0', description='位置環 Kd'),
        DeclareLaunchArgument('vel_kp', default_value='3000', description='速度環 Kp'),
        DeclareLaunchArgument('vel_ki', default_value='1050', description='速度環 Ki'),
        # odom 刻度校正:板子回授速度用寫死的內部校正,不吃上面的幾何 config,
        # 實測 odom 灌水 ~5x → 乘 0.2 修回真實單位。這是唯一有效的 odom 校準手段。
        # 直線用 1m 測試(§7.1)、旋轉用 720° 測試(§7.2)校準;設 1.0 看原始輸出。
        DeclareLaunchArgument('odom_linear_scale', default_value='0.16',
                              description='直線速度校正倍率(實測 raw 灌水 ~6.25x → 0.16)。'),
        DeclareLaunchArgument('odom_angular_scale', default_value='0.195',
                              description='輪速 az 的旋轉校正(只在 use_gyro_heading=false 時用)。'),
        # 航向來源:輪速 az 被麥輪打滑毀掉(實測轉 360° 輪速報 2270°),改用板子
        # 原始陀螺儀 Z 積分(實測 360° 準到 ~350°,不受打滑影響)。四元數無磁力計 yaw 凍結不能用。
        DeclareLaunchArgument('use_gyro_heading', default_value='true',
                              description='true=odom 朝向用陀螺儀(麥輪車正解);false=退回輪速 az。'),
        DeclareLaunchArgument('gyro_z_sign', default_value='1.0',
                              description='陀螺 Z 正負號(odom 轉向反了就設 -1.0)。'),
        DeclareLaunchArgument('gyro_scale', default_value='1.0',
                              description='陀螺積分倍率(實測 ~1.0,需要再微調)。'),

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

        # 3. 光達驅動 + base_link->laser_frame 靜態 TF(沿用你現有的檔案)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_share, 'launch', 'lidar_start.launch.py')
            )
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
                'pos_kp': ParameterValue(pos_kp, value_type=int),
                'pos_ki': ParameterValue(pos_ki, value_type=int),
                'pos_kd': ParameterValue(pos_kd, value_type=int),
                'vel_kp': ParameterValue(vel_kp, value_type=int),
                'vel_ki': ParameterValue(vel_ki, value_type=int),
                'odom_linear_scale': ParameterValue(odom_linear_scale, value_type=float),
                'odom_angular_scale': ParameterValue(odom_angular_scale, value_type=float),
                'use_gyro_heading': ParameterValue(use_gyro_heading, value_type=bool),
                'gyro_z_sign': ParameterValue(gyro_z_sign, value_type=float),
                'gyro_scale': ParameterValue(gyro_scale, value_type=float),
            }],
            condition=UnlessCondition(use_fake_odom),
        ),

        # 5. SLAM Toolbox(async),沿用 my_robot_lidar 的參數
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': False}],
            condition=IfCondition(use_slam),
        ),
    ])
