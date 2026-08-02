"""模擬版 bringup —— 對應真車的 car_assemble_description/robot_bringup.launch.py。

兩邊刻意做成**逐項對應**,你在模擬裡熟悉的每一個東西,實機上都有對照:

    真車(Pi 上跑)                     模擬(筆電上跑)
    ------------------------------     ------------------------------
    robot_state_publisher              robot_state_publisher      <- 同一個,同一份 URDF
    joint_state_publisher              joint_state_publisher      <- 同一個
    sllidar_node        (RPLidar C1)   fake_lidar                 <- 發一樣的 /scan
    static_tf base->laser              static_tf base->laser      <- 同一個,同樣的 laser_* 參數
    ominibot_driver     (序列埠底盤)   fake_base                  <- 收一樣的 /cmd_vel,發一樣的 /odom + TF

因為介面一致,上層(slam_toolbox / Nav2 / RViz / teleop)完全分辨不出差別,
run_slam.sh、run_nav2.sh、save_map.sh 一個字都不用改就能用。

--- 兩組光達參數的差別(這是本檔最重要的觀念)---

    laser_x / laser_y / laser_yaw          你**以為**光達裝在哪 -> 發成 TF,SLAM 會用
    sim_laser_x / sim_laser_y / sim_laser_yaw   光達**實際**裝在哪 -> 模擬器拿來產生 /scan

真車上第一組是你在 launch 檔填的數字,第二組是物理現實(你只能量,不能改)。
兩者不一致時 SLAM 會安靜地畫出爛地圖,沒有任何錯誤訊息 —— 這正是 2026-07-25
那個「光達幾乎反裝、launch 檔卻填 0」的 bug。預設兩組相同(= 已校準的車);
把它們設成不同就能重現該 bug,處方見 sim/faults.md。

用法:
    ./sim/run_sim.sh                                  # 預設場地,無故障
    ./sim/run_sim.sh world:=sim/worlds/simple.yaml    # 換小場地
    ./sim/run_sim.sh sim_laser_yaw:=2.9146            # 重現光達反裝
    ./sim/run_sim.sh odom_linear_error:=1.3           # 重現里程計刻度沒校準
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    sim_share = get_package_share_directory('tirt_sim')

    # 用真車同一份 URDF —— 車體尺寸、輪距、光達掛載點都是真的,只有物理是假的。
    urdf_path = os.path.join(desc_share, 'urdf', 'CAR_ASSEMBLE_URDF.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    slam_config = os.path.join(desc_share, 'config', 'mapper_params_online_async.yaml')
    default_world = os.path.join(sim_share, 'worlds', 'tirt_maze.yaml')

    world = LaunchConfiguration('world')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    use_slam = LaunchConfiguration('use_slam')

    def f(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='迷宮場地 yaml。sim/worlds/ 下面有 tirt_maze(9x9 比賽場)和 simple(3x3 除錯用)。'),
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='是否在這裡一起開 SLAM。預設 false —— 和真車一樣用另一個視窗跑 run_slam.sh,保持流程一致。'),

        # --- 光達外參:你「以為」的(發成 TF,SLAM 用這個)---
        # 真車的預設是 (0.014, -0.014, 0.109, 2.9146),其中 yaw 的 167° 是那台車
        # 光達實際反裝造成的。模擬的車是「裝正的」,所以這裡預設 0。
        DeclareLaunchArgument('laser_x', default_value='0.014'),
        DeclareLaunchArgument('laser_y', default_value='-0.014'),
        DeclareLaunchArgument('laser_z', default_value='0.109'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0',
                              description='TF 上的光達朝向(rad)。和 sim_laser_yaw 不一致 = 重現外參 bug。'),

        # --- 光達外參:「實際」的(模擬器產生 scan 用)---
        DeclareLaunchArgument('sim_laser_x', default_value='0.014'),
        DeclareLaunchArgument('sim_laser_y', default_value='-0.014'),
        DeclareLaunchArgument('sim_laser_yaw', default_value='0.0',
                              description='光達實際裝的朝向(rad)。物理現實,真車上你只能量不能改。'),

        # --- 光達感測特性(對齊 RPLidar C1)---
        DeclareLaunchArgument('scan_rate', default_value='10.0'),
        DeclareLaunchArgument('scan_samples', default_value='450'),
        DeclareLaunchArgument('range_noise', default_value='0.005',
                              description='每點測距高斯雜訊(m)。C1 約 ±0.5cm。'),
        DeclareLaunchArgument('dropout_prob', default_value='0.01',
                              description='無回波比例。白色高反射牆面偶爾會這樣。'),

        # --- 底盤:故障注入(詳細處方見 sim/faults.md)---
        DeclareLaunchArgument('odom_linear_error', default_value='1.0',
                              description='里程計直線刻度誤差。1.0=完美,1.3=多報 30%。'),
        DeclareLaunchArgument('odom_angular_error', default_value='1.0',
                              description='里程計角度刻度誤差。'),
        DeclareLaunchArgument('odom_drift_deg_per_m', default_value='0.0',
                              description='每走 1m 累積的航向偏差(度)。'),
        DeclareLaunchArgument('odom_noise_xy', default_value='0.0'),
        DeclareLaunchArgument('odom_noise_yaw', default_value='0.0'),
        DeclareLaunchArgument('slip_factor', default_value='1.0',
                              description='麥輪打滑:實際位移 / 命令位移。'),
        DeclareLaunchArgument('slip_on_accel', default_value='0.0',
                              description='加速度造成的額外打滑係數。'),
        DeclareLaunchArgument('cmd_dropout_prob', default_value='0.0',
                              description='/cmd_vel 掉包率(模擬 WiFi)。'),
        DeclareLaunchArgument('cmd_stall_prob', default_value='0.0',
                              description='每週期進入長卡頓的機率(模擬 WiFi 突發停頓)。'),
        DeclareLaunchArgument('cmd_stall_sec', default_value='1.5'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='1.0',
                              description='watchdog:多久沒收到指令就歸零。和真 driver 同名同義。'),
        DeclareLaunchArgument('max_accel', default_value='1.5',
                              description='加速度上限,對齊 nav2_params.yaml 的 acc_lim_x。'),
        DeclareLaunchArgument('robot_radius', default_value='0.075',
                              description='車體實體外接圓半徑,用來判定撞牆(規則:碰牆即當次失敗)。'),
        DeclareLaunchArgument('world_tf_mode', default_value='identity',
                              description='map->sim_world 怎麼接:identity(跑 Nav2 用產生的地圖)/ start(跑 SLAM)/ off。'),

        # 1. 車體模型 TF + /robot_description(和真車完全一樣的節點)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2. headless joint_state_publisher(和真車完全一樣)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # 3a. 假光達 —— 取代真車的 sllidar_node
        Node(
            package='tirt_sim',
            executable='fake_lidar',
            name='fake_lidar',
            output='screen',
            parameters=[{
                'world': world,
                'frame_id': 'laser_frame',
                'rate': f('scan_rate'),
                'samples': ParameterValue(LaunchConfiguration('scan_samples'), value_type=int),
                'sim_laser_x': f('sim_laser_x'),
                'sim_laser_y': f('sim_laser_y'),
                'sim_laser_yaw': f('sim_laser_yaw'),
                'range_noise': f('range_noise'),
                'dropout_prob': f('dropout_prob'),
            }],
        ),

        # 3b. base_link -> laser_frame 外參。和真車同一個節點、同樣的參數名。
        #     注意餵進去的是 laser_*(你以為的),不是 sim_laser_*(實際的)。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[laser_x, laser_y, laser_z, laser_yaw, '0', '0',
                       'base_link', 'laser_frame'],
            output='screen',
        ),

        # 4. 假底盤 —— 取代真車的 ominibot_driver
        Node(
            package='tirt_sim',
            executable='fake_base',
            name='fake_base',
            output='screen',
            parameters=[{
                'world': world,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'cmd_vel_timeout': f('cmd_vel_timeout'),
                'max_accel': f('max_accel'),
                'robot_radius': f('robot_radius'),
                'odom_linear_error': f('odom_linear_error'),
                'odom_angular_error': f('odom_angular_error'),
                'odom_drift_deg_per_m': f('odom_drift_deg_per_m'),
                'odom_noise_xy': f('odom_noise_xy'),
                'odom_noise_yaw': f('odom_noise_yaw'),
                'slip_factor': f('slip_factor'),
                'slip_on_accel': f('slip_on_accel'),
                'cmd_dropout_prob': f('cmd_dropout_prob'),
                'cmd_stall_prob': f('cmd_stall_prob'),
                'cmd_stall_sec': f('cmd_stall_sec'),
                'world_tf_mode': LaunchConfiguration('world_tf_mode'),
            }],
        ),

        # 5. SLAM(預設不開,和真車一致 —— 用另一個視窗跑 run_slam.sh)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': False}],
            condition=IfCondition(use_slam),
        ),
    ])
