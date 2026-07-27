"""PC 端 SLAM 啟動檔（在強壯的 Ubuntu PC 上跑,不在樹莓派上）。

為什麼 SLAM 搬到 PC:
  async_slam_toolbox 的 scan matching 很吃 CPU。Pi 4 同時扛 robot_state_publisher
  + joint_state_publisher + 光達 + ominibot_driver + SLAM 時算不過來,追不上 10Hz
  的 /scan → tf message filter 佇列塞爆、丟 scan(log 一直出現 "queue is full")→
  scan matching 失效 → 只能靠會飄的 odom 硬推 → 地圖被畫成一圈圈旋轉塗抹(fan smear)。
  把 SLAM 丟給閒著的 PC 跑,scan matching 有充足算力、不再丟 scan,航向就修得動。

資料怎麼流(全靠 DDS,兩台同 ROS_DOMAIN_ID + 同 LAN whitelist):
  Pi  發:/scan、odom->base_link TF、base_link->laser_frame TF、URDF 各 joint TF
  PC  收 /scan + 查 odom->base_link,做 scan matching,發 map->odom TF + /map
  → RViz(也在 PC)看得到完整 map->odom->base_link->laser_frame 與 /map。
  只有 /scan + 小小的 TF 過網路(都很輕);大張的 /map 在 PC 就地產生,不必再從
  Pi 傳過來,反而比原本省網路。

前提:PC 要先裝 slam_toolbox → sudo apt install ros-humble-slam-toolbox
      並且 build 過本 package(config/ 內含 slam 參數檔)。

RViz 也在這裡一起開(2026-07-27 併進來,預設 use_rviz:=true):
  以前 SLAM 和 RViz 是兩個指令、兩個終端,但這兩件事在操作上從來不會分開 ——
  要看地圖長出來就一定同時需要。而且 rviz/view_robot.rviz 已經把 Grid / RobotModel /
  LaserScan / Map(Durability 設成 Transient Local,才收得到 latch 的地圖)/ Odometry /
  TF 全部設好、Fixed Frame = map,所以是直接載入即用,不需要現場手動加 display。

  ※ Fixed Frame 是 map,而 map frame 要等 slam_toolbox 收到頭幾張 scan(約 10-15 秒)
    才會出現。這段期間 RViz 會顯示 "Fixed Frame does not exist" 一片空白,是正常的,
    等一下就好;真的想先看車體就把 Fixed Frame 暫時改成 base_link。

用法(PC 端):
  ros2 launch car_assemble_description slam_pc.launch.py
  ros2 launch car_assemble_description slam_pc.launch.py use_rviz:=false   # 只跑 SLAM
  一鍵版見 repo 根目錄 run_slam.sh(gcs.sh 會呼叫它)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    slam_config = os.path.join(desc_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'view_robot.rviz')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=slam_config,
            description='slam_toolbox 參數檔(預設用本 repo config/ 內的副本)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否一併開 RViz(預設 true;只想跑 SLAM 不看畫面時設 false)'),
        DeclareLaunchArgument(
            'rviz_config_file', default_value=rviz_config,
            description='RViz 設定檔(預設本 repo rviz/view_robot.rviz,已設好所有 display)'),

        # use_sim_time=False:吃 Pi 送來的真實時間戳(/scan 與 odom TF 都是 Pi 的
        # wall clock,兩台機器時間需大致同步 → 建議 PC 也開 NTP/chrony)。
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': False}],
        ),

        # RViz。關掉 RViz 視窗不會連帶關掉 SLAM(沒有設 on_exit),所以看完可以直接關,
        # 地圖還在繼續長;要再看一次就單獨跑 ./run_rviz.sh。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])
