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

用法(PC 端):
  ros2 launch car_assemble_description slam_pc.launch.py
  一鍵版見 repo 根目錄 run_slam.sh。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    slam_config = os.path.join(desc_share, 'config', 'mapper_params_online_async.yaml')

    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=slam_config,
            description='slam_toolbox 參數檔(預設用本 repo config/ 內的副本)'),

        # use_sim_time=False:吃 Pi 送來的真實時間戳(/scan 與 odom TF 都是 Pi 的
        # wall clock,兩台機器時間需大致同步 → 建議 PC 也開 NTP/chrony)。
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': False}],
        ),
    ])
