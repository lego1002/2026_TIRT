"""PC 端 Nav2 自主導航啟動檔（在 Ubuntu PC 上跑,不在樹莓派上）。

和 slam_pc.launch.py 是「二選一」的關係,不能同時跑:
  slam_pc.launch.py  → slam_toolbox 邊走邊建圖,自己發 map->odom TF
  nav2_pc.launch.py  → AMCL 讀已存好的地圖做定位,自己發 map->odom TF
兩個都開的話會有兩個節點搶著發同一條 map->odom TF,TF tree 直接壞掉,RViz 裡車子
會鬼影亂跳。run_nav2.sh 會先 pkill 掉 slam_toolbox 就是為了這件事。

為什麼跑在 PC 而不是 Pi(和 SLAM 同一個理由,只是更嚴重):
  Nav2 一次要跑 global costmap + local costmap(各自 5-10Hz 更新)+ MPPI
  局部規劃器(每個 control cycle 取樣 1000 條軌跡)+ AMCL 粒子濾波。這比
  slam_toolbox 還吃 CPU,Pi 4 上還要同時餵光達和 ominibot_driver,不可能撐得住。
  Pi 只負責發 /scan、/odom、TF,收 /cmd_vel。

資料怎麼流:
  Pi → PC:  /scan、odom->base_link TF、base_link->laser_frame TF、URDF 的 joint TF
  PC → Pi:  /cmd_vel(nav2 的 velocity_smoother 發出來的最終速度指令)
  PC 內部:  /map(map_server)、map->odom TF(AMCL)、costmap、path

  ※ /cmd_vel 這次是「反方向」跨 WiFi 的 —— 這正是當初把鍵盤 teleop 搬到 Pi 上要
    避開的路徑(見 CLAUDE.md「Where teleop runs, and why」)。目前的防護是
    driver 的 cmd_vel_timeout 已經放寬到 1.0s、QoS 是 BEST_EFFORT depth-1,
    外加 velocity_smoother 以固定 20Hz 持續發送。真的頓,就用
    tools/cmd_vel_check.py 在 Pi 上量 /cmd_vel 的到達間隔,別憑感覺調。

※ 開跑之前一定要先關掉 Pi 上的鍵盤 teleop:`./robotctl down teleop`。
  teleop_node 是「每個迴圈都重發目前的 Twist」的設計(為了餵 driver 的 watchdog),
  所以它閒著沒按鍵時是以 20Hz 持續發零速度。和 nav2 的指令交錯進來的結果就是
  車子抽一下停一下、幾乎走不動 —— 而且看起來會很像硬體故障,很難查。

前提(PC 端):
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  colcon build --packages-select car_assemble_description   # 取得 config/nav2_params.yaml

用法(PC 端,建議直接用 repo 根目錄的 run_nav2.sh):
  ros2 launch car_assemble_description nav2_pc.launch.py map:=/絕對路徑/201_self_test.yaml
  ros2 launch car_assemble_description nav2_pc.launch.py map:=... use_rviz:=false

  地圖檔刻意留在 repo 的 maps/ 目錄(package share 之外),用絕對路徑傳進來 ——
  這樣每存一張新地圖都不必重跑 colcon build。所以 map: 沒有預設值。

RViz 裡怎麼下指令:
  1. 車子位置對嗎?不對就用工具列的 "2D Pose Estimate" 點在車子真實位置、拖出朝向。
     (地圖是從起跑點建的,所以把車放回起跑點開機的話,params 裡的 set_initial_pose
      已經幫你對好了,這步可以跳過。)
  2. 用 "Nav2 Goal" 工具在地圖上點一個目標點並拖出朝向 → 車子就會自己走過去。
  3. 左下角 Navigation 2 面板可以看狀態、按 Cancel 中止。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _check_map(context, *args, **kwargs):
    """map: 是必填的。與其讓 map_server 在 activate 時丟一個很難讀的錯,不如在這裡
    就把話講清楚(launch 的參數是延遲求值的,所以要用 OpaqueFunction 才拿得到值）。"""
    map_file = LaunchConfiguration('map').perform(context)
    if not map_file:
        raise RuntimeError(
            'nav2_pc.launch.py 需要 map:=<地圖 yaml 的絕對路徑>。\n'
            '例如:ros2 launch car_assemble_description nav2_pc.launch.py '
            'map:=$PWD/maps/201_self_test.yaml\n'
            '(或直接用 repo 根目錄的 ./run_nav2.sh 201_self_test)')
    if not os.path.isfile(map_file):
        raise RuntimeError(f'找不到地圖檔:{map_file}')
    return []


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    nav2_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(desc_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(desc_share, 'rviz', 'view_nav2.rviz')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='地圖 yaml 的絕對路徑(必填;地圖不在 package share 裡)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 參數檔(預設用本 repo config/nav2_params.yaml)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否一併開 RViz(預設 true)'),
        DeclareLaunchArgument(
            'rviz_config_file', default_value=default_rviz,
            description='RViz 設定檔(預設 rviz/view_nav2.rviz,含 Nav2 面板與工具)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='是否自動把所有 lifecycle 節點推到 active'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='把 nav2 節點裝進單一 container(省記憶體、少 IPC 開銷)'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='nav2 節點的 log 等級(除錯時設 debug)'),

        OpaqueFunction(function=_check_map),

        # nav2_bringup 的 bringup_launch.py 已經把 localization(map_server + amcl)
        # 和 navigation(controller / planner / behaviors / bt_navigator /
        # velocity_smoother + 兩個 lifecycle_manager)包好了,沒必要自己重寫一份 ——
        # 我們要客製的東西全部在 params_file 裡。
        #
        # slam:=False → 用 AMCL 讀存好的地圖,不是邊走邊建圖。
        # use_sim_time:=false → 吃 Pi 的 wall clock(兩台機器時鐘要大致同步,建議都開 chrony)。
        #
        # 值得記住的 topic remap(bringup 內建,不是我們設的):controller_server 其實
        # 發到 /cmd_vel_nav,velocity_smoother 收它、平滑之後才發成 /cmd_vel。所以要
        # 抓「最後真的送到車上的指令」是看 /cmd_vel,要抓「規劃器原始輸出」是 /cmd_vel_nav。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': 'False',
                'slam': 'False',
                'log_level': log_level,
            }.items(),
        ),

        # RViz。關掉視窗不會把 nav2 一起關掉(沒設 on_exit),要再開就跑 ./run_rviz.sh
        # ——不過那支載的是 SLAM 用的 view_robot.rviz(沒有 Nav2 面板);要完整的導航
        # 介面就 rviz2 -d car_assemble_description/rviz/view_nav2.rviz。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_rviz),
        ),
    ])
