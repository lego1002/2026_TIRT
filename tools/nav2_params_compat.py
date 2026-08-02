#!/usr/bin/env python3
"""把「以 ROS 2 humble 為準」的 nav2_params.yaml 改寫成新發行版(jazzy 之後)能吃的版本。

    python3 tools/nav2_params_compat.py <in.yaml> <out.yaml>

為什麼要有這支
--------------
比賽的實機(Pi 和比賽筆電)是 **humble**,所以 `config/nav2_params.yaml` 必須維持
humble 正確 —— 那份檔案是唯一的真實來源,不該為了開發機而改壞。但開發/模擬機可能是
jazzy,而 Nav2 在 humble → jazzy 之間有幾個**不相容且錯誤訊息會騙人**的變更。
`run_nav2.sh` 偵測到本機不是 humble 形式時,會呼叫這支產生一份暫存參數檔,原檔不動。

刻意用純文字改寫而不是 yaml.load/dump:那份 yaml 有大量說明為什麼要這樣調的中文
註解,round-trip 會把註解全部吃掉,而註解正是那個檔案最有價值的部分。

處理的三件事(都是實際踩過的)
------------------------------
0. 補上 jazzy 才有、humble 沒有的節點設定(collision_monitor / docking_server)。
   jazzy 的 navigation_launch.py 把這兩個放進 lifecycle 清單,而 humble 根本沒有
   (collision_monitor 在 humble 是另一支 launch、預設不啟動;docking_server 不存在)。
   缺設定的症狀:
     ERROR [collision_monitor]: parameter 'observation_sources' is not initialized
     ERROR [docking_server]: Charging dock plugins not given!
   刻意放在這支而不是加進 config/nav2_params.yaml:實機是 humble,那兩段在實機上
   永遠不會被讀到,不該去汙染比賽用的參數檔。
1. pluginlib 名稱 `pkg/ClassName` -> `pkg::ClassName`。
   症狀:planner_server 一 configure 就 FATAL,
     "the class nav2_navfn_planner/NavfnPlanner ... does not exist.
      Declared types are nav2_navfn_planner::NavfnPlanner ..."
   影響 navfn(1 個)和 behaviors(spin/backup/drive_on_heading/wait/assisted_teleop)。
   其餘 nav2_costmap_2d:: / nav2_controller:: / dwb_core:: 等本來就是 "::",兩邊通用。

2. 拿掉 bt_navigator 的 `plugin_lib_names` 整段。
   症狀:bt_navigator FATAL "Failed to create navigator id navigate_to_pose.
         Exception: ID [ComputePathToPose] already registered"
   humble 需要這份完整註冊表(少一個載入 BT XML 就炸);jazzy 改成內建節點**自動註冊**,
   這個參數只留給「額外的自訂 BT plugin」,再列一次內建的就變成重複註冊。

為什麼這些症狀難認
------------------
lifecycle_manager 是「一個節點失敗就整包中止」,但 map_server 和 amcl 在它前面、
已經 active —— 所以 RViz 裡地圖看得到、粒子雲也在,看起來只是「導航按鈕沒反應」,
而 RViz 唯一的訊息是 "navigate_to_pose action server is not available.
Is the initial pose set?",會把人整個引去反覆重設 2D Pose Estimate。真正的原因和
初始位置毫無關係。要看真正的錯誤只能翻 nav2 那個視窗的 FATAL 行。
"""
import re
import sys


# jazzy 的 navigation_launch.py 比 humble 多了 route_server / collision_monitor /
# docking_server 三個 lifecycle 節點。route_server 全用預設值就能 configure,另外兩個
# 沒設定會直接失敗 —— 而 lifecycle_manager 是「一個失敗就整包中止」。
JAZZY_EXTRA_NODES = """

# ===========================================================================
# 以下由 tools/nav2_params_compat.py 自動補上,不在 config/nav2_params.yaml 裡。
# 這些節點只有 jazzy 之後的 nav2_bringup 才會啟動;實機的 humble 沒有它們。
# ===========================================================================

collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_link"        # 這台 URDF 沒有 base_footprint
    odom_frame_id: "odom"
    # 注意 jazzy 的 /cmd_vel 鏈是
    #   controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
    #   -> collision_monitor -> /cmd_vel
    # 最後發 /cmd_vel 的是 collision_monitor,不是 velocity_smoother(humble 是後者)。
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    base_shift_correction: True
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      visualize: False
      enabled: True
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
      # 原版 min_height 0.15 是拿來濾地面回波的。這台光達裝在 0.109m、迷宮牆只有 20cm
      # 高,0.15 會把牆整片濾掉。2D 光達本來就沒有高度資訊,所以放寬到全收。
      min_height: -1.0
      max_height: 2.0
      enabled: True

# 這台沒有充電座,也永遠不會用到自動對接。這段純粹是為了讓 docking_server 能通過
# configure —— 少了 dock_plugins 它會 "Charging dock plugins not given!" 而讓整包
# bringup 中止。內容照 nav2_bringup 原版,不必調。
docking_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 50.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 5.0
    dock_approach_timeout: 30.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.1
    max_retries: 3
    base_frame: "base_link"
    fixed_frame: "odom"
    dock_backwards: false
    dock_prestaging_tolerance: 0.5
    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05
      staging_x_offset: -0.7
      use_external_detection_pose: true
      use_battery_status: false
      use_stall_detection: false
      external_detection_timeout: 1.0
      external_detection_translation_x: -0.18
      external_detection_translation_y: 0.0
      external_detection_rotation_roll: -1.57
      external_detection_rotation_pitch: -1.57
      external_detection_rotation_yaw: 0.0
      filter_coef: 0.1
    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.15
      v_linear_max: 0.15
      use_collision_detection: true
      costmap_topic: "local_costmap/costmap_raw"
      footprint_topic: "local_costmap/published_footprint"
      transform_tolerance: 0.1
      projection_time: 5.0
      simulation_step: 0.1
      dock_collision_threshold: 0.3
"""


def convert(text):
    notes = []

    # 1) pkg/ClassName -> pkg::ClassName。只比對「nav2_ 開頭的套件名 / 大寫開頭的類別名」
    #    這個形狀,所以路徑(全小寫、含 . 或 /)不會被誤傷。
    text, n = re.subn(r'\b(nav2_[a-z0-9_]*)/([A-Z][A-Za-z0-9_]*)', r'\1::\2', text)
    if n:
        notes.append(f'plugin 名稱 / -> :: 共 {n} 處')

    # 2) 註解掉 plugin_lib_names 及其底下的清單項。用註解而不是刪除,這樣萬一有人去看
    #    產生出來的暫存檔,還看得出來原本有什麼、以及為什麼被拿掉。
    out, killed, in_block = [], 0, False
    for line in text.splitlines(True):
        if in_block:
            # 清單項(或空行)還算在這個 block 裡;遇到別的 key 就結束。
            if re.match(r'\s+-\s', line):
                out.append('#[compat] ' + line)
                killed += 1
                continue
            in_block = False
        if re.match(r'\s*plugin_lib_names:\s*$', line):
            out.append('#[compat] jazzy 之後內建 BT 節點會自動註冊,再列一次會 '
                       '"ID [...] already registered"\n')
            out.append('#[compat] ' + line)
            in_block = True
            continue
        out.append(line)
    if killed:
        notes.append(f'註解掉 bt_navigator plugin_lib_names({killed} 項)')
    text = ''.join(out)

    # 0) 補 jazzy 才有的節點。已經有就不重複補(方便有人哪天把它加進主檔)。
    missing = [k for k in ('collision_monitor', 'docking_server')
               if not re.search(r'^%s:' % k, text, re.M)]
    if missing:
        text += JAZZY_EXTRA_NODES
        notes.append('補上 ' + '、'.join(missing))

    return text, notes


def main():
    if len(sys.argv) != 3:
        print(__doc__.strip().splitlines()[2].strip(), file=sys.stderr)
        return 2
    src, dst = sys.argv[1], sys.argv[2]
    with open(src, encoding='utf-8') as f:
        text = f.read()
    converted, notes = convert(text)
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(converted)
    print('nav2_params_compat: ' + ('、'.join(notes) if notes else '沒有需要改的地方'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
