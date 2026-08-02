#!/usr/bin/env python3
"""比賽任務執行器 —— 依序把機器人送過「起點 -> 中繼區 -> 終點」。

這支就是比賽當天真正要跑的東西,模擬和實機**跑同一份程式**(純 rclpy,免 colcon)。

--- 為什麼迷宮演算法要寫在這一層,而不是寫成 Nav2 的 planner plugin ---

    右手法則 / DFS / flood-fill 這類迷宮演算法解的是「下一格往哪走」,
    前提是「我知道自己在第幾格、格子之間哪裡有牆」—— 那是一個**離散**的問題。

    Nav2 解的是完全不同的問題:給定連續空間的成本地圖和一個目標位姿,
    算出一條不會撞到東西的連續軌跡,並且即時避開臨時出現的障礙。

    把迷宮演算法塞進 planner plugin 等於要它同時做兩件事,而且會失去 Nav2 的
    復原行為(卡住時後退、原地轉、清 costmap)。正確的分層是:

        迷宮演算法(這一層)  決定「下一個目標點在哪」-> NavigateToPose
        Nav2                  負責「怎麼安全地走到那裡」

    notes/nav2_tuning.md 也是這個結論。

--- 為什麼這支目前只送兩個點,而不是跑 flood-fill ---

    因為**比賽規則允許你先掃地圖**(規則三.3:參賽隊伍需於競賽時間內,自行完成
    比賽場地地圖掃描與完成挑戰任務)。地圖既然已經有了,迷宮就不是未知的 ——
    Nav2 的 NavFn/Smac 全域規劃器本來就會在已知地圖上算出最短路徑,那已經是
    最佳解,再套 flood-fill 沒有任何好處。

    flood-fill / 右手法則是給「邊走邊探索、沒有地圖」的情況用的(傳統
    micromouse)。這場比賽不是那個設定。

    真正需要多送幾個點的情況只有一個:規則五.2 規定**沒通過中繼區就抵達終點
    視為失敗**,所以中繼點必須是強制途經點,不能讓 Nav2 直接規劃 起點->終點
    (它會挑最短路,可能繞過中繼區)。這就是這支存在的理由。

用法:
    # 模擬:座標直接從場地檔讀
    python3 tools/run_mission.py --world sim/worlds/tirt_maze.yaml

    # 實機:自己給座標(在 RViz 用 Publish Point 點出來,或從地圖量)
    python3 tools/run_mission.py --waypoint 2.54,2.54,0 --waypoint 3.92,3.92,0

    --dry-run    只印出要去哪裡,不真的送出目標
"""

import argparse
import math
import os
import sys
import time

# rclpy / nav2_msgs 都刻意延遲到真的要用時才匯入 —— --dry-run 只是把路線印出來
# 確認座標對不對,那應該在還沒裝 Nav2 的機器上也能用。
import rclpy
from rclpy.node import Node


class Mission(Node):

    def __init__(self, waypoints, timeout_per_leg):
        super().__init__('tirt_mission')
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import Bool

        self.NavigateToPose = NavigateToPose
        self.waypoints = waypoints
        self.timeout = timeout_per_leg
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # /sim/collision 只有模擬才有。實機上沒有這個 topic,訂閱不會出錯,
        # 只是永遠收不到東西 —— 所以同一份程式兩邊都能跑。
        self.hits = 0
        self.create_subscription(Bool, '/sim/collision', self._hit, 10)

    def _hit(self, msg):
        if msg.data:
            self.hits += 1

    def _pose(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def go(self, name, x, y, yaw):
        """送一個 NavigateToPose 目標並等它結束。回傳 True/False。"""
        self.get_logger().info(f'>>> 前往 {name}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        goal = self.NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{name}: 目標被拒絕 —— Nav2 沒接受這個點')
            return False

        result_future = handle.get_result_async()
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            if time.time() - t0 > self.timeout:
                self.get_logger().error(f'{name}: 超過 {self.timeout}s 還沒到,放棄')
                handle.cancel_goal_async()
                return False

        from action_msgs.msg import GoalStatus
        status = result_future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        dt = time.time() - t0
        if ok:
            self.get_logger().info(f'<<< 抵達 {name},耗時 {dt:.1f}s')
        else:
            # 這裡最常見的失敗是 ABORTED。實機上第一個要查的**不是**規劃器,
            # 而是 costmap 幾何 —— 見 tools/costmap_check.py 和 sim/faults.md #7。
            self.get_logger().error(
                f'{name}: 失敗(status={status},耗時 {dt:.1f}s)。'
                f'先跑 tools/costmap_check.py 看是不是 robot_radius/inflation 太大。')
        return ok


def load_from_world(path):
    """從模擬場地檔讀 start / checkpoint / goal。"""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'sim'))
    from tirt_sim.maze import load_world
    w = load_world(path)
    pts = []
    if w.checkpoint:
        pts.append(('中繼區', *w.checkpoint))
    if w.goal:
        pts.append(('終點', *w.goal))
    if not pts:
        raise SystemExit(f'{path} 裡沒有 checkpoint 也沒有 goal')
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', help='模擬場地 yaml,自動讀出中繼區與終點')
    ap.add_argument('--waypoint', action='append', default=[],
                    metavar='X,Y[,YAW度]', help='手動指定途經點,可重複')
    ap.add_argument('--timeout', type=float, default=120.0,
                    help='每一段的逾時秒數(預設 120)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    if args.waypoint:
        pts = []
        for i, s in enumerate(args.waypoint):
            v = [float(x) for x in s.split(',')]
            yaw = math.radians(v[2]) if len(v) > 2 else 0.0
            pts.append((f'途經點{i+1}', v[0], v[1], yaw))
    elif args.world:
        pts = load_from_world(args.world)
    else:
        ap.error('要嘛給 --world,要嘛給至少一個 --waypoint')

    print('任務路線:')
    for name, x, y, yaw in pts:
        print(f'  {name:8s} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
    if args.dry_run:
        return 0

    rclpy.init()
    node = Mission(pts, args.timeout)

    print('\n等待 Nav2 的 navigate_to_pose action server...')
    if not node.client.wait_for_server(timeout_sec=30.0):
        node.get_logger().error(
            'Nav2 沒有回應。確認 ./run_nav2.sh 已經在跑,而且 lifecycle 都 active:\n'
            '  ros2 lifecycle get /bt_navigator')
        rclpy.shutdown()
        return 1

    t0 = time.time()
    ok = True
    for name, x, y, yaw in pts:
        if not node.go(name, x, y, yaw):
            ok = False
            break

    total = time.time() - t0
    print()
    print('=' * 60)
    print(f' 任務{"完成" if ok else "失敗"}    總耗時 {total:.1f}s')
    if node.hits:
        # 規則五.5:碰到迷宮牆面任一處即當次失敗
        print(f' ★ 撞牆 {node.hits} 次 —— 依規則這一輪不算數,要重新來過')
    print('=' * 60)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok and not node.hits else 1


if __name__ == '__main__':
    sys.exit(main())
