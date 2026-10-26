#!/usr/bin/env python3
"""錄一趟 Nav2 實跑:cmd_vel_nav / cmd_vel / odom / amcl_pose / plan / DWB evaluation / rosout。

用法(Pi 上跑,Nav2 在筆電上跑也行,走 DDS):
    python3 tools/nav_record.py <out.jsonl> <秒數>
開始錄之後再去 RViz 點 2D Pose Estimate + Nav2 Goal。輸出每行一筆
[unix_time, kind, data],kind ∈ nav/cmd/odom/amcl/goal/plan/lplan/eval/log。

⚠ 純 Python 單執行緒,Pi 上會掉樣本(實測 /odom 20 Hz 只收到 ~13 Hz,
  cmd_vel 掉更多)—— 看趨勢可以,不要拿它算頻率或判斷「指令有沒有送到」;
  那個用 tools/cmd_vel_check.py。2026-10-10 的兩趟錄檔在 notes/nav_runs/。
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from dwb_msgs.msg import LocalPlanEvaluation
rclpy.init(); n=rclpy.create_node('navrec')
be=QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
rel=QoSProfile(depth=20)
out=open(sys.argv[1],'w'); t0=None; started=None
def w(kind,d):
    global t0,started
    now=time.time()
    out.write(json.dumps([round(now,3),kind,d])+'\n'); out.flush()
def yaw(q): return math.degrees(2*math.atan2(q.z,q.w))
n.create_subscription(Twist,'/cmd_vel_nav',lambda m: w('nav',[round(m.linear.x,3),round(m.linear.y,3),round(m.angular.z,3)]),be)
n.create_subscription(Twist,'/cmd_vel',lambda m: w('cmd',[round(m.linear.x,3),round(m.linear.y,3),round(m.angular.z,3)]),be)
n.create_subscription(Odometry,'/odom',lambda m: w('odom',[round(m.pose.pose.position.x,3),round(m.pose.pose.position.y,3),round(yaw(m.pose.pose.orientation),1),round(m.twist.twist.linear.x,3),round(m.twist.twist.linear.y,3),round(m.twist.twist.angular.z,3)]),be)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m: w('amcl',[round(m.pose.pose.position.x,3),round(m.pose.pose.position.y,3),round(yaw(m.pose.pose.orientation),1)]),QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(PoseStamped,'/goal_pose',lambda m: w('goal',[round(m.pose.position.x,3),round(m.pose.position.y,3),round(yaw(m.pose.orientation),1)]),rel)
n.create_subscription(Path,'/plan',lambda m: w('plan',[len(m.poses)]+([round(m.poses[-1].pose.position.x,3),round(m.poses[-1].pose.position.y,3)] if m.poses else [])),be)
n.create_subscription(Path,'/local_plan',lambda m: w('lplan',[len(m.poses)]),be)
def ev(m):
    if not m.twists: return
    b=m.twists[m.best_index]
    valid=sum(1 for t in m.twists if t.total>=0)
    w('eval',[len(m.twists),valid,round(b.traj.velocity.x,3),round(b.traj.velocity.y,3),round(b.traj.velocity.theta,3),{s.name:round(s.raw_score*s.scale,2) for s in b.scores}])
n.create_subscription(LocalPlanEvaluation,'/evaluation',ev,rel)
n.create_subscription(Log,'/rosout',lambda m: w('log',[m.name,m.level,m.msg[:200]]) if m.name in ('controller_server','bt_navigator','planner_server','behavior_server','velocity_smoother','amcl') or m.level>=30 else None,QoSProfile(depth=200))
end=time.time()+float(sys.argv[2])
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.05)
