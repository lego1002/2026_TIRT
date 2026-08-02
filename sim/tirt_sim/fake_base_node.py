"""假底盤 —— 模擬版的 ominibot_driver。

對外介面和真的 ominibot_driver **逐項相同**,這是整個模擬環境的重點:

    訂閱  /cmd_vel               geometry_msgs/Twist,BEST_EFFORT depth 1
    發布  /odom                  nav_msgs/Odometry @ 20Hz
    廣播  odom -> base_link      TF
    參數  cmd_vel_timeout        沒收到指令就把底盤歸零(watchdog)

因為介面一樣,上層的 slam_toolbox / Nav2 / RViz / teleop **完全不知道自己
接的是模擬還是真車**,run_slam.sh 和 run_nav2.sh 一個字都不用改。這樣在
模擬裡學到的操作流程和除錯手法,比賽當天可以原封不動地用。

除了真車有的東西之外,它多發三個「上帝視角」的 topic —— 真車上不可能有,
但正是拿來學習與除錯的關鍵:

    /sim/ground_truth   真實位姿(odom 是「車以為自己在哪」,這個是「實際在哪」)
    /sim/odom_error     兩者的差(dx, dy, dyaw度)—— 里程計漂移一眼就看得到
    /sim/collision      有沒有撞牆(規則:碰到牆面任一處即當次失敗)

--- 為什麼是假物理 ---

刻意不算輪子接觸力、馬達扭矩、電池垂降。那些東西模擬得再像也不會跟實機一樣,
反而給人「模擬過了就沒問題」的錯覺。這裡只保留**會改變上層行為**的那幾件事:
加速度上限、里程計刻度誤差、麥輪打滑、指令延遲/掉包、光達雜訊。這些才是
會讓 SLAM 畫爛地圖、讓 Nav2 走歪的原因,也是 notes/ 裡那些歷史 bug 的成因。
"""

import math
import random

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from tirt_sim.maze import load_world


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class FakeBase(Node):

    def __init__(self):
        super().__init__('fake_base')

        # --- 和真 driver 同名同義的參數 ---
        self.declare_parameter('world', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('cmd_vel_best_effort', True)
        self.declare_parameter('rate', 20.0)          # 真板子的回授約 20Hz

        # --- 車體 ---
        # 規則:機器人高度 <20cm,長寬不限。這台約 15cm,取外接圓半徑 0.075。
        # 這個值決定「撞牆」的判定,和 nav2_params.yaml 的 robot_radius 是
        # 兩回事(那個是規劃時的安全半徑,通常會設得比實體大)。
        self.declare_parameter('robot_radius', 0.075)
        self.declare_parameter('max_accel', 1.5)      # 對齊 nav2_params 的 acc_lim
        self.declare_parameter('max_accel_theta', 3.2)

        # --- 故障注入:里程計 ---
        # 1.0 = 完美。真車上 odom_linear_scale 沒校準時就是這裡不等於 1。
        self.declare_parameter('odom_linear_error', 1.0)
        self.declare_parameter('odom_angular_error', 1.0)
        # 每走 1 公尺累積的航向偏差(度)。真車上輪徑不等、地面不平就會這樣。
        self.declare_parameter('odom_drift_deg_per_m', 0.0)
        self.declare_parameter('odom_noise_xy', 0.0)      # 每步高斯雜訊(m)
        self.declare_parameter('odom_noise_yaw', 0.0)     # 每步高斯雜訊(rad)

        # --- 故障注入:麥輪打滑 ---
        # 實際走的距離 = 指令的 slip_factor 倍,但 odom 仍照指令算 ->
        # 這正是麥輪車最典型的失準來源(CLAUDE.md 記載:真的轉 360°,輪速卻報 2270°)。
        self.declare_parameter('slip_factor', 1.0)
        self.declare_parameter('slip_on_accel', 0.0)      # 加速度越大打滑越多

        # --- 故障注入:指令通道(重現「一頓一頓」)---
        self.declare_parameter('cmd_dropout_prob', 0.0)   # 每筆指令被丟掉的機率
        self.declare_parameter('cmd_stall_prob', 0.0)     # 每個週期進入長卡頓的機率
        self.declare_parameter('cmd_stall_sec', 1.5)      # 卡頓持續多久

        # --- 真值 TF ---
        # map -> sim_world 要怎麼接:
        #   identity = map 原點就是世界原點(跑 Nav2 + make_map.py 產生的地圖時)
        #   start    = map 原點在車子起始位置(跑 SLAM 時,slam_toolbox 就是這樣定義的)
        #   off      = 不發,真值只看 topic
        self.declare_parameter('world_tf_mode', 'identity')

        p = self.get_parameter
        world_path = p('world').value
        if not world_path:
            raise RuntimeError('fake_base 需要 world:=<迷宮 yaml 路徑>')
        self.world = load_world(world_path)

        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.timeout = p('cmd_vel_timeout').value
        self.rate = p('rate').value
        self.radius = p('robot_radius').value
        self.a_max = p('max_accel').value
        self.a_max_th = p('max_accel_theta').value
        self.e_lin = p('odom_linear_error').value
        self.e_ang = p('odom_angular_error').value
        self.drift = math.radians(p('odom_drift_deg_per_m').value)
        self.n_xy = p('odom_noise_xy').value
        self.n_yaw = p('odom_noise_yaw').value
        self.slip = p('slip_factor').value
        self.slip_a = p('slip_on_accel').value
        self.drop_p = p('cmd_dropout_prob').value
        self.stall_p = p('cmd_stall_prob').value
        self.stall_s = p('cmd_stall_sec').value

        # 真實位姿(上帝視角)與里程計位姿(車以為的)分開存 —— 這是整個模擬
        # 最重要的一行:真車上你永遠只看得到後者,所以永遠不知道自己錯多少。
        self.tx, self.ty, self.tyaw = self.world.start
        self.ox, self.oy, self.oyaw = 0.0, 0.0, 0.0

        self.cmd = Twist()
        self.cmd_time = self.get_clock().now()
        self.vx = self.vy = self.wz = 0.0        # 目前實際速度(受加速度限制)
        self.stall_until = None
        self.hit_count = 0
        self.was_hit = False

        cmd_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if p('cmd_vel_best_effort').value
                         else ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, cmd_qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        self.err_pub = self.create_publisher(Vector3, '/sim/odom_error', 10)
        self.hit_pub = self.create_publisher(Bool, '/sim/collision', 10)

        self.tf = TransformBroadcaster(self)
        self._publish_world_tf(p('world_tf_mode').value)

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self._step)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'假底盤啟動 | 場地「{self.world.name}」'
            f' {self.world.width:.2f}x{self.world.height:.2f}m'
            f' | 起點 ({self.tx:.2f}, {self.ty:.2f}, {math.degrees(self.tyaw):.0f}°)')
        faults = self._active_faults()
        if faults:
            self.get_logger().warn('故障注入啟用中: ' + ', '.join(faults))

    def _active_faults(self):
        out = []
        if self.e_lin != 1.0:
            out.append(f'odom 直線刻度 x{self.e_lin}')
        if self.e_ang != 1.0:
            out.append(f'odom 角度刻度 x{self.e_ang}')
        if self.drift:
            out.append(f'航向漂移 {math.degrees(self.drift):.1f}°/m')
        if self.slip != 1.0:
            out.append(f'打滑 x{self.slip}')
        if self.drop_p:
            out.append(f'指令掉包 {self.drop_p:.0%}')
        if self.stall_p:
            out.append(f'指令卡頓 {self.stall_p:.1%}/週期 x{self.stall_s}s')
        if self.n_xy or self.n_yaw:
            out.append('odom 雜訊')
        return out

    def _publish_world_tf(self, mode):
        """map -> sim_world。讓 RViz 能在同一張圖上同時顯示真值和估計值。

        TF 允許一個 frame 有很多子節點,所以這條和 SLAM/AMCL 發的 map->odom
        不會打架(它們的子節點是 odom,這條是 sim_world)。
        """
        if mode == 'off':
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'sim_world'
        if mode == 'start':
            # slam_toolbox 把 map 原點定在車子開機的位置,所以 sim_world 在
            # map 裡的位姿 = 起始位姿的反矩陣。
            sx, sy, syaw = self.world.start
            c, s = math.cos(-syaw), math.sin(-syaw)
            t.transform.translation.x = -(c * sx - s * sy)
            t.transform.translation.y = -(s * sx + c * sy)
            t.transform.rotation = yaw_to_quat(-syaw)
        else:
            t.transform.rotation.w = 1.0
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform(t)

    # ---------- 主迴圈 ----------

    def _cmd_cb(self, msg):
        # 掉包模擬:直接不理這筆。真車上這是 WiFi 丟包,現象是 watchdog 跳掉
        # -> 底盤歸零 -> 下一筆到了又衝出去,也就是操作者說的「一頓一頓」。
        if self.drop_p and random.random() < self.drop_p:
            return
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()

        # 隨機長卡頓:模擬 WiFi 突發性停頓(比逐筆掉包更接近實測到的現象)
        if self.stall_until is not None:
            if (now - self.stall_until).nanoseconds < 0:
                self._integrate(0.0, 0.0, 0.0)
                self._publish(now)
                return
            self.stall_until = None
        elif self.stall_p and random.random() < self.stall_p:
            self.stall_until = now + rclpy.duration.Duration(seconds=self.stall_s)
            self.get_logger().warn(f'指令卡頓 {self.stall_s}s(模擬 WiFi 停頓)')

        # watchdog —— 和真 driver 一樣的行為
        age = (now - self.cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            tgt = (0.0, 0.0, 0.0)
        else:
            tgt = (self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z)

        self._integrate(*tgt)
        self._publish(now)

    def _integrate(self, tx_v, ty_v, tw):
        dt = self.dt

        # 加速度上限。沒有這個的話車子會瞬間到達指令速度,Nav2 怎麼調都很完美,
        # 一上實機就發現會打滑 —— 模擬反而害了你。
        def ramp(cur, tgt, amax):
            d = tgt - cur
            m = amax * dt
            return cur + max(-m, min(m, d))

        ax = abs(ramp(self.vx, tx_v, self.a_max) - self.vx) / dt
        ay = abs(ramp(self.vy, ty_v, self.a_max) - self.vy) / dt
        self.vx = ramp(self.vx, tx_v, self.a_max)
        self.vy = ramp(self.vy, ty_v, self.a_max)
        self.wz = ramp(self.wz, tw, self.a_max_th)

        # 打滑:實際位移比命令少。加速度越大滑得越兇(麥輪的真實行為)。
        slip = self.slip - self.slip_a * math.hypot(ax, ay)
        slip = max(0.0, slip)

        c, s = math.cos(self.tyaw), math.sin(self.tyaw)
        dx = (self.vx * c - self.vy * s) * dt * slip
        dy = (self.vx * s + self.vy * c) * dt * slip
        dyaw = self.wz * dt * slip

        # 撞牆判定 —— 規則五.5:碰到迷宮牆面任一處即當次失敗。
        #
        # 撞到就**擋住**,不讓車穿過去。這點很重要:允許穿牆的話,Nav2 會在
        # 模擬裡學會一條穿牆的捷徑並且「成功抵達」,你看到的是綠色的成功訊息,
        # 實機上卻是一頭撞死。擋住之後再沿軸滑動(先只走 x、再只走 y),
        # 貼著牆走比整個卡死接近真實 —— 麥輪車擦到牆通常是被推歪,不是原地停住。
        nx, ny = self.tx + dx, self.ty + dy
        blocked = self._collides(nx, ny)
        if blocked:
            if self._collides(self.tx + dx, self.ty):
                if self._collides(self.tx, self.ty + dy):
                    nx, ny = self.tx, self.ty          # 兩軸都不通 -> 卡住
                else:
                    nx, ny = self.tx, self.ty + dy     # 只能沿 y 滑
            else:
                nx, ny = self.tx + dx, self.ty         # 只能沿 x 滑

            if not self.was_hit:
                self.hit_count += 1
                self.get_logger().error(
                    f'★ 撞牆 #{self.hit_count} @ ({self.tx:.2f}, {self.ty:.2f}) '
                    f'—— 比賽規則:當次失敗,要重新從起點來過')
        self.was_hit = blocked

        self.tx, self.ty = nx, ny
        self.tyaw = math.atan2(math.sin(self.tyaw + dyaw), math.cos(self.tyaw + dyaw))

        # --- 里程計:車「以為」自己走了多少 ---
        # 注意這裡用的是**指令速度**(沒有 slip),再乘上刻度誤差。真車的
        # 編碼器量的是輪子轉了幾圈,輪子打滑時輪子照轉,所以 odom 不知道自己滑了。
        odx = (self.vx * c - self.vy * s) * dt * self.e_lin
        ody = (self.vx * s + self.vy * c) * dt * self.e_lin
        odyaw = self.wz * dt * self.e_ang

        dist = math.hypot(odx, ody)
        odyaw += self.drift * dist              # 每公尺累積的系統性偏差

        if self.n_xy:
            odx += random.gauss(0, self.n_xy)
            ody += random.gauss(0, self.n_xy)
        if self.n_yaw:
            odyaw += random.gauss(0, self.n_yaw)

        # odom 位姿要在自己的座標系裡積分(用 odom 的航向,不是真實航向)——
        # 這樣航向一旦錯開,位置誤差就會跟著滾雪球,和真車完全一樣。
        oc, os_ = math.cos(self.oyaw), math.sin(self.oyaw)
        lx = odx * c + ody * s                  # 先轉回車體座標
        ly = -odx * s + ody * c
        self.ox += lx * oc - ly * os_
        self.oy += lx * os_ + ly * oc
        self.oyaw = math.atan2(math.sin(self.oyaw + odyaw), math.cos(self.oyaw + odyaw))

    def _collides(self, x, y):
        """車體外接圓碰到任何牆就算撞。取圓周 12 點 + 圓心,夠用且便宜。"""
        if self.world.is_occupied(x, y):
            return True
        for k in range(12):
            a = k * math.pi / 6
            if self.world.is_occupied(x + self.radius * math.cos(a),
                                      y + self.radius * math.sin(a)):
                return True
        return False

    def _publish(self, now):
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.ox
        odom.pose.pose.position.y = self.oy
        odom.pose.pose.orientation = yaw_to_quat(self.oyaw)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.ox
        t.transform.translation.y = self.oy
        t.transform.rotation = yaw_to_quat(self.oyaw)
        self.tf.sendTransform(t)

        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = 'sim_world'
        truth.child_frame_id = 'sim_base_true'
        truth.pose.pose.position.x = self.tx
        truth.pose.pose.position.y = self.ty
        truth.pose.pose.orientation = yaw_to_quat(self.tyaw)
        self.truth_pub.publish(truth)

        tt = TransformStamped()
        tt.header.stamp = stamp
        tt.header.frame_id = 'sim_world'
        tt.child_frame_id = 'sim_base_true'
        tt.transform.translation.x = self.tx
        tt.transform.translation.y = self.ty
        tt.transform.rotation = yaw_to_quat(self.tyaw)
        self.tf.sendTransform(tt)

        # odom 相對起點走了多少 vs 真實相對起點走了多少
        sx, sy, syaw = self.world.start
        c, s = math.cos(syaw), math.sin(syaw)
        true_lx = (self.tx - sx) * c + (self.ty - sy) * s
        true_ly = -(self.tx - sx) * s + (self.ty - sy) * c
        e = Vector3()
        e.x = self.ox - true_lx
        e.y = self.oy - true_ly
        e.z = math.degrees(math.atan2(math.sin(self.oyaw - (self.tyaw - syaw)),
                                      math.cos(self.oyaw - (self.tyaw - syaw))))
        self.err_pub.publish(e)
        self._err = e

        b = Bool()
        b.data = self.was_hit
        self.hit_pub.publish(b)

    def _report(self):
        e = getattr(self, '_err', None)
        if e is None:
            return
        d = math.hypot(e.x, e.y)
        if d > 0.05 or abs(e.z) > 5.0:
            self.get_logger().warn(
                f'里程計誤差 {d:.3f}m / {e.z:+.1f}°  '
                f'(車以為在 {self.ox:+.2f},{self.oy:+.2f};實際 {self.tx:.2f},{self.ty:.2f})')


def main():
    rclpy.init()
    node = FakeBase()
    try:
        rclpy.spin(node)
    # ExternalShutdownException = 被 SIGTERM 關掉。run_sim.sh / gcs_sim.sh 都是用
    # pkill(SIGTERM)收尾,不接住的話每次正常關閉都會吐一整串 traceback,
    # 久了會讓人對紅字免疫,真的出事時反而看不見。
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 用 print 而不是 get_logger():走到這裡時 context 可能已經因為 SIGTERM
        # 失效,rosout publisher 會噴 "publisher's context is invalid"。
        if node.hit_count:
            print(f'\n*** 本次共撞牆 {node.hit_count} 次 —— 比賽規則:每次都是當次失敗 ***',
                  flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
