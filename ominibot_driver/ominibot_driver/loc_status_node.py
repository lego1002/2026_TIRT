"""Floating "ready to navigate?" label above the robot in RViz.

  /amcl_pose (geometry_msgs/PoseWithCovarianceStamped) --> /loc_status (visualization_msgs/Marker)

AMCL's particle cloud is hard to judge by eye -- "is that green blob small
enough?" -- so this turns AMCL's own covariance into a verdict next to the
robot: READY (green), ALMOST (yellow), NOT LOCALIZED (red), with the 1-sigma
position and heading spread. Same thresholds as tools/amcl_settle.py (menu
option 9), which is the thing to run when the label stays red with the robot
standing still: AMCL only re-scores particles when the robot moves past
update_min_d / update_min_a, so a stationary robot never converges on its own.

The label is drawn in English on purpose: RViz's text font has no CJK glyphs.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class LocStatus(Node):
    def __init__(self):
        super().__init__('loc_status')
        self.declare_parameter('good_xy', 0.05)     # m, 1-sigma
        self.declare_parameter('good_yaw', 5.0)     # deg, 1-sigma
        self.declare_parameter('ok_xy', 0.10)       # yellow band upper edge
        self.declare_parameter('ok_yaw', 10.0)
        self.declare_parameter('text_height', 0.08)  # m
        p = self.get_parameter
        self.good_xy, self.good_yaw = p('good_xy').value, p('good_yaw').value
        self.ok_xy, self.ok_yaw = p('ok_xy').value, p('ok_yaw').value
        self.text_h = p('text_height').value

        self.pose = None
        self.stamp = 0.0
        # amcl publishes /amcl_pose reliable + transient local
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._cb, qos)
        self.pub = self.create_publisher(Marker, 'loc_status', 1)
        self.create_timer(0.5, self._tick)

    def _cb(self, msg):
        self.pose = msg
        self.stamp = time.monotonic()

    def _tick(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'loc_status', 0
        m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        m.scale.z = self.text_h
        m.color.a = 1.0
        if self.pose is None:
            m.text = 'NO AMCL POSE'
            m.color.r, m.color.g, m.color.b = 1.0, 0.2, 0.2
            m.pose.orientation.w = 1.0
            self.pub.publish(m)
            return

        c = self.pose.pose.covariance
        sxy = math.sqrt(max(c[0], c[7], 0.0))
        syaw = math.degrees(math.sqrt(max(c[35], 0.0)))
        if sxy <= self.good_xy and syaw <= self.good_yaw:
            verdict, rgb = 'READY', (0.2, 1.0, 0.2)
        elif sxy <= self.ok_xy and syaw <= self.ok_yaw:
            verdict, rgb = 'ALMOST', (1.0, 0.9, 0.1)
        else:
            verdict, rgb = 'NOT LOCALIZED', (1.0, 0.2, 0.2)
        m.text = f'{verdict}  +-{sxy * 100:.0f}cm {syaw:.0f}deg'
        m.color.r, m.color.g, m.color.b = rgb
        m.pose.position.x = self.pose.pose.pose.position.x
        m.pose.position.y = self.pose.pose.pose.position.y
        m.pose.position.z = 0.35
        m.pose.orientation.w = 1.0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = LocStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
