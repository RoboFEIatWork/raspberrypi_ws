#!/usr/bin/env python3

import math
import time
from typing import List, cast

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .encoder import QuadratureEncoder


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Parameters (declare individually)
        self.declare_parameter('cpr', 2048)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx_plus_ly', 0.39)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)

        # Encoder pins (BCM numbering)
        self.declare_parameter('fl_a', 17)
        self.declare_parameter('fl_b', 27)
        self.declare_parameter('fl_invert', False)
        self.declare_parameter('fr_a', 22)
        self.declare_parameter('fr_b', 23)
        self.declare_parameter('fr_invert', False)
        self.declare_parameter('bl_a', 24)
        self.declare_parameter('bl_b', 25)
        self.declare_parameter('bl_invert', False)
        self.declare_parameter('br_a', 5)
        self.declare_parameter('br_b', 6)
        self.declare_parameter('br_invert', False)

        # Cache parameters
        self.cpr = cast(int, self.get_parameter('cpr').value)
        self.publish_rate = cast(float, self.get_parameter('publish_rate').value)
        self.wheel_radius = cast(float, self.get_parameter('wheel_radius').value)
        self.lx_plus_ly = cast(float, self.get_parameter('lx_plus_ly').value)
        self.base_frame_id = cast(str, self.get_parameter('base_frame_id').value)
        self.odom_frame_id = cast(str, self.get_parameter('odom_frame_id').value)
        self.publish_tf_param = cast(bool, self.get_parameter('publish_tf').value)

        # Encoders
        self.enc_fl = QuadratureEncoder(
            cast(int, self.get_parameter('fl_a').value),
            cast(int, self.get_parameter('fl_b').value),
            self.cpr,
            cast(bool, self.get_parameter('fl_invert').value),
        )
        self.enc_fr = QuadratureEncoder(
            cast(int, self.get_parameter('fr_a').value),
            cast(int, self.get_parameter('fr_b').value),
            self.cpr,
            cast(bool, self.get_parameter('fr_invert').value),
        )
        self.enc_bl = QuadratureEncoder(
            cast(int, self.get_parameter('bl_a').value),
            cast(int, self.get_parameter('bl_b').value),
            self.cpr,
            cast(bool, self.get_parameter('bl_invert').value),
        )
        self.enc_br = QuadratureEncoder(
            cast(int, self.get_parameter('br_a').value),
            cast(int, self.get_parameter('br_b').value),
            self.cpr,
            cast(bool, self.get_parameter('br_invert').value),
        )

    # Encoders start their polling threads automatically in __init__

        # State
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]  # rad
        self.last_counts = [0, 0, 0, 0]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
        ]

        self.timer = self.create_timer(1.0 / max(1.0, self.publish_rate), self._on_timer)

    def _read_counts(self) -> List[int]:
        return [
            self.enc_fl.get_count(),
            self.enc_fr.get_count(),
            self.enc_bl.get_count(),
            self.enc_br.get_count(),
        ]

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return

        counts = self._read_counts()
        cpr = float(self.cpr)
        R = float(self.wheel_radius)
        L = float(self.lx_plus_ly)

        # Delta angles per wheel in rad
        deltas = []
        for i in range(4):
            dcount = counts[i] - self.last_counts[i]
            self.last_counts[i] = counts[i]
            dtheta = (2.0 * math.pi) * (dcount / cpr)
            self.wheel_pos[i] += dtheta
            deltas.append(dtheta)

        # Wheel angular velocities (rad/s)
        w_fl, w_fr, w_bl, w_br = [d / dt for d in deltas]

        # Mecanum forward kinematics to body twist
        # Using convention: +x forward, +y left, +z yaw CCW
        Vx = (w_fl + w_fr + w_bl + w_br) * (R / 4.0)
        Vy = (-w_fl + w_fr + w_bl - w_br) * (R / 4.0)
        Wz = (-w_fl + w_fr - w_bl + w_br) * (R / (4.0 * L))

        # Integrate to pose in odom frame
        cos_th = math.cos(self.th)
        sin_th = math.sin(self.th)
        self.x += (Vx * cos_th - Vy * sin_th) * dt
        self.y += (Vx * sin_th + Vy * cos_th) * dt
        self.th += Wz * dt

        # Publish JointState
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = self.wheel_pos
        js.velocity = [w_fl, w_fr, w_bl, w_br]
        self.joint_pub.publish(js)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qz = math.sin(self.th * 0.5)
        qw = math.cos(self.th * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = Vx
        odom.twist.twist.linear.y = Vy
        odom.twist.twist.angular.z = Wz
        self.odom_pub.publish(odom)

        # TF (odom -> base)
        if self.publish_tf_param:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = odom.header.frame_id
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        self.last_time = now

    def destroy_node(self):
        try:
            for e in (self.enc_fl, self.enc_fr, self.enc_bl, self.enc_br):
                e.stop()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
