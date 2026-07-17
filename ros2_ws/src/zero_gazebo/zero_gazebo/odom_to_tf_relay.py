#!/usr/bin/env python3
from __future__ import annotations

from typing import assert_never

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .odom_to_tf_model import (
    NoTransform,
    OdometryInput,
    Quaternion,
    SourceStamp,
    TransformCopy,
    Vector3,
    parse_odom_transform,
)


class OdomToTfRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tf_relay")
        # Broadcast only the odom -> zero_base_link transform copied from Gazebo odometry.
        # 仅广播从 Gazebo 里程计复制出的 odom -> zero_base_link 变换。
        self._transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        result = parse_odom_transform(
            OdometryInput(
                frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id,
                stamp=SourceStamp(
                    sec=msg.header.stamp.sec,
                    nanosec=msg.header.stamp.nanosec,
                ),
                position=Vector3(
                    x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y,
                    z=msg.pose.pose.position.z,
                ),
                orientation=Quaternion(
                    x=msg.pose.pose.orientation.x,
                    y=msg.pose.pose.orientation.y,
                    z=msg.pose.pose.orientation.z,
                    w=msg.pose.pose.orientation.w,
                ),
            ),
        )
        match result:
            case NoTransform():
                # Drop odometry samples outside the expected frame pair or numeric range.
                # 丢弃帧对或数值范围不符合预期的里程计样本。
                return
            case TransformCopy():
                transform = TransformStamped()
                transform.header.stamp.sec = result.stamp.sec
                transform.header.stamp.nanosec = result.stamp.nanosec
                transform.header.frame_id = result.parent_frame_id
                transform.child_frame_id = result.child_frame_id
                transform.transform.translation.x = result.translation.x
                transform.transform.translation.y = result.translation.y
                transform.transform.translation.z = result.translation.z
                transform.transform.rotation.x = result.rotation.x
                transform.transform.rotation.y = result.rotation.y
                transform.transform.rotation.z = result.rotation.z
                transform.transform.rotation.w = result.rotation.w
                self._transform_broadcaster.sendTransform(transform)
            case unreachable:
                assert_never(unreachable)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomToTfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
