from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Final


# Gazebo DiffDrive odometry frame pair expected by the TF relay.
# TF 中继期望的 Gazebo DiffDrive 里程计帧对。
ODOM_FRAME: Final = "odom"
BASE_FRAME: Final = "zero_base_link"


@dataclass(frozen=True, slots=True)
class SourceStamp:
    # ROS time seconds field.
    # ROS 时间秒字段。
    sec: int
    # ROS time nanoseconds field.
    # ROS 时间纳秒字段。
    nanosec: int


@dataclass(frozen=True, slots=True)
class Vector3:
    # X component in the parent frame [m].
    # 父坐标系下的 X 分量 [m]。
    x: float
    # Y component in the parent frame [m].
    # 父坐标系下的 Y 分量 [m]。
    y: float
    # Z component in the parent frame [m].
    # 父坐标系下的 Z 分量 [m]。
    z: float


@dataclass(frozen=True, slots=True)
class Quaternion:
    # Quaternion x component.
    # 四元数 x 分量。
    x: float
    # Quaternion y component.
    # 四元数 y 分量。
    y: float
    # Quaternion z component.
    # 四元数 z 分量。
    z: float
    # Quaternion w component.
    # 四元数 w 分量。
    w: float


@dataclass(frozen=True, slots=True)
class OdometryInput:
    frame_id: str
    child_frame_id: str
    stamp: SourceStamp
    position: Vector3
    orientation: Quaternion


@dataclass(frozen=True, slots=True)
class TransformCopy:
    stamp: SourceStamp
    parent_frame_id: str
    child_frame_id: str
    translation: Vector3
    rotation: Quaternion


@dataclass(frozen=True, slots=True)
class NoTransform:
    pass


NO_TRANSFORM: Final = NoTransform()
TransformParseResult = TransformCopy | NoTransform


def parse_odom_transform(sample: OdometryInput) -> TransformParseResult:
    # Relay only Gazebo's authoritative odom -> zero_base_link transform.
    # 仅中继 Gazebo 权威的 odom -> zero_base_link 变换。
    if sample.frame_id != ODOM_FRAME or sample.child_frame_id != BASE_FRAME:
        return NO_TRANSFORM

    position = sample.position
    position_components = (position.x, position.y, position.z)
    if not all(isfinite(component) for component in position_components):
        return NO_TRANSFORM

    orientation = sample.orientation
    orientation_components = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(isfinite(component) for component in orientation_components):
        return NO_TRANSFORM

    norm = hypot(*orientation_components)
    if norm == 0.0 or not isfinite(norm):
        return NO_TRANSFORM

    # Normalize orientation before broadcasting so TF receives a unit quaternion.
    # 广播前归一化姿态，确保 TF 收到单位四元数。
    return TransformCopy(
        stamp=sample.stamp,
        parent_frame_id=ODOM_FRAME,
        child_frame_id=BASE_FRAME,
        translation=position,
        rotation=Quaternion(
            x=orientation.x / norm,
            y=orientation.y / norm,
            z=orientation.z / norm,
            w=orientation.w / norm,
        ),
    )
