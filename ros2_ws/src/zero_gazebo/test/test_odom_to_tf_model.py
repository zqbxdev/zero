from __future__ import annotations

from dataclasses import replace
from math import inf, isclose, nan, sqrt

import pytest

from zero_gazebo.odom_to_tf_model import (
    NO_TRANSFORM,
    OdometryInput,
    Quaternion,
    SourceStamp,
    TransformCopy,
    Vector3,
    parse_odom_transform,
)


VALID_ODOM = OdometryInput(
    frame_id="odom",
    child_frame_id="zero_base_link",
    stamp=SourceStamp(sec=12, nanosec=345),
    position=Vector3(x=1.25, y=-2.5, z=-0.0),
    orientation=Quaternion(x=1.0, y=2.0, z=3.0, w=4.0),
)


def test_valid_odom_copies_source_transform_and_normalizes_quaternion() -> None:
    # Given: valid odometry with a non-unit finite quaternion
    expected_norm = sqrt(30.0)

    # When: the pure parser converts the sample
    result = parse_odom_transform(VALID_ODOM)

    # Then: source metadata and position are exact while orientation is normalized
    assert isinstance(result, TransformCopy)
    assert result.stamp == VALID_ODOM.stamp
    assert result.parent_frame_id == "odom"
    assert result.child_frame_id == "zero_base_link"
    assert result.translation == VALID_ODOM.position
    assert result.rotation == Quaternion(
        x=1.0 / expected_norm,
        y=2.0 / expected_norm,
        z=3.0 / expected_norm,
        w=4.0 / expected_norm,
    )
    assert isclose(
        sum(
            component * component
            for component in (
                result.rotation.x,
                result.rotation.y,
                result.rotation.z,
                result.rotation.w,
            )
        ),
        1.0,
    )


@pytest.mark.parametrize(
    "sample",
    (
        replace(VALID_ODOM, frame_id="map"),
        replace(VALID_ODOM, child_frame_id="base_link"),
        replace(VALID_ODOM, position=Vector3(nan, 0.0, 0.0)),
        replace(VALID_ODOM, position=Vector3(0.0, inf, 0.0)),
        replace(VALID_ODOM, position=Vector3(0.0, 0.0, -inf)),
        replace(VALID_ODOM, orientation=Quaternion(nan, 0.0, 0.0, 1.0)),
        replace(VALID_ODOM, orientation=Quaternion(0.0, inf, 0.0, 1.0)),
        replace(VALID_ODOM, orientation=Quaternion(0.0, 0.0, -inf, 1.0)),
        replace(VALID_ODOM, orientation=Quaternion(0.0, 0.0, 0.0, nan)),
        replace(VALID_ODOM, orientation=Quaternion(0.0, 0.0, 0.0, 0.0)),
        replace(VALID_ODOM, orientation=Quaternion(1.3e308, 1.3e308, 0.0, 0.0)),
    ),
)
def test_invalid_odom_returns_explicit_no_transform(sample: OdometryInput) -> None:
    # Given: malformed odometry from one rejected input class

    # When: the pure parser evaluates the sample
    result = parse_odom_transform(sample)

    # Then: invalid input produces the explicit no-output result
    assert result == NO_TRANSFORM


def test_alternating_invalid_and_valid_inputs_cannot_leak_state() -> None:
    # Given: two distinct valid samples separated by invalid frame and quaternion data
    later_valid = replace(
        VALID_ODOM,
        stamp=SourceStamp(sec=99, nanosec=7),
        position=Vector3(x=-8.0, y=9.0, z=10.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-2.0),
    )
    samples = (
        replace(VALID_ODOM, frame_id="map"),
        VALID_ODOM,
        replace(VALID_ODOM, orientation=Quaternion(0.0, 0.0, 0.0, 0.0)),
        later_valid,
    )

    # When: every message is parsed independently
    results = tuple(parse_odom_transform(sample) for sample in samples)

    # Then: invalid messages stay empty and later valid output uses only its own source
    assert results[0] == NO_TRANSFORM
    assert isinstance(results[1], TransformCopy)
    assert results[2] == NO_TRANSFORM
    assert results[3] == TransformCopy(
        stamp=later_valid.stamp,
        parent_frame_id="odom",
        child_frame_id="zero_base_link",
        translation=later_valid.position,
        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=-1.0),
    )
