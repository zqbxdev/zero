from math import pi

import pytest

from zero_sim.simple_usv_model import SimpleUsvModel, SimpleUsvPose, SimpleUsvTuning


def test_forward_motion_when_equal_positive_rpm_at_zero_yaw() -> None:
    model = SimpleUsvModel(
        tuning=SimpleUsvTuning(
            rpm_to_linear_velocity=0.01,
            rpm_difference_to_angular_velocity=0.002,
        ),
    )

    state = model.integrate(left_actual_rpm=100.0, right_actual_rpm=100.0, dt_seconds=2.0)

    assert state.pose.x == pytest.approx(2.0)
    assert state.pose.y == pytest.approx(0.0)
    assert state.pose.yaw == pytest.approx(0.0)
    assert state.twist.linear_velocity == pytest.approx(1.0)
    assert state.twist.angular_velocity == pytest.approx(0.0)


def test_reverse_motion_when_equal_negative_rpm_at_zero_yaw() -> None:
    model = SimpleUsvModel(
        tuning=SimpleUsvTuning(
            rpm_to_linear_velocity=0.01,
            rpm_difference_to_angular_velocity=0.002,
        ),
    )

    state = model.integrate(left_actual_rpm=-50.0, right_actual_rpm=-50.0, dt_seconds=2.0)

    assert state.pose.x == pytest.approx(-1.0)
    assert state.pose.y == pytest.approx(0.0)
    assert state.twist.linear_velocity == pytest.approx(-0.5)
    assert state.twist.angular_velocity == pytest.approx(0.0)


def test_yaw_changes_when_right_rpm_exceeds_left_rpm() -> None:
    model = SimpleUsvModel(
        tuning=SimpleUsvTuning(
            rpm_to_linear_velocity=0.01,
            rpm_difference_to_angular_velocity=0.002,
        ),
    )

    state = model.integrate(left_actual_rpm=20.0, right_actual_rpm=80.0, dt_seconds=3.0)

    assert state.pose.yaw == pytest.approx(0.36)
    assert state.twist.linear_velocity == pytest.approx(0.5)
    assert state.twist.angular_velocity == pytest.approx(0.12)


def test_zero_rpm_keeps_pose_and_twist_zero() -> None:
    model = SimpleUsvModel(
        pose=SimpleUsvPose(x=1.0, y=-2.0, yaw=0.5),
        tuning=SimpleUsvTuning(
            rpm_to_linear_velocity=0.01,
            rpm_difference_to_angular_velocity=0.002,
        ),
    )

    state = model.integrate(left_actual_rpm=0.0, right_actual_rpm=0.0, dt_seconds=5.0)

    assert state.pose.x == pytest.approx(1.0)
    assert state.pose.y == pytest.approx(-2.0)
    assert state.pose.yaw == pytest.approx(0.5)
    assert state.twist.linear_velocity == pytest.approx(0.0)
    assert state.twist.angular_velocity == pytest.approx(0.0)


def test_nonzero_heading_integrates_x_and_y_components() -> None:
    model = SimpleUsvModel(
        pose=SimpleUsvPose(x=0.0, y=0.0, yaw=pi / 2.0),
        tuning=SimpleUsvTuning(
            rpm_to_linear_velocity=0.01,
            rpm_difference_to_angular_velocity=0.002,
        ),
    )

    state = model.integrate(left_actual_rpm=100.0, right_actual_rpm=100.0, dt_seconds=1.5)

    assert state.pose.x == pytest.approx(0.0)
    assert state.pose.y == pytest.approx(1.5)
    assert state.pose.yaw == pytest.approx(pi / 2.0)
