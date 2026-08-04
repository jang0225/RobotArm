from pathlib import Path

import pytest

from robot_arm_bringup.calibration import (
    load_calibration,
    relative_limits_deg,
    selected_joint_names,
    xacro_arguments,
)


CALIBRATION = (
    Path(__file__).parents[1] / "config" / "robot_arm_calibration.yaml"
)


def test_calibration_is_complete_and_has_unique_ids():
    calibration = load_calibration(CALIBRATION)
    assert selected_joint_names(False) == ["joint1", "joint2", "joint3"]
    assert selected_joint_names(True)[-1] == "gripper_joint"
    ids = [joint["motor_id"] for joint in calibration["joints"].values()]
    assert len(ids) == len(set(ids)) == 4


def test_relative_limits_are_centered_and_ordered():
    calibration = load_calibration(CALIBRATION)
    names, minimums, maximums, velocities = relative_limits_deg(calibration, True)
    assert names == ["joint1", "joint2", "joint3", "gripper_joint"]
    # Arm joints use their mechanical centre as zero.  The gripper instead uses
    # fully closed as zero, so its permitted range starts at zero.
    assert all(minimum < 0.0 < maximum for minimum, maximum in zip(minimums[:3], maximums[:3]))
    assert all(velocity > 0.0 for velocity in velocities)
    assert minimums[3] == pytest.approx(0.0)
    assert maximums[3] == pytest.approx(278.905469)
    assert calibration["joints"]["gripper_joint"]["max_opening_cm"] == 13.0


def test_xacro_receives_every_motor_calibration():
    calibration = load_calibration(CALIBRATION)
    arguments = " ".join(xacro_arguments(calibration))
    for joint_name in selected_joint_names(True):
        assert f"{joint_name}_zero_ticks:=" in arguments
        assert f"{joint_name}_min_absolute_deg:=" in arguments
        assert f"{joint_name}_max_absolute_deg:=" in arguments
