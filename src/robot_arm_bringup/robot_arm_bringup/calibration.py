"""Load and validate the robot-arm calibration source of truth."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml


ARM_JOINTS = ("joint1", "joint2", "joint3")
GRIPPER_JOINT = "gripper_joint"
ALL_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)
REQUIRED_FIELDS = (
    "motor_id",
    "expected_model",
    "expected_operating_mode",
    "expected_drive_mode",
    "zero_ticks",
    "direction",
    "min_absolute_deg",
    "max_absolute_deg",
    "max_velocity_deg_s",
)


def load_calibration(path: Path) -> dict[str, Any]:
    """Return a validated calibration dictionary."""
    with path.open("r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)

    if not isinstance(calibration, dict):
        raise ValueError(f"{path}: calibration root must be a mapping")
    degrees_per_tick = calibration.get("degrees_per_tick")
    if not _finite_number(degrees_per_tick) or degrees_per_tick <= 0.0:
        raise ValueError(f"{path}: degrees_per_tick must be positive and finite")

    joints = calibration.get("joints")
    if not isinstance(joints, dict) or set(joints) != set(ALL_JOINTS):
        raise ValueError(f"{path}: joints must contain exactly {', '.join(ALL_JOINTS)}")

    motor_ids: set[int] = set()
    for name in ALL_JOINTS:
        joint = joints[name]
        if not isinstance(joint, dict):
            raise ValueError(f"{path}: {name} must be a mapping")
        missing = [field for field in REQUIRED_FIELDS if field not in joint]
        if missing:
            raise ValueError(f"{path}: {name} is missing {', '.join(missing)}")
        if not all(_finite_number(joint[field]) for field in REQUIRED_FIELDS):
            raise ValueError(f"{path}: {name} contains a non-finite numeric value")

        motor_id = int(joint["motor_id"])
        if motor_id < 0 or motor_id > 252 or motor_id in motor_ids:
            raise ValueError(f"{path}: invalid or duplicate motor_id for {name}")
        motor_ids.add(motor_id)
        if int(joint["zero_ticks"]) < 0 or int(joint["zero_ticks"]) > 4095:
            raise ValueError(f"{path}: {name} zero_ticks must be within [0, 4095]")
        if float(joint["direction"]) not in (-1.0, 1.0):
            raise ValueError(f"{path}: {name} direction must be -1.0 or 1.0")
        if float(joint["min_absolute_deg"]) >= float(joint["max_absolute_deg"]):
            raise ValueError(f"{path}: {name} min angle must be smaller than max angle")
        if (
            float(joint["min_absolute_deg"]) < 0.0
            or float(joint["max_absolute_deg"]) > 360.0
        ):
            raise ValueError(
                f"{path}: {name} absolute angles must be within [0, 360]"
            )
        if float(joint["max_velocity_deg_s"]) <= 0.0:
            raise ValueError(f"{path}: {name} max velocity must be positive")
        if name == GRIPPER_JOINT:
            max_opening_cm = joint.get("max_opening_cm")
            if not _finite_number(max_opening_cm) or float(max_opening_cm) <= 0.0:
                raise ValueError(f"{path}: {name} max_opening_cm must be positive")

    return calibration


def selected_joint_names(include_gripper: bool) -> list[str]:
    names = list(ARM_JOINTS)
    if include_gripper:
        names.append(GRIPPER_JOINT)
    return names


def relative_limits_deg(
    calibration: dict[str, Any], include_gripper: bool
) -> tuple[list[str], list[float], list[float], list[float]]:
    """Return names and reference-relative min/max/velocity values in degrees."""
    degrees_per_tick = float(calibration["degrees_per_tick"])
    names = selected_joint_names(include_gripper)
    minimums: list[float] = []
    maximums: list[float] = []
    velocities: list[float] = []
    for name in names:
        joint = calibration["joints"][name]
        zero_deg = int(joint["zero_ticks"]) * degrees_per_tick
        direction = float(joint["direction"])
        endpoint_a = direction * (float(joint["min_absolute_deg"]) - zero_deg)
        endpoint_b = direction * (float(joint["max_absolute_deg"]) - zero_deg)
        minimums.append(min(endpoint_a, endpoint_b))
        maximums.append(max(endpoint_a, endpoint_b))
        velocities.append(float(joint["max_velocity_deg_s"]))
    return names, minimums, maximums, velocities


def xacro_arguments(calibration: dict[str, Any]) -> list[str]:
    """Return xacro name:=value arguments for every calibrated motor."""
    arguments = [
        " degrees_per_tick:=",
        str(calibration["degrees_per_tick"]),
    ]
    for name in ALL_JOINTS:
        joint = calibration["joints"][name]
        for field in (
            "motor_id",
            "expected_model",
            "expected_operating_mode",
            "expected_drive_mode",
            "zero_ticks",
            "direction",
            "min_absolute_deg",
            "max_absolute_deg",
        ):
            arguments.extend([" ", f"{name}_{field}:=", str(joint[field])])
    return arguments


def radians(values_deg: list[float]) -> list[float]:
    return [math.radians(value) for value in values_deg]


def _finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))
