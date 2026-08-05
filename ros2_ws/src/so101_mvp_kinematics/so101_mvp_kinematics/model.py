from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


def _parse_vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if text is None:
        values = default
    else:
        parts = text.split()
        if len(parts) != 3:
            raise ValueError(f"Expected 3 vector values, got {text!r}")
        values = tuple(float(part) for part in parts)

    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"Invalid vector values: {text!r}")
    return vector


class So101KinematicModel:
    """Small URDF-backed SO-101 arm model for offline MVP kinematics."""

    def __init__(
        self,
        urdf_path: str | Path,
        base_link: str = BASE_LINK,
        tip_link: str = TIP_LINK,
        joint_names: list[str] | tuple[str, ...] = tuple(JOINT_NAMES),
    ) -> None:
        self.urdf_path = Path(urdf_path)
        self.base_link = base_link
        self.tip_link = tip_link
        self.joint_names = list(joint_names)

        self.all_joints = self._parse_urdf_joints()
        self.chain = self._build_chain()
        self.active_joints = [joint for joint in self.chain if joint.joint_type != "fixed"]
        self._validate_expected_chain()

        self.lower_limits = np.asarray(
            [joint.lower for joint in self.active_joints],
            dtype=np.float64,
        )
        self.upper_limits = np.asarray(
            [joint.upper for joint in self.active_joints],
            dtype=np.float64,
        )

    def _parse_urdf_joints(self) -> dict[str, JointSpec]:
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")

        root = ET.parse(self.urdf_path).getroot()
        joints: dict[str, JointSpec] = {}

        for element in root.findall("joint"):
            name = element.attrib.get("name")
            joint_type = element.attrib.get("type")
            if not name or not joint_type:
                raise ValueError("URDF joint is missing name or type")

            parent = element.find("parent")
            child = element.find("child")
            if parent is None or child is None:
                raise ValueError(f"Joint {name} is missing parent or child")

            origin = element.find("origin")
            axis = element.find("axis")
            limit = element.find("limit")

            lower = None
            upper = None
            if limit is not None:
                if "lower" in limit.attrib:
                    lower = float(limit.attrib["lower"])
                if "upper" in limit.attrib:
                    upper = float(limit.attrib["upper"])

            joints[name] = JointSpec(
                name=name,
                joint_type=joint_type,
                parent=parent.attrib["link"],
                child=child.attrib["link"],
                origin_xyz=_parse_vector(
                    None if origin is None else origin.attrib.get("xyz"),
                    (0.0, 0.0, 0.0),
                ),
                origin_rpy=_parse_vector(
                    None if origin is None else origin.attrib.get("rpy"),
                    (0.0, 0.0, 0.0),
                ),
                axis=_parse_vector(
                    None if axis is None else axis.attrib.get("xyz"),
                    (1.0, 0.0, 0.0),
                ),
                lower=lower,
                upper=upper,
            )

        return joints

    def _build_chain(self) -> list[JointSpec]:
        joints_by_child = {joint.child: joint for joint in self.all_joints.values()}
        reversed_chain: list[JointSpec] = []
        current = self.tip_link
        visited: set[str] = set()

        while current != self.base_link:
            if current in visited:
                raise ValueError(f"Cycle detected while resolving link {current}")
            visited.add(current)

            joint = joints_by_child.get(current)
            if joint is None:
                raise ValueError(
                    f"No URDF joint connects {current} back toward {self.base_link}"
                )
            reversed_chain.append(joint)
            current = joint.parent

        return list(reversed(reversed_chain))

    def _validate_expected_chain(self) -> None:
        actual_active = [joint.name for joint in self.active_joints]
        if actual_active != self.joint_names:
            raise ValueError(
                "Unexpected active joint order in URDF chain. "
                f"Expected {self.joint_names}, got {actual_active}"
            )

        if not self.chain or self.chain[-1].name != "gripper_frame_joint":
            raise ValueError("Expected chain to end with fixed gripper_frame_joint")

        for joint in self.active_joints:
            if joint.joint_type != "revolute":
                raise ValueError(f"Joint {joint.name} must be revolute")
            if joint.lower is None or joint.upper is None:
                raise ValueError(f"Joint {joint.name} is missing lower/upper limits")
            if joint.lower >= joint.upper:
                raise ValueError(f"Joint {joint.name} has invalid limits")

    def joint_dict(self, joint_positions_rad: np.ndarray) -> dict[str, float]:
        values = np.asarray(joint_positions_rad, dtype=np.float64)
        if values.shape != (len(self.joint_names),):
            raise ValueError(f"Expected {len(self.joint_names)} joint values")
        if not np.all(np.isfinite(values)):
            raise ValueError("Joint positions contain NaN or Inf")
        return {
            name: float(value)
            for name, value in zip(self.joint_names, values.tolist(), strict=True)
        }
