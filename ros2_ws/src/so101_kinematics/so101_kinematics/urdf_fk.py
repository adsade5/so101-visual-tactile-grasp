from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_URDF_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "so101_new_calib.urdf"
)

DEFAULT_REPORT_PATH = (
    PROJECT_ROOT
    / "data"
    / "robot_model"
    / "so101"
    / "fk_verification.json"
)

BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# 根据当前冻结URDF计算得到的零位末端位置。
# 这是一项回归检查，后续URDF被误改时会直接失败。
EXPECTED_ZERO_POSITION_M = np.asarray(
    [
        0.3913614702,
        -0.0000092121,
        0.2264697102,
    ],
    dtype=np.float64,
)

ZERO_POSITION_TOLERANCE_M = 5e-5  # 0.05 mm


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


def parse_vector(
    text: str | None,
    default: tuple[float, float, float],
) -> np.ndarray:
    if text is None:
        return np.asarray(
            default,
            dtype=np.float64,
        )

    values = [
        float(value)
        for value in text.split()
    ]

    if len(values) != 3:
        raise ValueError(
            f"Expected three values, got: {text}"
        )

    vector = np.asarray(
        values,
        dtype=np.float64,
    )

    if not np.all(np.isfinite(vector)):
        raise ValueError(
            f"Vector contains NaN or Inf: {text}"
        )

    return vector


def rpy_to_rotation(
    rpy: np.ndarray,
) -> np.ndarray:
    roll, pitch, yaw = [
        float(value)
        for value in rpy
    ]

    cr = math.cos(roll)
    sr = math.sin(roll)

    cp = math.cos(pitch)
    sp = math.sin(pitch)

    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # URDF固定轴RPY约定。
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def axis_angle_to_rotation(
    axis: np.ndarray,
    angle: float,
) -> np.ndarray:
    axis_norm = float(
        np.linalg.norm(axis)
    )

    if axis_norm <= 1e-12:
        raise ValueError(
            "Revolute joint axis has zero length"
        )

    x, y, z = axis / axis_norm

    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine

    return np.asarray(
        [
            [
                cosine + x * x * one_minus_cosine,
                x * y * one_minus_cosine - z * sine,
                x * z * one_minus_cosine + y * sine,
            ],
            [
                y * x * one_minus_cosine + z * sine,
                cosine + y * y * one_minus_cosine,
                y * z * one_minus_cosine - x * sine,
            ],
            [
                z * x * one_minus_cosine - y * sine,
                z * y * one_minus_cosine + x * sine,
                cosine + z * z * one_minus_cosine,
            ],
        ],
        dtype=np.float64,
    )


def make_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return transform


def compose_transform(
    left: np.ndarray,
    right: np.ndarray,
) -> np.ndarray:
    result = np.eye(
        4,
        dtype=np.float64,
    )
    left_rotation = left[:3, :3]
    right_rotation = right[:3, :3]
    left_translation = left[:3, 3]
    right_translation = right[:3, 3]

    for row in range(3):
        for col in range(3):
            result[row, col] = (
                left_rotation[row, 0] * right_rotation[0, col]
                + left_rotation[row, 1] * right_rotation[1, col]
                + left_rotation[row, 2] * right_rotation[2, col]
            )
        result[row, 3] = (
            left_rotation[row, 0] * right_translation[0]
            + left_rotation[row, 1] * right_translation[1]
            + left_rotation[row, 2] * right_translation[2]
            + left_translation[row]
        )

    return result


def rotation_to_quaternion_xyzw(
    rotation: np.ndarray,
) -> np.ndarray:
    trace = float(
        np.trace(rotation)
    )

    if trace > 0.0:
        scale = math.sqrt(
            trace + 1.0
        ) * 2.0

        qw = 0.25 * scale
        qx = (
            rotation[2, 1]
            - rotation[1, 2]
        ) / scale
        qy = (
            rotation[0, 2]
            - rotation[2, 0]
        ) / scale
        qz = (
            rotation[1, 0]
            - rotation[0, 1]
        ) / scale

    elif (
        rotation[0, 0] > rotation[1, 1]
        and rotation[0, 0] > rotation[2, 2]
    ):
        scale = math.sqrt(
            1.0
            + rotation[0, 0]
            - rotation[1, 1]
            - rotation[2, 2]
        ) * 2.0

        qw = (
            rotation[2, 1]
            - rotation[1, 2]
        ) / scale
        qx = 0.25 * scale
        qy = (
            rotation[0, 1]
            + rotation[1, 0]
        ) / scale
        qz = (
            rotation[0, 2]
            + rotation[2, 0]
        ) / scale

    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(
            1.0
            + rotation[1, 1]
            - rotation[0, 0]
            - rotation[2, 2]
        ) * 2.0

        qw = (
            rotation[0, 2]
            - rotation[2, 0]
        ) / scale
        qx = (
            rotation[0, 1]
            + rotation[1, 0]
        ) / scale
        qy = 0.25 * scale
        qz = (
            rotation[1, 2]
            + rotation[2, 1]
        ) / scale

    else:
        scale = math.sqrt(
            1.0
            + rotation[2, 2]
            - rotation[0, 0]
            - rotation[1, 1]
        ) * 2.0

        qw = (
            rotation[1, 0]
            - rotation[0, 1]
        ) / scale
        qx = (
            rotation[0, 2]
            + rotation[2, 0]
        ) / scale
        qy = (
            rotation[1, 2]
            + rotation[2, 1]
        ) / scale
        qz = 0.25 * scale

    quaternion = np.asarray(
        [qx, qy, qz, qw],
        dtype=np.float64,
    )

    quaternion /= np.linalg.norm(
        quaternion
    )

    # q与-q表示同一个旋转。
    # 固定w为非负，便于输出比较。
    if quaternion[3] < 0.0:
        quaternion = -quaternion

    return quaternion


class UrdfForwardKinematics:
    def __init__(
        self,
        urdf_path: Path,
        base_link: str,
        tip_link: str,
    ) -> None:
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.tip_link = tip_link

        self.joints = self._parse_joints()
        self.chain = self._build_chain()

    def _parse_joints(
        self,
    ) -> dict[str, JointSpec]:
        if not self.urdf_path.is_file():
            raise FileNotFoundError(
                f"URDF not found: {self.urdf_path}"
            )

        robot = ET.parse(
            self.urdf_path
        ).getroot()

        joints: dict[str, JointSpec] = {}

        for element in robot.findall("joint"):
            name = element.attrib["name"]
            joint_type = element.attrib["type"]

            parent_element = element.find(
                "parent"
            )
            child_element = element.find(
                "child"
            )

            if (
                parent_element is None
                or child_element is None
            ):
                raise ValueError(
                    f"Joint {name} is missing "
                    "parent or child"
                )

            origin_element = element.find(
                "origin"
            )

            xyz = parse_vector(
                None
                if origin_element is None
                else origin_element.attrib.get(
                    "xyz"
                ),
                (0.0, 0.0, 0.0),
            )

            rpy = parse_vector(
                None
                if origin_element is None
                else origin_element.attrib.get(
                    "rpy"
                ),
                (0.0, 0.0, 0.0),
            )

            axis_element = element.find(
                "axis"
            )

            axis = parse_vector(
                None
                if axis_element is None
                else axis_element.attrib.get(
                    "xyz"
                ),
                (1.0, 0.0, 0.0),
            )

            limit_element = element.find(
                "limit"
            )

            lower = None
            upper = None

            if limit_element is not None:
                if "lower" in limit_element.attrib:
                    lower = float(
                        limit_element.attrib[
                            "lower"
                        ]
                    )

                if "upper" in limit_element.attrib:
                    upper = float(
                        limit_element.attrib[
                            "upper"
                        ]
                    )

            joints[name] = JointSpec(
                name=name,
                joint_type=joint_type,
                parent=parent_element.attrib[
                    "link"
                ],
                child=child_element.attrib[
                    "link"
                ],
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )

        return joints

    def _build_chain(
        self,
    ) -> list[JointSpec]:
        joints_by_child = {
            joint.child: joint
            for joint in self.joints.values()
        }

        reversed_chain: list[JointSpec] = []

        current_link = self.tip_link
        visited_links: set[str] = set()

        while current_link != self.base_link:
            if current_link in visited_links:
                raise ValueError(
                    "Cycle detected in URDF"
                )

            visited_links.add(current_link)

            joint = joints_by_child.get(
                current_link
            )

            if joint is None:
                raise ValueError(
                    f"No joint connects "
                    f"{current_link} toward "
                    f"{self.base_link}"
                )

            reversed_chain.append(joint)
            current_link = joint.parent

        return list(
            reversed(reversed_chain)
        )

    def validate_expected_chain(
        self,
    ) -> None:
        actual_names = [
            joint.name
            for joint in self.chain
        ]

        expected_names = (
            ARM_JOINT_NAMES
            + ["gripper_frame_joint"]
        )

        if actual_names != expected_names:
            raise ValueError(
                "Unexpected FK chain.\n"
                f"Expected: {expected_names}\n"
                f"Actual:   {actual_names}"
            )

        if self.chain[-1].joint_type != "fixed":
            raise ValueError(
                "gripper_frame_joint must be fixed"
            )

        if "gripper" in actual_names:
            raise ValueError(
                "Gripper opening joint must not "
                "appear in arm FK chain"
            )

    def check_joint_limits(
        self,
        joint_positions: dict[str, float],
    ) -> None:
        for joint in self.chain:
            if joint.joint_type != "revolute":
                continue

            angle = float(
                joint_positions.get(
                    joint.name,
                    0.0,
                )
            )

            if (
                joint.lower is not None
                and angle < joint.lower
            ):
                raise ValueError(
                    f"{joint.name}={angle:.6f} rad "
                    f"is below limit "
                    f"{joint.lower:.6f}"
                )

            if (
                joint.upper is not None
                and angle > joint.upper
            ):
                raise ValueError(
                    f"{joint.name}={angle:.6f} rad "
                    f"is above limit "
                    f"{joint.upper:.6f}"
                )

    def compute(
        self,
        joint_positions: dict[str, float],
    ) -> np.ndarray:
        self.check_joint_limits(
            joint_positions
        )

        transform = np.eye(
            4,
            dtype=np.float64,
        )

        for joint in self.chain:
            origin_transform = make_transform(
                rpy_to_rotation(joint.rpy),
                joint.xyz,
            )

            transform = compose_transform(
                transform,
                origin_transform,
            )

            if joint.joint_type == "revolute":
                angle = float(
                    joint_positions.get(
                        joint.name,
                        0.0,
                    )
                )

                joint_rotation = (
                    axis_angle_to_rotation(
                        joint.axis,
                        angle,
                    )
                )

                transform = compose_transform(
                    transform,
                    make_transform(
                        joint_rotation,
                        np.zeros(
                            3,
                            dtype=np.float64,
                        ),
                    )
                )

            elif joint.joint_type == "fixed":
                continue

            else:
                raise ValueError(
                    f"Unsupported joint type "
                    f"{joint.joint_type} "
                    f"for {joint.name}"
                )

        return transform


def validate_transform(
    transform: np.ndarray,
) -> dict[str, float]:
    if transform.shape != (4, 4):
        raise ValueError(
            f"Invalid transform shape: "
            f"{transform.shape}"
        )

    if not np.all(np.isfinite(transform)):
        raise ValueError(
            "Transform contains NaN or Inf"
        )

    rotation = transform[:3, :3]

    orthogonality_error = float(
        np.linalg.norm(
            rotation.T
            @ rotation
            - np.eye(3)
        )
    )

    determinant = float(
        np.linalg.det(rotation)
    )

    bottom_row_error = float(
        np.linalg.norm(
            transform[3]
            - np.asarray(
                [0.0, 0.0, 0.0, 1.0]
            )
        )
    )

    if orthogonality_error > 1e-9:
        raise ValueError(
            "Rotation is not orthonormal: "
            f"{orthogonality_error:.3e}"
        )

    if abs(determinant - 1.0) > 1e-9:
        raise ValueError(
            "Rotation determinant is not 1: "
            f"{determinant:.12f}"
        )

    if bottom_row_error > 1e-12:
        raise ValueError(
            "Invalid homogeneous transform "
            f"bottom row: {bottom_row_error:.3e}"
        )

    return {
        "orthogonality_error": (
            orthogonality_error
        ),
        "rotation_determinant": determinant,
        "bottom_row_error": bottom_row_error,
    }


def evaluate_pose(
    solver: UrdfForwardKinematics,
    name: str,
    q_degrees: list[float],
) -> dict[str, Any]:
    if len(q_degrees) != 5:
        raise ValueError(
            "Expected five arm joint angles"
        )

    q_radians = np.radians(
        np.asarray(
            q_degrees,
            dtype=np.float64,
        )
    )

    joint_positions = dict(
        zip(
            ARM_JOINT_NAMES,
            q_radians.tolist(),
            strict=True,
        )
    )

    transform = solver.compute(
        joint_positions
    )

    numerical_checks = validate_transform(
        transform
    )

    quaternion = (
        rotation_to_quaternion_xyzw(
            transform[:3, :3]
        )
    )

    return {
        "name": name,
        "joint_names": ARM_JOINT_NAMES,
        "q_degrees": [
            float(value)
            for value in q_degrees
        ],
        "q_radians": [
            float(value)
            for value in q_radians
        ],
        "position_m": (
            transform[:3, 3].tolist()
        ),
        "position_mm": (
            transform[:3, 3]
            * 1000.0
        ).tolist(),
        "rotation_matrix": (
            transform[:3, :3].tolist()
        ),
        "quaternion_xyzw": (
            quaternion.tolist()
        ),
        "transform": transform.tolist(),
        "numerical_checks": (
            numerical_checks
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Offline SO-101 URDF forward "
            "kinematics verification."
        )
    )

    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF_PATH,
    )

    parser.add_argument(
        "--report",
        type=Path,
        default=DEFAULT_REPORT_PATH,
    )

    parser.add_argument(
        "--q-deg",
        type=float,
        nargs=5,
        metavar=(
            "PAN",
            "LIFT",
            "ELBOW",
            "WRIST_FLEX",
            "WRIST_ROLL",
        ),
        help=(
            "Evaluate one additional pose "
            "using joint angles in degrees."
        ),
    )

    args = parser.parse_args()

    solver = UrdfForwardKinematics(
        urdf_path=args.urdf.resolve(),
        base_link=BASE_LINK,
        tip_link=TIP_LINK,
    )

    solver.validate_expected_chain()

    test_cases = [
        (
            "zero",
            [0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        (
            "pose_a",
            [30.0, -30.0, 45.0, -20.0, 15.0],
        ),
        (
            "pose_b",
            [-30.0, 20.0, -40.0, 30.0, -15.0],
        ),
    ]

    if args.q_deg is not None:
        test_cases.append(
            (
                "user_pose",
                [
                    float(value)
                    for value in args.q_deg
                ],
            )
        )

    results = [
        evaluate_pose(
            solver,
            name,
            q_degrees,
        )
        for name, q_degrees in test_cases
    ]

    zero_position = np.asarray(
        results[0]["position_m"],
        dtype=np.float64,
    )

    zero_position_error_m = float(
        np.linalg.norm(
            zero_position
            - EXPECTED_ZERO_POSITION_M
        )
    )

    if (
        zero_position_error_m
        > ZERO_POSITION_TOLERANCE_M
    ):
        raise ValueError(
            "Zero-pose regression failed: "
            f"{zero_position_error_m * 1000.0:.6f} mm"
        )

    chain_report = []

    for joint in solver.chain:
        chain_report.append(
            {
                "name": joint.name,
                "type": joint.joint_type,
                "parent": joint.parent,
                "child": joint.child,
                "origin_xyz": (
                    joint.xyz.tolist()
                ),
                "origin_rpy": (
                    joint.rpy.tolist()
                ),
                "axis": joint.axis.tolist(),
                "lower_rad": joint.lower,
                "upper_rad": joint.upper,
            }
        )

    report = {
        "status": "PASS",
        "urdf_path": str(
            solver.urdf_path
        ),
        "base_link": BASE_LINK,
        "tip_link": TIP_LINK,
        "arm_joint_names": (
            ARM_JOINT_NAMES
        ),
        "chain": chain_report,
        "zero_pose_expected_position_m": (
            EXPECTED_ZERO_POSITION_M.tolist()
        ),
        "zero_pose_position_error_mm": (
            zero_position_error_m
            * 1000.0
        ),
        "poses": results,
    }

    args.report.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    args.report.write_text(
        json.dumps(
            report,
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    print("=== SO-101 FK CHAIN ===")

    for joint in solver.chain:
        lower_text = (
            "-"
            if joint.lower is None
            else f"{joint.lower:.6f}"
        )

        upper_text = (
            "-"
            if joint.upper is None
            else f"{joint.upper:.6f}"
        )

        print(
            f"{joint.parent} "
            f"--[{joint.name}, "
            f"{joint.joint_type}]--> "
            f"{joint.child} "
            f"limits=[{lower_text}, "
            f"{upper_text}] rad"
        )

    print()
    print("=== FK VERIFICATION ===")

    for result in results:
        position = result["position_mm"]
        quaternion = (
            result["quaternion_xyzw"]
        )

        print(
            f"{result['name']}: "
            f"q_deg={result['q_degrees']} | "
            f"position_mm=("
            f"{position[0]:.3f}, "
            f"{position[1]:.3f}, "
            f"{position[2]:.3f}) | "
            f"quat_xyzw=("
            f"{quaternion[0]:.6f}, "
            f"{quaternion[1]:.6f}, "
            f"{quaternion[2]:.6f}, "
            f"{quaternion[3]:.6f})"
        )

    print()
    print(
        "ZERO_POSITION_ERROR_MM="
        f"{zero_position_error_m * 1000.0:.9f}"
    )

    print(
        f"REPORT={args.report.resolve()}"
    )

    print(
        "PASS: offline SO-101 forward "
        "kinematics verified"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
