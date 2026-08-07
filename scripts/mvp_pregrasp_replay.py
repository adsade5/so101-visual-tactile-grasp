from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT_ROOT / "ros2_ws" / "src"
for package_path in (
    ROS_SRC / "so101_mvp_control",
    ROS_SRC / "so101_mvp_kinematics",
):
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from so101_mvp_control.pregrasp_planner import (
    PoseSnapshot,
    compute_pregrasp_plan,
    create_model,
    make_failure_message,
    make_success_message,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--pregrasp-height-m", type=float, default=0.08)
    args = parser.parse_args()

    now = time.monotonic()
    model = create_model(PROJECT_ROOT)
    object_pose = PoseSnapshot(
        frame_id=str(args.frame),
        position_m=np.asarray([args.x, args.y, args.z], dtype=np.float64),
        received_monotonic_s=now,
    )
    plan = compute_pregrasp_plan(
        model=model,
        object_pose=object_pose,
        joint_state=None,
        base_frame="base_link",
        now_monotonic_s=now,
        max_object_pose_age_s=1.0,
        pregrasp_height_m=float(args.pregrasp_height_m),
        use_joint_state_seed=True,
        max_joint_state_age_s=1.0,
    )

    print(f"object_pose_base_m={plan.diagnostic_dict()['object_pose_base']}")
    print(f"requested_pregrasp_pose_base_m={plan.diagnostic_dict()['requested_pregrasp_xyz_m']}")
    print(f"selected_pregrasp_pose_base_m={plan.diagnostic_dict()['selected_pregrasp_xyz_m']}")
    print(f"selected_candidate_index={plan.selected_candidate_index}")
    print(
        "selected_offset_m="
        f"{None if plan.selected_offset_m is None else plan.selected_offset_m.tolist()}"
    )
    print(f"solution_type={plan.solution_type}")
    print(f"target_radius_xy_m={plan.target_radius_xy_m}")
    print(f"target_distance_3d_m={plan.target_distance_3d_m}")
    print(f"approx_max_reach_m={plan.approx_max_reach_m}")
    for attempt in plan.attempt_results:
        print(
            "IK_ATTEMPT "
            f"index={attempt.attempt_index} "
            f"source={attempt.seed_source} "
            f"success={str(attempt.solver_success).lower()} "
            f"reason={attempt.solver_reason} "
            f"iterations={attempt.iterations} "
            f"position_error_m={attempt.position_error_m} "
            f"approach_error_deg={attempt.approach_error_deg} "
            f"joint_limit_valid={str(attempt.joint_limit_valid).lower()} "
            f"solution_rad={None if attempt.solution_rad is None else attempt.solution_rad.tolist()} "
            f"final_reason={attempt.final_reason}"
        )

    if plan.success:
        print(make_success_message(plan))
        print(f"selected_solution_rad={plan.joint_positions_rad.tolist()}")
        print("NO_HARDWARE_COMMAND_SENT")
        return 0

    print(make_failure_message(plan))
    print("NO_HARDWARE_COMMAND_SENT")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
