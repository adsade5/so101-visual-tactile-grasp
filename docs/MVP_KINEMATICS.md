# MVP-1 SO-101 Kinematics

This stage implements a small offline kinematics module for the SO-101 arm. It is meant to be readable, testable, and easy to connect to a later simple motion stage. It does not start ROS2 nodes, open serial ports, or send any hardware command.

## Model

The model reads the frozen URDF:

`data/robot_model/so101/so101_new_calib.urdf`

It follows the chain from `base_link` to `gripper_frame_link` and requires the five arm joints to appear in this exact order:

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`

If the chain or order does not match, the model raises an error instead of guessing.

## Forward Kinematics

FK starts with an identity 4x4 matrix at `base_link`. For each joint in the URDF chain it multiplies:

`T = T * T_origin * T_joint`

`T_origin` is made from the URDF `origin xyz/rpy`. For a revolute joint, `T_joint` is the rotation around the URDF joint axis by the input joint angle. Fixed joints only apply their origin transform.

The final matrix gives the gripper pose in the base frame. The implementation also records each joint axis origin and axis direction in the base frame, because the Jacobian uses those values.

## Jacobian

The geometric Jacobian explains how a small joint movement changes the tool pose. For joint `i`:

`Jv_i = axis_i x (p_tip - p_joint_i)`

`Jw_i = axis_i`

The top three rows describe linear motion of the tip. The bottom three rows describe angular motion. A finite-difference check compares this analytical Jacobian with small FK perturbations.

## Inverse Kinematics

The IK uses damped least squares. Directly inverting the Jacobian can become unstable near singular poses. DLS adds a small damping term, so the update remains well behaved:

`dq = J.T * inverse(J * J.T + lambda^2 * I) * error`

SO-101 has five arm joints, so this stage does not force a full six-degree pose. Instead, the error has:

1. 3D position error.
2. Gripper approach-axis alignment error.

The gripper approach axis is `gripper_frame_link` local `+Z`, confirmed from the previously verified legacy top-down IK. The desired grasp direction is base-frame `-Z`, so the gripper points down toward the table.

`wrist_roll` is not used to force a full roll angle. It is allowed to move only as needed by the DLS solve.

## Seeds And Failures

IK starts from a seed, normally the current or previous known joint posture. This keeps solutions continuous and avoids a complicated global search. If IK cannot meet the position and approach tolerances, the later execution stage must reject the target.

This stage never clips a failed result and calls it successful. A returned solution is valid only when the final FK pose meets tolerance and every joint stays inside the URDF limits.

URDF range is used for offline IK feasibility. Real execution must use the LeRobot Follower calibration range check as the final limit.

## Hardware Boundary

MVP-1 is offline only. It does not parse or modify `my_follower.json`, does not open COM ports, does not enable torque, and does not write goal positions.

In MVP-2, a successful joint target can be converted into a simple per-joint motion sequence, with hardware-range checks added before any real robot command is allowed.
