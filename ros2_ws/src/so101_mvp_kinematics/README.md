# so101_mvp_kinematics

Simplified offline SO-101 MVP kinematics package.

Implemented modules:

- `model.py`: frozen URDF chain parsing.
- `transforms.py`: small rotation and homogeneous transform helpers.
- `fk.py`: forward kinematics from five arm joints to `gripper_frame_link`.
- `jacobian.py`: geometric Jacobian and finite-difference checks.
- `ik.py`: damped least-squares position IK with gripper-down alignment.
- `joint_limits.py`: URDF-limit checks and clamping helpers.

This package is offline only. It does not open serial ports or send robot commands.
