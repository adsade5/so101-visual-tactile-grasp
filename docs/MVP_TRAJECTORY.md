# MVP-2 Sequential Joint Trajectory

MVP-2 uses a deliberately simple offline trajectory: one joint moves at a time, in the fixed order `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`.

This is slower than synchronized multi-joint motion, but it is much easier to debug. When something looks wrong, there is only one active joint to inspect. That is useful before moving toward real hardware.

## Single-Joint Profile

Each moving joint uses a low-speed trapezoidal velocity curve:

1. Start at zero velocity.
2. Accelerate with `0.20 rad/s^2`.
3. Cruise at `0.08 rad/s` when the move is long enough.
4. Decelerate back to zero at the target.

For short moves, there is not enough distance to reach the fixed cruise speed. In that case the profile becomes triangular: accelerate, then immediately decelerate.

The sampled trajectory runs at `20 Hz`. The first sample is exactly the start position with zero velocity. The last sample is exactly the target position with zero velocity and zero acceleration.

## Sequential Motion

For a full arm target, the generator compares the start and target joint arrays. It moves each joint in order while all other joints hold their last value. After a joint finishes, it inserts a short `0.20 s` settle segment before the next joint starts.

The complete pick-place demo is built from IK targets:

1. reference to pregrasp
2. pregrasp to descend
3. descend to lift
4. lift to place

There is also a `0.5 s` static pause between these four stages.

## Limits

Offline checks use the frozen URDF lower and upper limits. The generator does not expand limits, does not clip an invalid target into range, and does not mark a limit violation as valid.

Real execution must still use the LeRobot Follower calibration range check as the final hardware limit.

## Curves

The verification script writes CSV and PNG files under:

`data/verification/mvp2_trajectory`

The position plot shows each joint stepping through its portion of the sequence. The velocity plot should show one joint moving at a time with simple acceleration, possible cruise, and deceleration. The acceleration plot is piecewise constant and may jump at sample boundaries.

The TCP path is computed with MVP-1 FK for visualization only. MVP-2 does not perform collision checking, self-collision checking, workspace path safety, torque/load analysis, or command-gate validation.

MVP-3 should begin with very small single-joint hardware-server checks before any larger real motion is attempted.
