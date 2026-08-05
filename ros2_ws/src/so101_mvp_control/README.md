# so101_mvp_control

Minimal control-side helpers for the simplified SO-101 MVP.

The existing MVP-0 ROS2 nodes only announce that hardware motion is disabled. They do not connect to the robot, open serial ports, publish motion commands, or depend on the legacy command gate or shadow executor.

Stage MVP-2 adds offline sequential joint trajectory helpers:

- `simple_trajectory.py`
- `trajectory_validation.py`

These helpers generate and validate low-speed sampled joint curves. They do not start ROS2 nodes or send robot commands.
