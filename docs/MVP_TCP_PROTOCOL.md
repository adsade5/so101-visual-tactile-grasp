# MVP TCP JSON Lines Protocol

This document defines the future MVP JSON Lines protocol between ROS2 and the LeRobot-side SO-101 server. Stage MVP-0 documents the protocol only; real hardware communication is not implemented.

Each request is a single JSON object encoded on one line.

## Allowed Commands

### `get_state`

```json
{
  "command": "get_state"
}
```

### `move_joints`

```json
{
  "command": "move_joints",
  "target_rad": [0, 0, 0, 0, 0],
  "speed_rad_s": 0.08
}
```

### `move_joints_sequential`

```json
{
  "command": "move_joints_sequential",
  "target_rad": [0, 0, 0, 0, 0],
  "joint_order": [0, 1, 2, 3, 4],
  "speed_rad_s": 0.08
}
```

### `open_gripper`

```json
{
  "command": "open_gripper"
}
```

### `close_gripper`

```json
{
  "command": "close_gripper"
}
```

### `stop`

```json
{
  "command": "stop"
}
```

## Explicitly Not Allowed

The MVP protocol must not add:

- `plan_id`
- `trajectory_hash`
- active/pending plans
- trajectory cache
- heartbeat negotiation
- shadow execution
- command gate tokens

