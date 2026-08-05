# MVP-3A Hardware Test Preparation

MVP-3A prepares the first tiny real motion test for the SO-101 follower. Codex must not run the motion command. The user runs it manually only after checking power, cable, workspace, and the printed command.

## First Motion

The first test only moves `wrist_roll` by `+2 deg`, waits briefly, then returns to the original wrist roll position.

This joint is chosen because it keeps the shoulder, elbow, wrist flex, and gripper still. A tiny wrist-only move is easier to observe and easier to stop than a full pick-place sequence.

The speed is fixed at `0.04 rad/s` with a `20 Hz` command rate, so each cycle can advance at most `0.002 rad`. The executor does not increase speed automatically.

## Held Joints

Before planning, the current six motor values are read. The plan keeps:

- `shoulder_pan`
- `shoulder_lift`
- `elbow_flex`
- `wrist_flex`
- `gripper`

at their initial values. Only `wrist_roll` changes.

## Calibration Range

The executor reads:

`C:/Users/82053/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`

Targets are checked against the LeRobot calibration range, not just the URDF range. If the `+2 deg` target or return target is outside calibration, the motion is refused. The calibration file is never modified.

## Commands

Configuration check, no serial port:

```powershell
& E:\Anaconda\Scripts\conda.exe run --no-capture-output -p E:\Anaconda\envs_dirs\lerobot python scripts\mvp_so101_hardware_test.py --config config\mvp_hardware.json --config-check
```

Read-only state preflight, opens COM4 but does not write `Goal_Position`:

```powershell
& E:\Anaconda\Scripts\conda.exe run --no-capture-output -p E:\Anaconda\envs_dirs\lerobot python scripts\mvp_so101_hardware_test.py --config config\mvp_hardware.json --read-state
```

Plan only, no serial port:

```powershell
& E:\Anaconda\Scripts\conda.exe run --no-capture-output -p E:\Anaconda\envs_dirs\lerobot python scripts\mvp_so101_hardware_test.py --config config\mvp_hardware.json --plan-only --state-file data\verification\mvp3a_current_state.json
```

Manual motion command, generated but not run by Codex:

```powershell
& E:\Anaconda\Scripts\conda.exe run --no-capture-output -p E:\Anaconda\envs_dirs\lerobot python scripts\mvp_so101_hardware_test.py --config config\mvp_hardware.json --execute --enable-hardware-motion --confirm SMALL_WRIST_ROLL_2DEG
```

## Stop Behavior

Press `Ctrl+C` to stop target progression. The executor prints:

`MOTION STOP REQUESTED`

`NO FURTHER TARGET PROGRESSION`

`POWER OFF SERVO SUPPLY IF NEEDED`

It does not send a large automatic return-to-home motion. If anything looks wrong, close the program and switch off the servo power supply.

## Scope

There is no ROS2 node in this stage, no visual grasping, no TCP server, no command gate, no heartbeat, and no trajectory hash. After this tiny motion is manually verified, MVP-3 can add a small hardware server around the same conservative executor.
