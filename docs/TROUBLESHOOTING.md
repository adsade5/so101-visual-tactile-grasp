# Troubleshooting

Common issues encountered during setup and operation of the SO-101 visual-tactile grasp system.

---

## Hardware Connection Issues

### COM4 Cannot Open

**Symptom:** Server error `cannot open COM4` or `ROBOT_CONNECTED` never appears.

**Causes & solutions:**
1. Another program holds COM4 open (serial monitor, previous server instance, Arduino IDE).
   - Close all other programs that might access COM4.
   - Check Windows Device Manager → Ports (COM & LPT) for the device.
2. USB cable disconnected or loose.
   - Reconnect the SO-101 USB cable.
   - Verify the device appears as "USB-Enhanced-SERIAL CH343" in Device Manager.
3. Wrong COM port number.
   - Verify the actual port in Device Manager. If it changed (e.g., COM5), update `follower_port` in `config/mvp_hardware.json`.

### COM8 Cannot Open

**Symptom:** Server error `cannot open COM8` or `TACTILE_SERIAL_OPENED` never appears.

**Causes & solutions:**
1. Another program holds COM8 open.
   - Close serial monitors, previous server instances.
2. USB cable disconnected.
   - Reconnect the FlexiTac USB cable.
   - Verify the device appears as "USB-SERIAL CH340" in Device Manager.
3. Wrong port number.
   - If the FlexiTac appears on a different COM port, update `tactile.port` in `config/mvp_hardware.json`.
   - **Note:** The server enforces COM8 for the `direct_serial` source. If you must use a different port, you'll need to modify `TactileRuntime.connect()` in `mvp_so101_server.py`.

### Tactile Baseline Fails

**Symptom:** Server starts but `TACTILE_BASELINE_COMPLETED` never appears, or tactile readings are noisy.

**Causes & solutions:**
1. Sensor was touched during baseline capture.
   - **Do not touch the FlexiTac sensor** during the first ~1-2 seconds after `TACTILE_SERIAL_OPENED`. The baseline captures 30 frames of the resting state. Any contact during this period will corrupt the baseline.
   - Restart the server and keep the sensor untouched.
2. Sensor surface has residual pressure.
   - Ensure nothing is pressing against the sensor.
3. Baud rate mismatch.
   - Verify `tactile.baudrate` is `2000000` in `config/mvp_hardware.json`.

---

## Communication Issues

### Bridge Cannot Connect

**Symptom:** Terminal 2 (Bridge) shows no `BRIDGE_TCP_CONNECTED` message.

**Checklist:**
1. Is the server running? Check Terminal 1 shows `TCP_SERVER_LISTENING`.
2. Is Zenoh running? Check Terminal 0 shows normal startup messages.
3. Is the port correct? Both server and bridge must use port 8770.
4. Is another TCP client connected? The server accepts only one client. Kill any stale bridge processes.
5. Firewall: localhost (127.0.0.1) connections should not be blocked, but verify no firewall rule is interfering.

**Quick diagnostic:** Run `netstat -an | findstr 8770` in a command prompt. You should see `LISTENING` on `127.0.0.1:8770`.

### Server Shows "TCP Client Disconnected" Repeatedly

**Cause:** The bridge process crashed or was terminated, and a new bridge instance is trying to connect (or a stale client is holding the connection).

**Solution:**
1. Stop Terminal 2 (Bridge) with Ctrl+C.
2. Wait for the server to show "Waiting for new connection."
3. Restart Terminal 2.

---

## Vision Issues

### No Object Pose

**Symptom:** `mvp_visual_grasp.py --plan-only` reports "no object pose" or timeout waiting for `/object_pose_base`.

**Causes & solutions:**
1. Camera not connected or wrong camera index.
   - Verify the camera is connected and recognized by Windows.
   - Check `config/camera.yaml` for the correct camera index.
2. ArUco marker not visible.
   - Ensure the object with the ArUco marker is within the camera's field of view.
   - Check lighting conditions — glare or shadows can prevent detection.
   - Verify the marker dictionary and ID match `config/object_marker.json`.
3. Vision nodes not running or crashed.
   - Check Terminal 3 for error messages.
   - Verify `ros2 node list` shows `object_pose_node` and `workspace_to_base_node`.
   - Verify `ros2 topic echo /object_pose_base` shows data.

### Object Pose Drifts or Is Inaccurate

**Causes & solutions:**
1. Camera moved after calibration.
   - Re-run camera calibration if the camera position changed.
2. Workspace calibration stale.
   - Re-run workspace-to-base calibration if the table or robot base moved.
3. ArUco marker partially occluded.
   - Ensure clear line of sight from camera to marker.

---

## Kinematics Issues

### Plan-Only Fails IK

**Symptom:** Plan-only reports `success=false` with IK solver failure.

**Causes & solutions:**
1. Object outside reachable workspace.
   - The SO-101 has a limited reach. If the object is too far from the base, IK will fail.
   - Move the object closer to the robot base (within ~0.25 m radius).
   - Check `mvp4a_last_pregrasp_diagnostic.json` (in `data/verification/final/`) for the last pregrasp IK attempt details.
2. Workspace calibration error.
   - Verify `config/workspace_to_base.json` contains a valid transform.
3. Pregrasp offset too far above or below object.
   - Check `config/mvp_pregrasp.yaml` approach parameters.

---

## Grasp Execution Issues

### Gripper Closes But No Lift

**Symptom:** The arm descends and the gripper closes, but the arm does not lift.

**This is expected behavior when no tactile contact is confirmed.**

The lift is **gated on tactile contact**. If the gripper closed without touching the object (e.g., object misplaced, gripper misaligned), there will be no lift.

**Causes:**
1. Object not positioned correctly under the gripper.
2. X-axis offset needs adjustment for your setup.
3. FlexiTac not making contact with the object surface.
4. Tactile thresholds too high (check `tactile.contact_on_threshold` in config).

**What to check:**
- The terminal output will show `tactile_contact_confirmed: false` and `lift_executed: false`.
- Run `--tactile-test` to verify the sensor is detecting contact.
- Manually touch the FlexiTac to confirm contact detection is working.

### Wrong Grasp Position (Grasping Behind Object)

**Symptom:** The arm grasps ~2 cm behind the visual target.

This is the known systematic bias that the X-axis offset corrects.

**Check:**
1. `grasp_x_offset_m` in `config/mvp_hardware.json` should be `+0.020`.
2. In your setup, verify whether `+X` points forward (away from robot base) by checking the URDF or observing arm motion.
3. If your setup differs from the validated setup, adjust `grasp_x_offset_m` accordingly.

**Important:** The +20 mm value was calibrated for the specific validated setup. It may need adjustment for different robot mounting positions, camera angles, or workspace calibrations.

### Gripper Closes Too Far (Beyond Contact)

**Symptom:** Gripper continues closing after making contact with the object, potentially crushing it.

**Check:**
1. Tactile contact detection is working — run `--tactile-test` first.
2. `tactile.contact_confirm_frames` is set to 3 in config — if too high, transient contacts may be missed.
3. `tactile.contact_on_threshold` may be too high — try lowering to 30.0.
4. Server-side `stop_gripper_on_tactile_contact` flag is being sent — check bridge debug output.

### Arm Does Not Move

**Symptom:** Plan-only passes but execute produces no motion.

**Causes:**
1. `--confirm VISUAL_GRASP` flag missing or misspelled.
2. `hardware_motion_enabled` is `false` in config (must be `true` for execute).
3. Server started without `--enable-hardware-motion` flag.
4. Bridge launched with `enable_hardware_motion:=false`.

---

## ROS2 / Zenoh Issues

### Zenoh Router Won't Start

**Symptom:** Terminal 0 hangs or crashes.

**Causes:**
1. Another Zenoh router instance is already running.
   - Check for existing `rmw_zenohd` processes and kill them.
2. ROS2 environment not properly sourced.
   - Verify `rmw_zenoh_cpp` is installed in the Pixi/ROS2 environment.
   - Run `ros2 run rmw_zenoh_cpp rmw_zenohd --help` to verify.

### ROS2 Nodes Can't Discover Each Other

**Symptom:** Nodes start but don't see each other's topics/services.

**Causes:**
1. Zenoh router not running.
   - Always start Terminal 0 (Zenoh) first.
2. Different ROS_DOMAIN_ID across terminals.
   - All terminals must share the same domain (default is fine if not set).

---

## Environment Issues

### "conda.exe not found" or Python Import Errors

**Cause:** Conda environment paths are hardcoded in helper scripts.

**Solutions:**
- For `open_mvp4e_terminals.ps1`: Update `$CondaExe` and the Conda environment path.
- For `audit/run_in_ros2_lyrical.ps1`: Verify `pixi.exe` location and `C:\pixi_ws` path.
- For `mvp_so101_server.py`: Verify LeRobot is installed and importable in your Conda environment.

### ROS2 Colcon Build Fails

**Symptom:** Build errors when running `colcon build`.

**Causes:**
1. Not in ROS2 Lyrical environment.
   - Always use `audit/run_in_ros2_lyrical.ps1` to wrap build commands.
2. Missing dependencies.
   - Ensure `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_srvs` are available.
3. CMake issues.
   - Only `so101_description` uses CMake; all other packages are pure Python.

---

## Legacy Launcher Debugging

<details>
<summary>Historical one-launch issues (not part of final workflow)</summary>

The deprecated `scripts/launch_mvp4e_system.ps1` had issues with:
- **Command file syntax**: Generated `.cmd` files sometimes had escaping issues with `&&` chaining.
- **Process readiness detection**: Log parsing for `BRIDGE_TCP_CONNECTED` etc. was fragile.
- **PID tree management**: Process tracking across nested PowerShell/conda/ROS2 launches was unreliable.
- **WinError classification**: Windows error codes from subprocess failures required custom classification.

These issues motivated the switch to the simplified manual multi-terminal workflow. The legacy launcher is retained only for debugging reference and is not recommended for use.

</details>
