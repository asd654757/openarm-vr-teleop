# Pico 4 OpenArm Route vs Official LeRobot OpenArm Route

This document compares the current local Pico 4 / `teleop_xr` / ROS2 workflow
with the official LeRobot OpenArm workflow.

Official reference:

```text
https://huggingface.co/docs/lerobot/openarm
```

OpenArmX reference considered:

```text
https://github.com/openarmx/openarmx_vla/tree/6.0_basic/openarmx_lerobot
```

Important: OpenArmX is a different robot stack and does not use the official
OpenArm Damiao motor path. Borrow workflow ideas only; do not copy its motor,
CAN, or controller assumptions into the official OpenArm hardware path.

## Summary

| Item | Current Pico 4 route | Official LeRobot OpenArm route |
| --- | --- | --- |
| Main purpose | XR teleoperation, simulation bring-up, Pico-based dataset collection | Standard OpenArm hardware control and dataset collection |
| Teleoperator | Pico 4 controllers through `teleop_xr` | Physical OpenArm leader arm |
| Robot target | ROS2 OpenArm fake hardware first | Damiao motor OpenArm follower over CAN |
| LeRobot robot type | `bi_openarm_ros2_follower` | `openarm_follower`, `bi_openarm_follower` |
| LeRobot teleop type | `pico_openarm_ik` | `openarm_leader`, `bi_openarm_leader` |
| Transport | WebXR/WebSocket + ROS2 topics | CAN/CAN-FD |
| Action source | `teleop_xr` IK output on `/joint_trajectory` | Leader arm joint positions |
| Dataset action | OpenArm joint targets from Pico IK | OpenArm joint targets from leader arm |
| Current maturity | Working in simulation/basic Pico teleop | Official supported hardware path |
| Real hardware readiness | Needs safety/deadman/CAN validation before use | Intended path after CAN setup and calibration |

## What OpenArmX Adds As A Useful Reference

The `openarmx_lerobot` package is mostly a composed ROS2 launch and usage guide.
It does not implement the low-level robot control or LeRobot robot classes in
that folder. The useful ideas for our workflow are:

- Split the system into explicit terminals/processes: robot bringup, Pico bridge,
  VR teleop, cameras, LeRobot recorder.
- Use `forward_position_controller` for the LeRobot recording path, so LeRobot is
  the process that sends actions.
- Use local/offline dataset collection by default:
  `HF_HUB_OFFLINE=1` and `--dataset.push_to_hub=false`.
- Enable `--display_data=true` during early collection to catch camera/state
  mistakes immediately.
- Treat cameras as first-class: left wrist, right wrist, and head/front camera.
- Use low camera defaults such as `424x240@15fps` during bring-up to reduce load.
- Document recording hotkeys: right arrow saves the current episode, left arrow
  discards/re-records, `Esc` exits.
- Keep camera serial numbers and camera model types explicit in launch commands.
- Use the same `ROS_DOMAIN_ID` across machines when camera/robot/recorder are
  split across computers.

Things not to borrow directly:

- `openarmx_bringup`, `openarmx_follower_ros2`, `openarmx_leader_ros2`,
  `pico_pose_bridge`, and `openarmx_teleop_by_pico` are OpenArmX-specific.
- Its `control_mode:=mit` and CAN setup are not a replacement for the official
  OpenArm Damiao setup.
- Its camera publisher launch is CC-BY-NC-SA licensed, so we should not copy the
  source into this LeRobot tree. Reimplement only the ideas we need.

## Current Local Pico 4 Route

Data/control flow:

```text
Pico 4 browser
  -> teleop_xr WebXR session
  -> teleop_xr OpenArm IK
  -> /joint_trajectory
  -> LeRobot pico_openarm_ik teleoperator
  -> bi_openarm_ros2_follower
  -> ROS2 OpenArm fake hardware
  -> LeRobot dataset
```

Main files:

```text
~/OPENARM/teleop_xr
~/OPENARM/lerobot/src/lerobot/teleoperators/pico_openarm_ik/
~/OPENARM/lerobot/src/lerobot/robots/openarm_ros2_follower/
~/OPENARM/lerobot/examples/openarm_xr/PICO4_OPENARM_TELEOP.md
~/OPENARM/lerobot/examples/openarm_xr/PICO4_TO_VLA_SIM_PIPELINE.md
```

Strengths:

- Does not require physical OpenArm leader arms.
- Good for Pico 4 teleoperation experiments.
- Good for ROS2 fake hardware and RViz validation.
- Can record LeRobot-style action/observation data from Pico motion.
- Uses official `teleop_xr` OpenArm IK instead of a hand-tuned Cartesian mapper.
- Can adopt OpenArmX-style multi-camera and offline recording conventions.

Limitations:

- Pico controller pose quality and button/deadman mapping must be validated.
- Real hardware safety is not yet validated.
- Current primary target is ROS2 fake hardware, not CAN motors.
- Camera recording still needs to be configured for useful imitation learning.
- `--no-require-deadman` is acceptable only for simulation bring-up.

## Official LeRobot OpenArm Route

Data/control flow:

```text
OpenArm leader arm
  -> openarm_leader / bi_openarm_leader
  -> openarm_follower / bi_openarm_follower
  -> Damiao motors over CAN/CAN-FD
  -> LeRobot dataset
```

Official setup highlights:

```bash
lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=test --interfaces=can0,can1
```

Install Damiao support:

```bash
pip install -e ".[damiao]"
```

Single-arm calibration example:

```bash
lerobot-calibrate \
  --robot.type=openarm_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.id=my_openarm_follower
```

Bimanual teleoperation example:

```bash
lerobot-teleoperate \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.id=my_bimanual_follower \
  --teleop.type=bi_openarm_leader \
  --teleop.left_arm_config.port=can2 \
  --teleop.right_arm_config.port=can3 \
  --teleop.id=my_bimanual_leader
```

Strengths:

- Officially supported LeRobot hardware path.
- Direct Damiao motor control through CAN/CAN-FD.
- Has standard calibration, motor configuration, and safety parameters.
- Best baseline before real OpenArm deployment.

Limitations:

- Requires OpenArm leader hardware for standard teleoperation.
- Requires working CAN adapters and CAN-FD setup.
- Linux only.
- Less convenient for XR controller experiments.

## What We Already Added Locally

### ROS2 OpenArm follower bridge

LeRobot robot types:

```text
openarm_ros2_follower
bi_openarm_ros2_follower
```

Purpose:

- Use OpenArm ROS2 fake hardware as a LeRobot robot.
- Expose OpenArm joints in LeRobot units: arm joints in degrees, gripper in degrees.
- Publish to ROS2 forward position controllers.
- Read observations from `/joint_states`.

### Pico OpenArm IK teleoperator

LeRobot teleop type:

```text
pico_openarm_ik
```

Purpose:

- Subscribe to `teleop_xr` official OpenArm IK output on `/joint_trajectory`.
- Convert ROS radians to LeRobot degree actions.
- Feed actions into LeRobot `lerobot_record` or `lerobot_teleoperate`.

### teleop_xr local patches

Purpose:

- Provide frontend `dist` assets in the source checkout.
- Avoid empty WebRTC video offers when no video source is configured.
- Use Pico controller `pose` when `gripPose` is missing.
- Add `--no-require-deadman` for simulation bring-up.

## Recommended Workflow From Here

### Stage 1: Finish Pico data collection in simulation

Use this when the goal is to collect early Pico demonstrations without risking
hardware.

1. Start OpenArm fake hardware with forward position controllers.
2. Start `teleop_xr` official OpenArm IK.
3. Open Pico 4 browser at `https://<PC_IP>:4443`.
4. Record with `lerobot_record` using `pico_openarm_ik`.
5. Add cameras after proprioception-only recording is stable.

Recording command:

```bash
HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_record \
  --robot.type=bi_openarm_ros2_follower \
  --robot.id=openarm_pico4_sim \
  --teleop.type=pico_openarm_ik \
  --teleop.trajectory_topic=/joint_trajectory \
  --dataset.repo_id=local/openarm_pico4_sim \
  --dataset.single_task="Move OpenArm with Pico 4 controllers" \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=5 \
  --dataset.fps=30 \
  --dataset.video=false \
  --dataset.push_to_hub=false \
  --display_data=true
```

OpenArmX-style camera bring-up recommendation:

```bash
lerobot-find-cameras realsense
```

Then add explicit cameras to the same recording command:

```bash
--robot.cameras='{
  left_wrist: {type: intelrealsense, serial_number_or_name: "LEFT_SERIAL", width: 424, height: 240, fps: 15},
  right_wrist: {type: intelrealsense, serial_number_or_name: "RIGHT_SERIAL", width: 424, height: 240, fps: 15},
  front: {type: intelrealsense, serial_number_or_name: "HEAD_SERIAL", width: 424, height: 240, fps: 15},
}'
```

If cameras are on another machine, keep all terminals on the same ROS domain:

```bash
export ROS_DOMAIN_ID=42
```

For now, prefer LeRobot's built-in `intelrealsense`, `opencv`, or `zmq` camera
configs. A direct ROS2 image-topic camera adapter can be added later if we want
to mirror OpenArmX's `/cam_left/color/image` style publisher.

### Stage 2: Validate official CAN hardware baseline

Use this before allowing Pico to command real motors.

1. Install Damiao support.
2. Configure CAN/CAN-FD.
3. Test motor communication.
4. Calibrate official `openarm_follower`.
5. Run official leader/follower teleoperation if leader hardware is available.
6. Confirm emergency stop and physical safety setup.

### Stage 3: Bridge Pico to real OpenArm

Only after Stage 1 and Stage 2 are stable:

```text
Pico 4
  -> teleop_xr OpenArm IK
  -> /joint_trajectory
  -> pico_openarm_ik
  -> bi_openarm_follower
  -> CAN/CAN-FD OpenArm hardware
```

Required before trying:

- Re-enable deadman control.
- Confirm Pico button mapping from `/xr/controller_* /joy`.
- Set conservative `max_relative_target`.
- Reduce speed/gains.
- Start with one arm if possible.
- Keep emergency stop ready.
- Confirm no second process is also commanding the robot.

## Gaps To Fill

| Gap | Why it matters | Suggested next step |
| --- | --- | --- |
| Camera config for recording | Vision is needed for useful imitation learning | Add left/right/front RealSense cameras at `424x240@15fps` first |
| Deadman mapping for Pico | Required before hardware | Identify stable Pico button index from `/xr/controller_right/joy` |
| Real CAN validation | Required before real OpenArm control | Run `lerobot-setup-can --mode=test` |
| Hardware safety limits | Prevent large unsafe jumps | Configure `max_relative_target` and test low speed |
| Single-arm hardware trial | Reduces risk | Try `openarm_follower` right arm first |
| Dataset inspection | Ensures action/observation are aligned | Load a short local dataset and plot action/state |
| Optional ROS2 image bridge | Useful if cameras publish ROS2 topics on another machine | Add a LeRobot `ros2_image` camera config only if built-in camera configs are insufficient |

## Decision Guide

Use the current Pico route when:

- You want XR teleoperation.
- You want simulation/RViz first.
- You do not have physical leader arms.
- You want to collect early Pico demonstrations.

Use the official route when:

- You are controlling real OpenArm hardware.
- You need a supported CAN/CAN-FD baseline.
- You have OpenArm leader hardware.
- You are debugging motors, calibration, or CAN.

Use both when:

- You want Pico teleoperation on real OpenArm.
- First validate official CAN hardware.
- Then swap the official leader teleoperator for `pico_openarm_ik`.
