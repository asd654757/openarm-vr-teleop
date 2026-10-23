# Pico 4 Teleop OpenArm With teleop_xr

This note records the working reproduction path for Pico 4 WebXR teleoperation
of OpenArm in ROS2 simulation.

For a side-by-side comparison with the official LeRobot OpenArm CAN workflow,
see:

```text
examples/openarm_xr/PICO4_VS_OFFICIAL_OPENARM.md
```

For the end-to-end Pico data -> VLA training -> simulation rollout workflow,
see:

```text
examples/openarm_xr/PICO4_TO_VLA_SIM_PIPELINE.md
```

That comparison also includes OpenArmX notes. OpenArmX is useful as a workflow
reference for Pico, cameras, and LeRobot recording, but its motor/control stack
is different from official OpenArm Damiao hardware.

There are two routes:

- Recommended official route: `teleop_xr --mode ik --robot-class openarm`
- Simple debug route: raw XR pose/joy topics mapped to joints by
  `openarm_xr_mapper.py`

Use simulation / fake hardware first. Do not switch to real hardware until the
deadman, gains, joint limits, and emergency stop path are verified.

## Paths

```bash
~/OPENARM/teleop_xr
~/OPENARM/lerobot
~/OPENARM/ros2_ws
```

Python environment:

```bash
conda activate oplerobot310
```

Always clean mixed ROS environment variables before using ROS2:

```bash
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
```

## One-Time Fixes / Setup

### teleop_xr frontend dist

If running from `~/OPENARM/teleop_xr` source tree and this error appears:

```text
RuntimeError: Directory '/home/w/OPENARM/teleop_xr/teleop_xr/dist' does not exist
```

Copy the installed frontend assets:

```bash
cp -a ~/anaconda3/envs/oplerobot310/lib/python3.10/site-packages/teleop_xr/dist \
  ~/OPENARM/teleop_xr/teleop_xr/
```

### IK dependencies

Official OpenArm IK needs `jax`, `pyroki`, `ballpark`, `yourdfpy`, etc.

Install non-GitHub dependencies:

```bash
/home/w/anaconda3/envs/oplerobot310/bin/python -m pip install \
  'jax>=0.6.2' jaxlie jaxls \
  'spatialmath-python>=1.1.15' \
  'viser>=1.0.21' \
  'yourdfpy>=0.0.60' \
  pin gitpython xacro filelock 'trimesh[easy]>=4.0.0'
```

Install `pyroki` and `ballpark` from source:

```bash
git clone --depth 1 https://github.com/chungmin99/pyroki.git /tmp/pyroki
git clone --depth 1 https://github.com/chungmin99/ballpark.git /tmp/ballpark
/home/w/anaconda3/envs/oplerobot310/bin/python -m pip install /tmp/pyroki /tmp/ballpark
```

Verify:

```bash
/home/w/anaconda3/envs/oplerobot310/bin/python - <<'PY'
mods = ["jax", "jaxlie", "jaxls", "pyroki", "ballpark", "yourdfpy"]
for m in mods:
    mod = __import__(m)
    print(m, "OK", getattr(mod, "__version__", ""))
PY
```

Verify official robot registration:

```bash
cd ~/OPENARM/teleop_xr
python -m teleop_xr.ros2 --list-robots
```

Expected:

```text
openarm: teleop_xr.ik.robots.openarm:OpenArmRobot
```

## Recommended: Official OpenArm IK Route

This route uses the official `teleop_xr` OpenArm model:

```text
Pico 4 WebXR
  -> teleop_xr.ros2 --mode ik --robot-class openarm
  -> /joint_trajectory
  -> openarm_ik_trajectory_splitter.py
  -> /left_joint_trajectory_controller/joint_trajectory
  -> /right_joint_trajectory_controller/joint_trajectory
  -> OpenArm fake hardware
  -> RViz
```

### Terminal 1: OpenArm fake hardware with trajectory controllers

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=joint_trajectory_controller
```

Check controllers:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
left_joint_trajectory_controller
right_joint_trajectory_controller
left_gripper_controller
right_gripper_controller
```

### Terminal 2: Split IK trajectory to left/right controllers

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

python examples/openarm_xr/openarm_ik_trajectory_splitter.py
```

The splitter subscribes:

```text
/joint_trajectory
```

and publishes:

```text
/left_joint_trajectory_controller/joint_trajectory
/right_joint_trajectory_controller/joint_trajectory
```

### Terminal 3: Start official OpenArm IK WebXR bridge

```bash
conda activate oplerobot310
cd ~/OPENARM/teleop_xr

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode ik \
  --robot-class openarm \
  --output-topic /joint_trajectory \
  --no-require-deadman
```

Notes:

- First start may take tens of seconds on CPU because JAX compiles IK problems.
- `--no-require-deadman` is for simulation only. Pico 4 button data was not
  reliable during the first bring-up.
- Expected logs include:

```text
Successfully fetched URDF
Initializing OpenArmRobot
Created RobotCollision
Building optimization problem
Server started at 0.0.0.0:4443
```

### Pico 4

Open Pico 4 browser:

```text
https://<PC_IP>:4443
```

Example from the test machine:

```text
https://192.168.3.11:4443
```

Accept the self-signed certificate if prompted and enter XR/VR session.

Successful session log:

```text
Session started: immersive-ar
```

### IK route verification

Check XR topics:

```bash
ros2 topic list | grep /xr
```

Expected:

```text
/xr/controller_left/pose
/xr/controller_left/joy
/xr/controller_right/pose
/xr/controller_right/joy
```

Check controller pose:

```bash
ros2 topic echo /xr/controller_right/pose --once
```

Check IK output:

```bash
ros2 topic echo /joint_trajectory --once
```

Check splitter output:

```bash
ros2 topic echo /right_joint_trajectory_controller/joint_trajectory --once
```

If both trajectory topics publish, RViz should show OpenArm moving.

## Record Pico 4 Data as a LeRobot Dataset

Yes: Pico 4 can be used as the teleoperation source for LeRobot data. The
recommended data path is:

```text
Pico 4 WebXR
  -> teleop_xr official OpenArm IK
  -> /joint_trajectory
  -> LeRobot pico_openarm_ik teleoperator
  -> bi_openarm_ros2_follower
  -> OpenArm forward position controllers
  -> LeRobot dataset action/observation frames
```

For recording with LeRobot, do not run `openarm_ik_trajectory_splitter.py`.
LeRobot should be the process that sends the IK action to the robot, otherwise
two processes can fight over control.

If `datasets` is missing in `oplerobot310`, install it once:

```bash
conda run -n oplerobot310 python -m pip install datasets
```

### Relationship to the official LeRobot OpenArm path

The official LeRobot OpenArm documentation uses the hardware CAN path:

```text
openarm_leader / bi_openarm_leader
  -> openarm_follower / bi_openarm_follower
  -> Damiao motors over CAN/CAN-FD
```

This Pico route keeps the same LeRobot dataset idea, but replaces the physical
leader arm with Pico 4 plus `teleop_xr` IK:

```text
pico_openarm_ik
  -> bi_openarm_ros2_follower for ROS2 simulation
  -> later: bi_openarm_follower for CAN hardware, after safety validation
```

For real hardware, first complete the official CAN setup and diagnostics:

```bash
lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=test --interfaces=can0,can1
```

Then verify the standard OpenArm follower path with conservative limits before
connecting Pico:

```bash
lerobot-calibrate \
  --robot.type=openarm_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.id=my_openarm_follower
```

Do not run Pico on real hardware with `--no-require-deadman`. Re-enable a
verified deadman button, set `max_relative_target`, and keep emergency stop
available.

### Terminal 1: OpenArm fake hardware with forward controllers

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

### Terminal 2: Start official OpenArm IK, publish only `/joint_trajectory`

```bash
conda activate oplerobot310
cd ~/OPENARM/teleop_xr

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode ik \
  --robot-class openarm \
  --output-topic /joint_trajectory \
  --no-require-deadman
```

Open Pico 4 browser at:

```text
https://<PC_IP>:4443
```

### Terminal 3: Record with LeRobot

First record proprioception-only data to verify the loop:

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

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

This records:

```text
observation.state: OpenArm joint states from /joint_states
action: Pico/teleop_xr IK joint targets from /joint_trajectory
```

For imitation learning, add cameras under `--robot.cameras=...` or the
left/right arm camera config, then set `--dataset.video=true`.

OpenArmX-inspired camera defaults for early collection:

```bash
lerobot-find-cameras realsense
```

Then add three explicit cameras to the record command:

```bash
--robot.cameras='{
  left_wrist: {type: intelrealsense, serial_number_or_name: "LEFT_SERIAL", width: 424, height: 240, fps: 15},
  right_wrist: {type: intelrealsense, serial_number_or_name: "RIGHT_SERIAL", width: 424, height: 240, fps: 15},
  front: {type: intelrealsense, serial_number_or_name: "HEAD_SERIAL", width: 424, height: 240, fps: 15},
}' \
--dataset.video=true
```

Useful collection keys:

```text
Right arrow: save current episode and enter reset
Left arrow: discard current episode and re-record
Esc: stop recording
```

If robot, Pico, cameras, and recorder are split across machines, use the same
ROS domain everywhere:

```bash
export ROS_DOMAIN_ID=42
```

### Recording Checks

Before starting `lerobot_record`, verify:

```bash
ros2 topic echo /joint_trajectory --once
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

Expected controllers for LeRobot recording:

```text
joint_state_broadcaster
left_forward_position_controller
right_forward_position_controller
left_gripper_controller
right_gripper_controller
```

If `lerobot_record` errors while creating calibration/cache folders in a
restricted environment, pass writable paths:

```bash
--robot.calibration_dir=/tmp/lerobot_calibration/robots \
--teleop.calibration_dir=/tmp/lerobot_calibration/teleoperators
```

## Debug Route: Raw XR Pose/Joy Joint Mapper

This route was useful for first bring-up before official IK was ready:

```text
Pico 4 WebXR
  -> teleop_xr.ros2 --mode teleop
  -> /xr/controller_* pose/joy
  -> openarm_xr_mapper.py
  -> /left_forward_position_controller/commands
  -> /right_forward_position_controller/commands
```

### Terminal 1: OpenArm fake hardware with forward position controllers

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

Expected active controllers:

```text
joint_state_broadcaster
left_forward_position_controller
right_forward_position_controller
left_gripper_controller
right_gripper_controller
```

### Terminal 2: Raw teleop_xr bridge

```bash
conda activate oplerobot310
cd ~/OPENARM/teleop_xr

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode teleop
```

### Terminal 3: Raw mapper

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

python examples/openarm_xr/openarm_xr_mapper.py \
  --no-require-deadman \
  --yaw-gain-deg-per-m 80 \
  --reach-gain-deg-per-m 70 \
  --lift-gain-deg-per-m 80 \
  --wrist-gain-deg 20 \
  --thumbstick-x-axis 2 \
  --thumbstick-y-axis 3
```

The observed Pico 4 thumbstick axes were:

```text
axes[2], axes[3]
```

The mapper now supports axis and sign tuning:

```bash
--yaw-axis x/y/z
--reach-axis x/y/z
--lift-axis x/y/z
--yaw-sign -1/1
--reach-sign -1/1
--lift-sign -1/1
--wrist-x-sign -1/1
--wrist-y-sign -1/1
--thumbstick-x-axis 2
--thumbstick-y-axis 3
```

## RViz

If RViz launched by OpenArm crashes because of Qt/CoppeliaSim library conflicts,
start RViz in a clean terminal:

```bash
conda deactivate

unset LD_LIBRARY_PATH
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH

source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

rviz2 -d ~/OPENARM/ros2_ws/install/openarm_description/share/openarm_description/rviz/bimanual.rviz
```

## Important Local Patches Made

### teleop_xr video request guard

File:

```text
~/OPENARM/teleop_xr/teleop_xr/__init__.py
```

When no video source is configured, ignore `video_request` instead of creating
an empty WebRTC offer. This avoids Pico-side errors like:

```text
BUNDLE group contains a MID='' matching no m= section
```

Expected log after the patch:

```text
No video sources configured; ignoring video request
```

### teleop_xr controller pose fallback

File:

```text
~/OPENARM/teleop_xr/teleop_xr/ros2/__main__.py
```

Controller pose publishing now uses:

```python
device.get("gripPose") or device.get("pose")
```

This was needed because Pico 4 provided `pose` while `gripPose` was missing.

### teleop_xr IK pose fallback and no-deadman option

Files:

```text
~/OPENARM/teleop_xr/teleop_xr/ik/controller.py
~/OPENARM/teleop_xr/teleop_xr/ros2/cli.py
~/OPENARM/teleop_xr/teleop_xr/ros2/__main__.py
```

Changes:

- IK controller also accepts `pose` when `gripPose` is missing.
- Added `--require-deadman / --no-require-deadman`.

## Troubleshooting

### Pico page opens but OpenArm does not move

Check XR pose:

```bash
ros2 topic echo /xr/controller_right/pose --once
```

Check IK trajectory:

```bash
ros2 topic echo /joint_trajectory --once
```

Check controller-specific trajectory:

```bash
ros2 topic echo /right_joint_trajectory_controller/joint_trajectory --once
```

### Controllers are not active

```bash
ros2 control list_controllers
```

If using official IK route, use:

```bash
robot_controller:=joint_trajectory_controller
```

If using raw mapper route, use:

```bash
robot_controller:=forward_position_controller
```

### WebRTC video errors

If no camera stream is configured, errors from `VideoClient` can be ignored after
the backend guard patch. The teleoperation path uses `/xr/controller_*` topics.

### Pico buttons are all zero

During bring-up, use:

```bash
--no-require-deadman
```

For hardware, do not use this. Find the real button index:

```bash
ros2 topic echo /xr/controller_right/joy
```

Then re-enable deadman with the correct index in the relevant bridge/mapper.

## Safety Before Real Hardware

- Remove `--no-require-deadman`.
- Reduce gains / speed.
- Confirm joint limits.
- Confirm trajectory controllers receive expected joint names only.
- Keep emergency stop ready.
- Test one arm at a time if possible.
