# Pico 4 To LeRobot VLA To Simulation Rollout

> 双臂实机丝滑摇操完整指南：
> [DUAL_ARM_SMOOTH_TELEOP_GUIDE.md](./DUAL_ARM_SMOOTH_TELEOP_GUIDE.md)

This is the recommended end-to-end path for the current project:

```text
Pico 4 teleoperation
  -> OpenArm ROS2 fake hardware
  -> LeRobot dataset
  -> VLA training
  -> simulation rollout
```

Use simulation until the dataset, model, and rollout loop are stable. Real
hardware should come only after official CAN validation and safety limits.

## Stage 0: Principle

For VLA training, do not collect proprioception-only data as the final dataset.
VLA policies need images plus language task text plus robot state/action.

Use proprioception-only recording only as a quick transport test.

Minimum useful dataset:

```text
observation.state
observation.images.front
action
task
```

Better dataset:

```text
observation.state
observation.images.left_wrist
observation.images.right_wrist
observation.images.front
action
task
```

Start with low camera load:

```text
424x240 @ 30fps
```

Then raise resolution only after the whole loop is stable.

### Camera Choice For Simulation

If the OpenArm is simulated, the training camera should normally be simulated
too. RealSense or USB cameras are useful for real-world collection, but they do
not see the simulated robot/task in RViz.

The current local OpenArm launch is `ros2_control` fake hardware plus RViz. It
publishes joint states and accepts position commands, but it is not a rendering
simulator and does not provide virtual RGB camera images by itself.

Use these camera levels:

| Level | Camera source | Purpose |
| --- | --- | --- |
| Smoke test | Synthetic video file | Check dataset, training, policy loading, and rollout plumbing |
| Rendered sim smoke | `openarm_sim` custom renderer | Collect simple grasp-task images from fake hardware joint states |
| Official MuJoCo sim | `enactic/openarm_mujoco` + `openarm_mujoco_follower` | Train/evaluate policies in a rendered OpenArm task scene |
| Real simulation data | Virtual camera from Gazebo/Ignition, Isaac Sim, MuJoCo, PyBullet, or a custom renderer | Train/evaluate policies on simulated scenes |
| Real robot data | RealSense or USB camera | Train/evaluate on physical hardware |

For the current project, do not use RealSense as the default for pure
simulation. First run the synthetic camera smoke test below, then add a virtual
sim camera before collecting meaningful VLA data.

The local tree includes a lightweight custom renderer camera:

```text
openarm_sim
```

It renders a simple tabletop grasp scene from OpenArm joint states. This is not
full physics, but it is enough to validate rendered image observations,
training, and policy rollout before moving to Gazebo/Isaac/MuJoCo.

For the preferred MuJoCo route, use the official assets:

```text
~/OPENARM/openarm_mujoco
```

The local LeRobot tree adds:

```text
openarm_mujoco_follower
```

This loads `openarm_mujoco/v2/openarm_mujoco_v2/demo.xml`, renders MuJoCo
cameras such as `camera_head`, and accepts the same 16D OpenArm action format
used by the ROS2 bridge.

## Stage 1: Collect Pico 4 Data In Simulation

### Terminal 1: OpenArm fake hardware

Use forward position controllers because LeRobot should be the process that
sends actions during recording.

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

### Terminal 2: Start Pico 4 / teleop_xr IK

```bash
conda activate oplerobot310
cd ~/OPENARM/teleop_xr

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export OPENARM_DESCRIPTION_PATH=~/OPENARM/ros2_ws/src/openarm_description
export RAM_CACHE_DIR=/tmp/ram_cache

python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode ik \
  --robot-class openarm \
  --output-topic /joint_trajectory \
  --no-urdf-topic \
  --no-require-deadman
```

Open Pico 4 browser:

```text
https://<PC_IP>:4443
```

### Terminal 3: Configure cameras

For pure simulation, use a virtual camera source. The current fake-hardware
OpenArm/RViz setup does not create one automatically, so there are two paths:

1. For a quick software check, use the synthetic camera video generated in
   `Smoke Test Without Pico Or Cameras`.
2. For meaningful VLA data, run a rendering simulator and expose its RGB camera
   as the LeRobot `front` camera.

Keep camera names stable across recording and rollout. If training uses
`observation.images.front`, rollout must also provide `front`.

RealSense and USB cameras are for real-world data collection or a temporary
debug view. They are not the right default for pure simulation.

RealSense detection, for later real-world collection:

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
python -m lerobot.scripts.lerobot_find_cameras realsense
```

If using generic USB cameras:

```bash
python -m lerobot.scripts.lerobot_find_cameras opencv
```

### Terminal 4: Record dataset

Start with 10 to 20 short episodes, then inspect before collecting more.

The command below shows a real RealSense camera because LeRobot needs an image
source. For pure simulation, replace the `front` camera config with your virtual
camera source before treating the dataset as useful training data.

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
export PATH=/home/w/anaconda3/envs/oplerobot310/bin:$PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_record \
  --robot.type=bi_openarm_ros2_follower \
  --robot.id=openarm_pico4_sim \
  --robot.cameras='{
    front: {type: intelrealsense, serial_number_or_name: "HEAD_SERIAL", width: 424, height: 240, fps: 30}
  }' \
  --teleop.type=pico_openarm_ik \
  --teleop.trajectory_topic=/joint_trajectory \
  --dataset.repo_id=local/openarm_pico4_vla \
  --dataset.single_task="Move the object to the target with the OpenArm robot" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=5 \
  --dataset.fps=30 \
  --dataset.video=true \
  --dataset.push_to_hub=false \
  --display_data=true
```

For three cameras:

```bash
--robot.cameras='{
  left_wrist: {type: intelrealsense, serial_number_or_name: "LEFT_SERIAL", width: 424, height: 240, fps: 30},
  right_wrist: {type: intelrealsense, serial_number_or_name: "RIGHT_SERIAL", width: 424, height: 240, fps: 30},
  front: {type: intelrealsense, serial_number_or_name: "HEAD_SERIAL", width: 424, height: 240, fps: 30}
}'
```

Temporary synthetic-video camera for a pipeline-only check:

```bash
--robot.cameras='{front: {type: opencv, index_or_path: "/tmp/openarm_front_smoke.mp4", width: 64, height: 64, fps: 5, warmup_s: 0}}'
```

This checks plumbing only. It does not produce a useful policy because the image
does not reflect the actual simulated OpenArm scene.

Rendered OpenArm grasp camera for the current fake-hardware sim:

```bash
--robot.cameras='{front: {type: openarm_sim, width: 128, height: 96, fps: 6, warmup_s: 0}}'
```

This renders the virtual scene from `/joint_states`, so rollout images change
with the simulated OpenArm state.

Useful recording keys:

```text
Right arrow: save current episode and enter reset
Left arrow: discard current episode and re-record
Esc: stop recording
```

Note: `lerobot_record` stamps the dataset repo id with a timestamp. A command
using `local/openarm_pico4_vla` will save as something like:

```text
local/openarm_pico4_vla_20260517_153000
```

Use the stamped name in training.

## Stage 2: Inspect Dataset

Find the actual dataset name:

```bash
ls ~/.cache/huggingface/lerobot/local
```

Print dataset info:

```bash
cd ~/OPENARM/lerobot
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

python - <<'PY'
from lerobot.datasets import LeRobotDataset

repo_id = "local/openarm_pico4_vla_YYYYMMDD_HHMMSS"
dataset = LeRobotDataset(repo_id)
print(dataset)
print("features:", dataset.features)
print("num_frames:", dataset.num_frames)
print("num_episodes:", dataset.num_episodes)
print("first action shape:", dataset[0]["action"].shape)
PY
```

Visualize an episode:

```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/openarm_pico4_vla_YYYYMMDD_HHMMSS \
  --episode-index 0
```

Pass criteria:

- Images are not black, frozen, or wrong camera.
- `observation.state` changes when OpenArm moves.
- `action` has 16 dimensions for bimanual OpenArm: 14 arm joints plus 2 grippers.
- The action and state roughly track each other with reasonable delay.
- Task text is correct and consistent.

If this fails, do not train yet. Fix recording first.

## Stage 3: Train A VLA Policy

For this project, start with `smolvla`. It is the most practical VLA starting
point in this local tree. `pi0`, `pi05`, and `xvla` are heavier and should come
after the data/rollout loop is proven.

Install training dependencies if needed:

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310
python -m pip install -e ".[training]"
```

Train a short smoke model first:

```bash
cd ~/OPENARM/lerobot
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_train \
  --policy.type=smolvla \
  --dataset.repo_id=local/openarm_pico4_vla_YYYYMMDD_HHMMSS \
  --output_dir=outputs/train/openarm_pico4_smolvla_smoke \
  --batch_size=2 \
  --steps=1000 \
  --save_freq=500 \
  --log_freq=20 \
  --policy.n_action_steps=10 \
  --policy.chunk_size=10 \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --policy.train_state_proj=true \
  --policy.load_vlm_weights=false \
  --wandb.enable=false
```

If the smoke run works, train a real first model:

```bash
HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_train \
  --policy.type=smolvla \
  --dataset.repo_id=local/openarm_pico4_vla_YYYYMMDD_HHMMSS \
  --output_dir=outputs/train/openarm_pico4_smolvla \
  --batch_size=4 \
  --steps=30000 \
  --save_freq=5000 \
  --log_freq=100 \
  --policy.n_action_steps=20 \
  --policy.chunk_size=20 \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --policy.train_state_proj=true \
  --policy.load_vlm_weights=false \
  --wandb.enable=false
```

Expected output checkpoint:

```text
outputs/train/openarm_pico4_smolvla/checkpoints/last/pretrained_model
```

## Stage 4: Deploy The Policy In Simulation

Start OpenArm fake hardware again with forward position controllers:

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

Run policy rollout:

```bash
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

python -m lerobot.scripts.lerobot_rollout \
  --strategy.type=base \
  --policy.path=outputs/train/openarm_pico4_smolvla/checkpoints/last/pretrained_model \
  --inference.type=rtc \
  --inference.rtc.execution_horizon=10 \
  --robot.type=bi_openarm_ros2_follower \
  --robot.id=openarm_pico4_sim_rollout \
  --robot.cameras='{
    front: {type: intelrealsense, serial_number_or_name: "HEAD_SERIAL", width: 424, height: 240, fps: 30}
  }' \
  --task="Move the object to the target with the OpenArm robot" \
  --duration=60 \
  --fps=30 \
  --display_data=true
```

Use the same camera names and task text used during recording. If the policy was
trained with a virtual `front` camera, rollout must provide the same virtual
`front` camera. If the policy was trained with the synthetic smoke video, use
that same video only for a plumbing check.

Pass criteria:

- Policy loads without feature mismatch errors.
- Rerun display shows the expected camera and state.
- OpenArm moves in RViz/fake hardware.
- No large jumps at episode start.

## Recommended Iteration Loop

1. Record 10 to 20 episodes.
2. Inspect data.
3. Train 1,000-step smoke model.
4. Roll out in simulation for 30 to 60 seconds.
5. If rollout starts and moves, collect 100+ cleaner episodes.
6. Train longer.
7. Only then consider real hardware.

## Smoke Test Without Pico Or Cameras

Before using the real Pico 4 and cameras, a fully synthetic smoke test can
validate the software chain:

```text
synthetic Pico-like actions + synthetic front image
  -> tiny LeRobot dataset
  -> tiny ACT training
  -> rollout into ROS2 OpenArm mock
```

This is not a quality test. It only checks that the data format, training,
policy loading, camera feature matching, ROS2 bridge, and rollout loop all work.

### 1. Generate a tiny dataset

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache

python examples/openarm_xr/smoke_synthetic_pico_dataset.py \
  --repo-id local/openarm_pico4_smoke \
  --root /tmp/lerobot_openarm_pico4_smoke \
  --episodes 2 \
  --frames 16 \
  --fps 5
```

Expected:

```text
repo_id=local/openarm_pico4_smoke
root=/tmp/lerobot_openarm_pico4_smoke
front_video=/tmp/openarm_front_smoke.mp4
frames=32
```

### 2. Check the dataset

```bash
python - <<'PY'
from lerobot.datasets import LeRobotDataset

ds = LeRobotDataset("local/openarm_pico4_smoke", root="/tmp/lerobot_openarm_pico4_smoke")
item = ds[0]
print(ds.num_frames, ds.num_episodes)
print(item["observation.state"].shape)
print(item["observation.images.front"].shape)
print(item["action"].shape)
print(item["task"])
PY
```

Expected shapes:

```text
torch.Size([16])
torch.Size([3, 64, 64])
torch.Size([16])
```

### 3. Train a two-step tiny model

```bash
HF_DATASETS_CACHE=/tmp/hf_datasets_cache HF_HUB_OFFLINE=1 \
python -m lerobot.scripts.lerobot_train \
  --policy.type=act \
  --dataset.repo_id=local/openarm_pico4_smoke \
  --dataset.root=/tmp/lerobot_openarm_pico4_smoke \
  --output_dir=/tmp/openarm_pico4_act_smoke \
  --batch_size=2 \
  --steps=2 \
  --save_freq=2 \
  --log_freq=1 \
  --num_workers=0 \
  --persistent_workers=false \
  --policy.device=cpu \
  --policy.push_to_hub=false \
  --policy.chunk_size=4 \
  --policy.n_action_steps=4 \
  --policy.dim_model=64 \
  --policy.n_heads=2 \
  --policy.dim_feedforward=128 \
  --policy.n_encoder_layers=1 \
  --policy.n_vae_encoder_layers=1 \
  --policy.use_vae=false \
  --policy.pretrained_backbone_weights=null \
  --wandb.enable=false
```

Expected checkpoint:

```text
/tmp/openarm_pico4_act_smoke/checkpoints/last/pretrained_model
```

### 4. Start ROS2 OpenArm mock

```bash
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export ROS_LOG_DIR=/tmp/ros_logs

python examples/openarm_ros2/mock_ros2_openarm.py --bimanual --fps 50
```

### 5. Roll out into the mock

In another terminal:

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export ROS_LOG_DIR=/tmp/ros_logs
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache

python -m lerobot.scripts.lerobot_rollout \
  --strategy.type=base \
  --policy.path=/tmp/openarm_pico4_act_smoke/checkpoints/last/pretrained_model \
  --inference.type=sync \
  --robot.type=bi_openarm_ros2_follower \
  --robot.id=openarm_pico4_smoke_rollout \
  --robot.cameras='{front: {type: opencv, index_or_path: "/tmp/openarm_front_smoke.mp4", width: 64, height: 64, fps: 5, warmup_s: 0}}' \
  --task="Smoke test synthetic Pico control for OpenArm" \
  --duration=2 \
  --fps=5 \
  --device=cpu
```

Successful rollout logs include:

```text
Policy loaded: type=act, device=cpu
Robot connected: bi_openarm_ros2_follower
SyncInferenceEngine initialized
Base strategy control loop started
Duration limit reached
Rollout finished
```

## Rendered Grasp Simulation Smoke Test

Use this after the pure synthetic smoke test. It creates a small virtual grasp
task with a front camera, trains a tiny ACT policy for a few steps, and deploys
it into the ROS2 OpenArm mock with the `openarm_sim` virtual camera.

### 1. Generate rendered grasp data

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache

python examples/openarm_xr/rendered_openarm_grasp_dataset.py \
  --repo-id local/openarm_rendered_grasp_smoke \
  --root /tmp/lerobot_openarm_rendered_grasp \
  --episodes 2 \
  --frames 24 \
  --fps 6 \
  --height 96 \
  --width 128 \
  --preview-video /tmp/openarm_rendered_grasp_preview.mp4
```

Expected:

```text
repo_id=local/openarm_rendered_grasp_smoke
root=/tmp/lerobot_openarm_rendered_grasp
preview_video=/tmp/openarm_rendered_grasp_preview.mp4
frames=48
task=Grasp the orange cube and move it to the green target with OpenArm
```

### 2. Train a tiny policy

```bash
HF_DATASETS_CACHE=/tmp/hf_datasets_cache HF_HUB_OFFLINE=1 \
python -m lerobot.scripts.lerobot_train \
  --policy.type=act \
  --dataset.repo_id=local/openarm_rendered_grasp_smoke \
  --dataset.root=/tmp/lerobot_openarm_rendered_grasp \
  --output_dir=/tmp/openarm_rendered_grasp_act_smoke \
  --batch_size=2 \
  --steps=3 \
  --save_freq=3 \
  --log_freq=1 \
  --num_workers=0 \
  --persistent_workers=false \
  --policy.device=cpu \
  --policy.push_to_hub=false \
  --policy.chunk_size=4 \
  --policy.n_action_steps=4 \
  --policy.dim_model=64 \
  --policy.n_heads=2 \
  --policy.dim_feedforward=128 \
  --policy.n_encoder_layers=1 \
  --policy.n_vae_encoder_layers=1 \
  --policy.use_vae=false \
  --policy.pretrained_backbone_weights=null \
  --wandb.enable=false
```

Expected checkpoint:

```text
/tmp/openarm_rendered_grasp_act_smoke/checkpoints/last/pretrained_model
```

### 3. Start the ROS2 OpenArm mock

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export ROS_LOG_DIR=/tmp/ros_logs

python examples/openarm_ros2/mock_ros2_openarm.py --bimanual --fps 50
```

### 4. Roll out with the rendered virtual camera

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export ROS_LOG_DIR=/tmp/ros_logs
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache

python -m lerobot.scripts.lerobot_rollout \
  --strategy.type=base \
  --policy.path=/tmp/openarm_rendered_grasp_act_smoke/checkpoints/last/pretrained_model \
  --inference.type=sync \
  --robot.type=bi_openarm_ros2_follower \
  --robot.id=openarm_rendered_grasp_rollout \
  --robot.cameras='{front: {type: openarm_sim, width: 128, height: 96, fps: 6, warmup_s: 0}}' \
  --task="Grasp the orange cube and move it to the green target with OpenArm" \
  --duration=2 \
  --fps=6 \
  --device=cpu
```

Successful rollout logs include:

```text
Policy loaded: type=act, device=cpu
Robot connected: bi_openarm_ros2_follower
SyncInferenceEngine initialized
Base strategy control loop started
Duration limit reached
Rollout finished
```

## Official MuJoCo OpenArm Grasp Smoke Test

This is the preferred rendered simulation path. It uses the official
`enactic/openarm_mujoco` MJCF model instead of the lightweight custom renderer.

### 0. Install and fetch assets

```bash
cd ~/OPENARM
git clone https://github.com/enactic/openarm_mujoco.git

conda activate oplerobot310
python -m pip install mujoco
```

Headless rendering uses EGL:

```bash
export MUJOCO_GL=egl
```

Useful official scene:

```text
~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml
```

The scene contains:

```text
orange_cube
black_frame
camera_ceiling
camera_head
camera_wrist_left
camera_wrist_right
```

### 1. Generate a tiny MuJoCo rendered dataset

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache
export MUJOCO_GL=egl

python examples/openarm_xr/mujoco_openarm_grasp_dataset.py \
  --repo-id local/openarm_mujoco_grasp_smoke \
  --root /tmp/lerobot_openarm_mujoco_grasp \
  --episodes 2 \
  --frames 20 \
  --fps 6 \
  --height 96 \
  --width 128 \
  --preview-video /tmp/openarm_mujoco_grasp_preview.mp4 \
  --cameras front:camera_head
```

Expected:

```text
repo_id=local/openarm_mujoco_grasp_smoke
root=/tmp/lerobot_openarm_mujoco_grasp
xml_path=~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml
preview_video=/tmp/openarm_mujoco_grasp_preview.mp4
frames=40
task=Grasp the orange cube and move it to the black target frame with OpenArm
```

### 2. Train a tiny policy

```bash
HF_DATASETS_CACHE=/tmp/hf_datasets_cache HF_HUB_OFFLINE=1 \
python -m lerobot.scripts.lerobot_train \
  --policy.type=act \
  --dataset.repo_id=local/openarm_mujoco_grasp_smoke \
  --dataset.root=/tmp/lerobot_openarm_mujoco_grasp \
  --output_dir=/tmp/openarm_mujoco_grasp_act_smoke \
  --batch_size=2 \
  --steps=3 \
  --save_freq=3 \
  --log_freq=1 \
  --num_workers=0 \
  --persistent_workers=false \
  --policy.device=cpu \
  --policy.push_to_hub=false \
  --policy.chunk_size=4 \
  --policy.n_action_steps=4 \
  --policy.dim_model=64 \
  --policy.n_heads=2 \
  --policy.dim_feedforward=128 \
  --policy.n_encoder_layers=1 \
  --policy.n_vae_encoder_layers=1 \
  --policy.use_vae=false \
  --policy.pretrained_backbone_weights=null \
  --wandb.enable=false
```

Expected checkpoint:

```text
/tmp/openarm_mujoco_grasp_act_smoke/checkpoints/last/pretrained_model
```

### 3. Roll out directly in MuJoCo

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache
export MUJOCO_GL=egl

python -m lerobot.scripts.lerobot_rollout \
  --strategy.type=base \
  --policy.path=/tmp/openarm_mujoco_grasp_act_smoke/checkpoints/last/pretrained_model \
  --inference.type=sync \
  --robot.type=openarm_mujoco_follower \
  --robot.id=openarm_mujoco_rollout \
  --robot.calibration_dir=/tmp/lerobot_openarm_mujoco_calibration \
  --robot.xml_path=~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml \
  --robot.width=128 \
  --robot.height=96 \
  --robot.fps=6 \
  --robot.camera_views='{front: camera_head}' \
  --robot.n_substeps=8 \
  --task="Grasp the orange cube and move it to the black target frame with OpenArm" \
  --duration=2 \
  --fps=6 \
  --device=cpu
```

Successful rollout logs include:

```text
Policy loaded: type=act, device=cpu
Robot connected: openarm_mujoco_follower
SyncInferenceEngine initialized
Base strategy control loop started
Duration limit reached
Rollout finished
```

### 4. Pico 4 data collection into MuJoCo

For Pico collection, MuJoCo replaces the ROS2 fake-hardware robot. Keep
`teleop_xr` only as the XR IK source, and let LeRobot send actions into MuJoCo.

Terminal 1:

```bash
conda activate oplerobot310
cd ~/OPENARM/teleop_xr

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export OPENARM_DESCRIPTION_PATH=~/OPENARM/ros2_ws/src/openarm_description
export RAM_CACHE_DIR=/tmp/ram_cache

python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode ik \
  --robot-class openarm \
  --output-topic /joint_trajectory \
  --no-urdf-topic \
  --no-require-deadman
```

Terminal 2:

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
export PATH=/home/w/anaconda3/envs/oplerobot310/bin:$PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache
export MUJOCO_GL=egl

HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_record \
  --robot.type=openarm_mujoco_follower \
  --robot.id=openarm_mujoco_pico4 \
  --robot.calibration_dir=/tmp/lerobot_openarm_mujoco_calibration \
  --robot.xml_path=~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml \
  --robot.width=128 \
  --robot.height=96 \
  --robot.fps=30 \
  --robot.camera_views='{front: camera_head}' \
  --robot.n_substeps=2 \
  --teleop.type=pico_openarm_ik \
  --teleop.trajectory_topic=/joint_trajectory \
  --teleop.require_joint_states=false \
  --dataset.repo_id=local/openarm_mujoco_pico4 \
  --dataset.single_task="Grasp the orange cube and move it to the black target frame with OpenArm" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=5 \
  --dataset.fps=30 \
  --dataset.video=true \
  --dataset.push_to_hub=false \
  --display_data=true \
  --display_compressed_images=true
```

Because `openarm_mujoco_follower` is not a ROS2 robot, it does not publish
`/joint_states`. Use:

```bash
--teleop.require_joint_states=false
```

so `pico_openarm_ik` waits for `/joint_trajectory` from `teleop_xr` instead of
waiting for ROS2 fake-hardware state.

Recording controls:

```text
Right arrow: end and save the current episode early
Left arrow: end and discard the current episode, then re-record it
Esc: stop recording
```

With `openarm_mujoco_follower`, ending an episode triggers an automatic MuJoCo
reset before the next episode starts. The OpenArm state returns to the
configured `home` keyframe, and the `orange_cube` returns to its initial pose.
The reset interval does not replay Pico teleop commands, so the neutral scene is
not immediately overwritten by the current controller pose.

Pico trigger grippers are enabled by default in `pico_openarm_ik`:

```text
Right trigger -> right_gripper.pos
Left trigger -> left_gripper.pos
released -> open
pressed -> closed
```

The default trigger source is WebXR button index `0`. `teleop_xr` publishes the
analog button values at the end of `/xr/controller_{left|right}/joy.axes`, so
the gripper can close gradually on headsets that report analog trigger values.
If a headset reports only digital buttons, the same mapping still works as
open/closed.

For consistent demonstrations, start each recording/reset with a stable neutral
hand pose:

```text
controllers in front of the chest
wrists relaxed and roughly level
arms not fully extended
triggers released
Pico tracking stable for 1-2 seconds
```

`teleop_xr` uses the controller pose when the XR session starts or when the IK
pose is reset as the practical reference for the motion range. The absolute
room position does not need to be exact, but the relative pose should be
repeatable. If the robot starts from an awkward offset, end the XR session,
return your hands to the neutral pose, and enter XR again.

### Pico Neutral Pose Calibration

Do this before collecting serious demonstrations. The goal is to make the human
neutral pose, teleop_xr IK neutral pose, and MuJoCo OpenArm initial pose agree.

1. Put the headset on and stand or sit in the posture you will use for the
   whole dataset.
2. Hold both controllers in front of the chest, about shoulder width apart.
3. Keep elbows slightly bent, wrists relaxed, and triggers released.
4. Start the XR session from that pose.
5. Start LeRobot teleoperation with display enabled for a short check:

```bash
python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=openarm_mujoco_follower \
  --robot.id=openarm_mujoco_pico4_calib \
  --robot.calibration_dir=/tmp/lerobot_openarm_mujoco_calibration \
  --robot.xml_path=~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml \
  --robot.width=128 \
  --robot.height=96 \
  --robot.fps=30 \
  --robot.camera_views='{front: camera_head}' \
  --robot.n_substeps=2 \
  --teleop.type=pico_openarm_ik \
  --teleop.trajectory_topic=/joint_trajectory \
  --teleop.require_joint_states=false \
  --fps=30 \
  --display_data=true \
  --teleop_time_s=20
```

Pass criteria:

```text
OpenArm starts in a comfortable bent-elbow pose
both arms are reachable without fully extending your own arms
left and right sides feel symmetric
triggers open and close the corresponding grippers
small hand motions produce small robot motions
```

If the start pose is awkward, stop both terminals, return your hands to the
neutral pose, and start the XR session again. Do not collect data until this
neutral pose feels repeatable.

The local `pico_openarm_ik` default action is matched to teleop_xr's OpenArm IK
default pose:

```text
left arm:  [0.0, -45.84, 0.0, 68.75, 0.0, 0.0, 0.0]
right arm: [0.0,  45.84, 0.0, 68.75, 0.0, 0.0, 0.0]
grippers: open
```

For fine tuning, override the default arm pose from the LeRobot command. For
example, to make the elbows slightly more bent:

```bash
--teleop.default_left_arm_degrees='[0.0, -50.0, 0.0, 75.0, 0.0, 0.0, 0.0]' \
--teleop.default_right_arm_degrees='[0.0, 50.0, 0.0, 75.0, 0.0, 0.0, 0.0]'
```

Before wearing the Pico 4, the same interface can be smoke-tested with a fake
trajectory publisher.

Terminal A:

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

python examples/openarm_xr/publish_fake_openarm_trajectory.py \
  --topic /joint_trajectory \
  --fps 30
```

Terminal B:

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
export PATH=/home/w/anaconda3/envs/oplerobot310/bin:$PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache
export MUJOCO_GL=egl

python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=openarm_mujoco_follower \
  --robot.id=openarm_mujoco_pico_smoke \
  --robot.calibration_dir=/tmp/lerobot_openarm_mujoco_calibration \
  --robot.xml_path=~/OPENARM/openarm_mujoco/v2/openarm_mujoco_v2/demo.xml \
  --robot.width=128 \
  --robot.height=96 \
  --robot.fps=30 \
  --robot.camera_views='{front: camera_head}' \
  --robot.n_substeps=2 \
  --teleop.type=pico_openarm_ik \
  --teleop.trajectory_topic=/joint_trajectory \
  --teleop.require_joint_states=false \
  --fps=30 \
  --teleop_time_s=2
```

Successful smoke logs include:

```text
PicoOpenArmIKTeleop connected to /joint_trajectory
OpenArmMujocoFollower connected to MuJoCo model
Teleop loop time: ... (30 Hz)
OpenArmMujocoFollower disconnected
```

## Common Failure Points

### Policy feature mismatch

Cause:

- Training used different camera names than rollout.
- Training had `front`, rollout uses `head`, or vice versa.

Fix:

- Keep camera keys identical in record and rollout.

### VLA has no useful visual signal

Cause:

- Dataset recorded with `--dataset.video=false`.
- Camera was missing, black, or pointed away from the task.
- Pure fake hardware/RViz was used without a virtual RGB camera.
- Synthetic smoke video was used as if it were meaningful training data.

Fix:

- Record with `--dataset.video=true`.
- Use `--display_data=true` and inspect before training.
- For simulation, add a real virtual camera from a rendering simulator.

### RealSense cameras are not detected

If this command:

```bash
python -m lerobot.scripts.lerobot_find_cameras realsense
```

prints an error like:

```text
'NoneType' object has no attribute 'context'
```

install the Python RealSense binding in the active environment:

```bash
conda activate oplerobot310
python -m pip install pyrealsense2
```

Then check system-level detection:

```bash
rs-enumerate-devices
lsusb | grep -Ei '8086|realsense|intel'
```

If `rs-enumerate-devices` says:

```text
No device detected. Is it plugged in?
```

the issue is below LeRobot: camera not plugged in, bad cable, non-USB3 port,
missing udev permission, or librealsense device setup. Until this is fixed,
use an OpenCV USB camera for the software smoke loop:

```bash
python -m lerobot.scripts.lerobot_find_cameras opencv
```

and record with:

```bash
--robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}'
```

### Robot does not move during recording

Check:

```bash
ros2 topic echo /joint_trajectory --once
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

Expected controllers:

```text
left_forward_position_controller
right_forward_position_controller
joint_state_broadcaster
```

### Rollout moves too abruptly

Fix:

- Reduce rollout FPS.
- Train on smoother demonstrations.
- Add conservative robot-side `max_relative_target` before real hardware.

## When To Move To Real Hardware

Only after all are true:

- Dataset collection works in simulation with cameras.
- VLA trains without feature mismatch.
- Simulation rollout runs without large jumps.
- Official OpenArm CAN setup passes.
- Deadman button is mapped and required.
- Emergency stop is ready.

Then replace:

```text
bi_openarm_ros2_follower
```

with:

```text
bi_openarm_follower
```

and start with one arm, low speed, and strict safety limits.
