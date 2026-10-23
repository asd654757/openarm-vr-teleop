# OpenArm Pico 4 / WebXR Teleop

真实双臂 LeRobot 采集、重播、重力补偿和 Rerun 排查流程见：

[`OPENARMX_LEROBOT_TROUBLESHOOTING_AND_WORKFLOW.md`](OPENARMX_LEROBOT_TROUBLESHOOTING_AND_WORKFLOW.md)

项目阶段总结、参数理由、从开机到录制/重播、D435i 视觉采集以及
VLA/仿真路线见：

[`OPENARMX_PROJECT_TECHNICAL_SUMMARY.md`](OPENARMX_PROJECT_TECHNICAL_SUMMARY.md)

从开机、IP/CAN 检查、50% 重力补偿分级设置到 D435i 带视觉摇操采集的可执行
SOP 见：

[`OPENARMX_VISUAL_TELEOP_RECORDING_SOP.md`](OPENARMX_VISUAL_TELEOP_RECORDING_SOP.md)

D435i 全局 RGB 相机和 VLA 视觉采集命令见：

[`OPENARMX_BIMANUAL_100HZ_RECORD_REPLAY.md`](OPENARMX_BIMANUAL_100HZ_RECORD_REPLAY.md)

This example wires `teleop_xr` raw ROS2 topics into the OpenArm bimanual ROS2
controllers. It is meant for fake hardware / simulation first, then the same
topics can be pointed at real OpenArm controllers after safety checks.

## 1. Start OpenArm fake hardware

In terminal 1:

```bash
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/w/OPENARM/ros2_ws/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

If the forward position controllers are not active, load them once:

```bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active left_forward_position_controller
ros2 control load_controller --set-state active right_forward_position_controller
```

Check:

```bash
ros2 control list_controllers
```

## 2. Start teleop_xr ROS2 bridge

In terminal 2:

```bash
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/w/OPENARM/ros2_ws/install/setup.bash

/home/w/anaconda3/envs/oplerobot310/bin/python -m teleop_xr.ros2 \
  --host 0.0.0.0 \
  --port 4443 \
  --input-mode CONTROLLER \
  --mode teleop
```

Open Pico 4 Browser to:

```text
https://<PC_IP>:4443
```

Accept the local self-signed certificate if prompted, enter XR mode, and keep the
Pico 4 and PC on the same LAN.

Useful topic checks:

```bash
ros2 topic echo /xr/controller_right/pose --once
ros2 topic echo /xr/controller_right/joy --once
```

## 3. Start the OpenArm mapper

In terminal 3:

```bash
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/w/OPENARM/ros2_ws/install/setup.bash
export PYTHONPATH=/home/w/OPENARM/lerobot/src:$PYTHONPATH

/home/w/anaconda3/envs/oplerobot310/bin/python \
  /home/w/OPENARM/lerobot/examples/openarm_xr/openarm_xr_mapper.py
```

Default control mapping:

- Hold button `0` on each controller as the deadman switch.
- Move controller left/right (`y` in ROS FLU) to change `joint_1`.
- Move controller forward/back (`x` in ROS FLU) to change `joint_2`.
- Move controller up/down (`z` in ROS FLU) to change `joint_4`.
- Use thumbstick axes for `joint_5` and `joint_6`.
- Button `1` closes the gripper, button `2` opens the gripper.

If an axis feels swapped, remap it without editing code:

```bash
/home/w/anaconda3/envs/oplerobot310/bin/python \
  /home/w/OPENARM/lerobot/examples/openarm_xr/openarm_xr_mapper.py \
  --yaw-axis y \
  --reach-axis x \
  --lift-axis z
```

If a direction is inverted, flip only that sign, for example:

```bash
--lift-sign -1
```

Tune gains if the fake arm moves too much:

```bash
/home/w/anaconda3/envs/oplerobot310/bin/python \
  /home/w/OPENARM/lerobot/examples/openarm_xr/openarm_xr_mapper.py \
  --yaw-gain-deg-per-m 45 \
  --reach-gain-deg-per-m 40 \
  --lift-gain-deg-per-m 45 \
  --wrist-gain-deg 10
```

For first smoke tests without Pico button data, you can temporarily disable the
deadman gate:

```bash
/home/w/anaconda3/envs/oplerobot310/bin/python \
  /home/w/OPENARM/lerobot/examples/openarm_xr/openarm_xr_mapper.py \
  --no-require-deadman
```

Use very small gains if you do this.

## Notes before real hardware

- Keep `--require-deadman` enabled.
- Start with smaller gains than simulation.
- Confirm `/joint_states` names match the OpenArm bimanual names.
- Keep the controller manager and emergency stop path visible before enabling
  hardware command topics.
