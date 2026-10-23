# OpenarmXR VR Teleoperation Reproduction Plan

This is a separate experimental path from the current Pico 4 + `teleop_xr` +
LeRobot path.

Reference repository:

```text
https://github.com/j0minjae/OpenarmXR
```

## Goal

Use the OpenarmXR stack to reproduce Pico/Quest-style VR teleoperation for a
standard OpenArm robot, while keeping the existing LeRobot pipeline intact.

Reference dataflow:

```text
Pico 4 APK
  -> UDP packets on host port 5100
  -> openarmx_teleop_bridge_vr
  -> VR controller ROS 2 topics
  -> OpenArm-compatible IK teleop node
  -> forward_position_controller command topics
  -> openarm_bringup
  -> fake hardware or CAN
  -> OpenArm
```

This path should be validated in fake hardware first. Do not connect real CAN
until the command topics and RViz motion are correct.

## Local Install Status

Installed locally:

```text
~/OPENARM/OpenarmXR
```

Build status:

```text
9 packages built successfully
```

Installed Python dependency for the system ROS Python:

```text
openarmx-arm-driver
pin==2.7.0
numpy==1.26.4
```

`pin==2.7.0` is used because the latest `pin` package currently pulls
`numpy>=2`, while OpenarmXR expects `numpy<2`.

ROS 2 can find the key executables after sourcing the OpenarmXR workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 pkg executables openarmx_teleop_vr
ros2 pkg executables openarmx_teleop_bridge_vr
```

Expected:

```text
openarmx_teleop_vr joint_name_remapper
openarmx_teleop_vr openarmx_teleop_vr_node
openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

## Which Launch Files To Use

Use the OpenArm-specific launch files:

```bash
ros2 launch openarmx_teleop_vr openarm_teleop_vr_sim.launch.py
ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

Do not use the OpenArmX-only launch files for the standard OpenArm robot.

`openarm_teleop_vr_real.launch.py` starts:

```text
joint_name_remapper
openarmx_teleop_bridge_vr_node
openarmx_teleop_vr_node
```

The remapper reads OpenArm `/joint_states`, changes the `openarm_` joint-name
prefix into the `openarmx_` prefix expected by the IK stack, and publishes:

```text
/joint_states_remapped
```

The teleop node consumes `/joint_states_remapped` and publishes controller
commands to:

```text
/left_forward_position_controller/commands
/right_forward_position_controller/commands
```

## Fake Hardware Test

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

Validation:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /joint_states_remapped --once
ros2 topic hz /left_forward_position_controller/commands
ros2 topic hz /right_forward_position_controller/commands
```

Expected:

```text
/joint_states contains OpenArm original joint names.
/joint_states_remapped contains OpenArmX-compatible joint names.
Holding the Pico grip publishes controller commands.
RViz fake hardware moves before real CAN is connected.
```

## Real Hardware Notes

OpenarmXR README shows ordinary CAN setup. This machine's OpenArm has already
been validated with CAN FD, so keep using CAN FD:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
```

Confirm:

```bash
ip -details link show can0
ip -details link show can1
```

Expected:

```text
can <FD>
bitrate 1000000
dbitrate 5000000
```

Only after fake hardware is correct, launch the real robot:

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

Then start VR teleop in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs

ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

## Relationship To LeRobot

OpenarmXR is useful for direct VR teleoperation:

```text
Pico 4 -> OpenArm
```

It does not automatically record LeRobot datasets. It publishes ROS controller
commands, not LeRobot actions.

For future data collection, the cleaner integration is:

```text
OpenarmXR command topics
  -> adapter to LeRobot action/state format
  -> LeRobot record
```

This avoids running IK twice.

## Next Milestone

The next safe milestone is:

```text
Pico APK sends UDP
  -> bridge publishes VR topics
  -> teleop node publishes forward_position_controller commands
  -> fake hardware moves in RViz
```

Only after that should this path touch the real OpenArm.
