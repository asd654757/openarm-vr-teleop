# OpenArm Gravity Compensation

This package computes model-based gravity feedforward torque for the OpenArm
V10 arms. It follows the OpenArmX controller topology:

```text
/joint_states
  -> KDL JntToGravity
  -> per-joint clamp and scale ramp
  -> forward effort controller
  -> OpenArm MIT tau_ff
```

The KDL chains start at `openarm_body_link0`, so the fixed left and right arm
mount transforms are included in the dynamics model. They end at each
`hand_tcp`, so the hand body inertia is included as an end load. The two
movable finger branches are not included in the seven-axis KDL chains. No
OpenArmX motor direction multiplier is copied into this implementation.

## Safety defaults

- Gravity compensation is disabled by default in the bimanual launch.
- `apply_gravity_compensation:=false` is a dry run and only publishes preview
  topics.
- Live output starts at zero and ramps over 3 seconds.
- Stale or incomplete joint state data ramps the output back to zero.
- Output is clamped per joint to `[20, 20, 7, 7, 2, 2, 2]` Nm.
- Left and right arms can be tested independently.

Preview topics:

```text
/gravity_compensation/left_effort
/gravity_compensation/right_effort
```

## CAN-free dynamics validation

The validator loads the same ROS URDF into MuJoCo, compares KDL gravity torque
against both the URDF potential-energy gradient and MuJoCo inverse dynamics,
then measures arm sag with and without feedforward. It also runs a two-second
position hold using the gains from `openarm_hardware` and gravity updates at
100 Hz.

Generate the URDF and run the validator without starting any CAN interface:

```bash
cd ~/OPENARM
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run xacro xacro \
  ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
  bimanual:=true ros2_control:=false use_fake_hardware:=true \
  -o /tmp/openarm_v10_bimanual.urdf

MUJOCO_GL=egl /home/w/anaconda3/envs/oplerobot310/bin/python \
  ros2_ws/install/openarm_gravity_comp/lib/openarm_gravity_comp/validate_mujoco_gravity.py \
  --urdf /tmp/openarm_v10_bimanual.urdf \
  --calculator ros2_ws/install/openarm_gravity_comp/lib/openarm_gravity_comp/gravity_comp_calculate
```

The script exits nonzero unless static torque error is below 1%, free-sag
reduction exceeds 80%, and the two-second compensated position-hold error is
below the configured limits. It disables MuJoCo's URDF fixed-body fusion;
leaving fusion enabled changes the transformed mount/hand inertias and is not
a valid comparison with the KDL tree.

## Dry-run validation

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller \
  enable_gravity_compensation:=true \
  apply_gravity_compensation:=false \
  gravity_ramp_duration_s:=10.0 \
  gravity_scale:=0.2
```

## First live test

Do this only after checking the preview sign while manually supporting the arm.
Start with one arm and a low scale:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  enable_gravity_compensation:=true \
  apply_gravity_compensation:=true \
  gravity_enable_left:=true \
  gravity_enable_right:=false \
  gravity_scale:=0.1
```

Use `ros2 param set /openarm_gravity_comp enabled false` to ramp the live
gravity output to zero. Stop the launch to deactivate hardware and disarm the
motors.
