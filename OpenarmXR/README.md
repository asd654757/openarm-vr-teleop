# OpenarmXR 워크스페이스

VR 헤드셋(Pico / Meta Quest)을 이용한 OpenArm 양팔 로봇 텔레오퍼레이션 시스템.

## 패키지 구성

| 패키지 | 설명 | 사용 여부 |
|--------|------|----------|
| `openarm_description` | openarm v1.0 URDF/메쉬/RViz 설정 + openarmx 호환 변환 URDF | 사용 |
| `openarmx_description` | openarmx URDF/메쉬/RViz 설정 | 사용 |
| `openarmx_teleop_vr` | VR 텔레오퍼레이션 (UDP bridge + Python teleop 노드 + launch 파일) | 사용 |
| `openarmx_teleop_vr_apk` | VR 헤드셋용 APK (Pico, Meta Quest) | 사용 |
| `rci_openarm_controller` | C++ pinocchio IK 솔버 기반 VR teleop 노드 | 미사용 (실험용) |

> `rci_openarm_controller`는 openarmx_arm_driver 없이 동작하는 C++ IK 솔버 실험용 패키지입니다. 현재는 기존 openarmx Python teleop 파이프라인을 사용하므로 필요하지 않습니다.

## openarm vs openarmx

| | openarm | openarmx |
|---|---------|----------|
| 구조 | 7DOF 양팔 | 7DOF 양팔 (동일) |
| 좌표계/관절축 | 동일 | 동일 |
| 링크 길이/질량/관절 한계 | 다름 | 다름 |
| EE 프레임 (joint7 rotation) | `[[1,0,0],[0,-1,0],[0,0,-1]]` | `[[1,0,0],[0,-1,0],[0,0,-1]]` (동일) |
| link7_pico 프레임 | 없음 (변환 시 추가, rpy 재계산) | 있음 (rpy="0 3.1416 4.7124") |
| 메쉬 | `package://openarm_description/...` | `package://openarmx_description/...` |

### openarm을 openarmx teleop으로 사용하는 방법

openarm URDF를 `openarmx_arm_driver` IK 솔버와 호환되도록 변환합니다:
1. 관절/링크 이름: `openarm_` → `openarmx_` prefix 변경
2. `link7_pico` 프레임 추가 — openarm link7 기준 rpy 재계산 (`rpy="3.141593 0 1.570796"`)하여 openarmx와 동일한 world frame rotation 보장
3. `link4_ext` 프레임 추가
4. 메쉬 경로는 `package://openarm_description/` 유지 (openarm 외형 표시)
5. 기구학/동역학 파라미터는 openarm 값 유지

변환 스크립트: `openarm_description/scripts/convert_to_openarmx_compat.py`

## 사전 요구사항

```bash
source /opt/ros/humble/setup.bash
pip3 install openarmx-arm-driver
pip3 install "numpy<2"
sudo apt install adb   # VR APK 설치용
```

## VR 헤드셋 APK 설치

### Pico

1. 개발자 모드 활성화: `설정 > 정보 > 소프트웨어 버전 번호 여러 번 탭`
2. USB 디버깅 활성화: `설정 > 개발자 옵션 > USB 디버깅`
3. USB-C 케이블로 PC 연결
4. APK 설치:
```bash
adb install openarmx_teleop_vr_apk/openarmx-vr-pico.apk
```

### Meta Quest

1. 개발자 모드 활성화 (Meta 계정 + 개발자 등록 필요)
2. USB-C 케이블로 PC 연결, 헤드셋에서 USB 디버깅 허용
3. APK 설치:
```bash
adb install openarmx_teleop_vr_apk/openarmx-vr-quest.apk
```

## 빌드

```bash
cd ~/Project/OpenarmXR
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 실행 방법

### 1. 로봇 모델 시각화 (RViz + GUI 슬라이더)

```bash
# openarmx 모델
ros2 launch openarmx_description display_openarmx.launch.py arm_type:=v10 bimanual:=true

# openarm 모델
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true
```

### 2. VR 텔레오퍼레이션 시뮬레이션 (RViz + fake hardware)

VR 헤드셋에서 grip을 잡으면 RViz의 로봇이 움직입니다.
모든 시뮬레이션은 기존 openarmx Python teleop 노드(`openarmx_arm_driver` IK 솔버)를 사용합니다.

| 명령어 | 모델 | 설명 |
|--------|------|------|
| `ros2 launch openarmx_teleop_vr openarm_teleop_vr_sim.launch.py` | openarm | openarm 역학 + openarm 메쉬 |
| `ros2 launch openarmx_teleop_vr openarmx_teleop_vr_sim.launch.py` | openarmx | openarmx 역학 + openarmx 메쉬 |

**VR 헤드셋 설정:**
1. Pico 또는 Meta Quest에 APK 설치 (`openarmx_teleop_vr_apk/` 참조)
2. VR 헤드셋과 PC가 같은 네트워크에 연결
3. VR 앱 실행 후 PC IP 입력

> launch 전에 잔류 프로세스를 정리하세요:
```bash
pkill -9 -f "ros2_control_node|controller_manager|rviz2|bridge_vr|vr_teleop|robot_state_publisher|spawner|openarmx_teleop"
```

### 3. 실제 로봇 적용 전 테스트 (fake hardware + remapper)

CAN/모터 없이 bringup → remapper → teleop 전체 경로를 검증합니다.

```bash
# 터미널 1: bringup (fake hardware — CAN 불필요)
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    use_fake_hardware:=true \
    robot_controller:=forward_position_controller

# 터미널 2: VR teleop (remapper 포함)
ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

**검증 항목:**
```bash
ros2 topic echo /joint_states --once | grep "name:" -A 16         # openarm_ prefix 확인
ros2 topic echo /joint_states_remapped --once | grep "name:" -A 16 # openarmx_ prefix 변환 확인
ros2 topic echo /left_forward_position_controller/commands         # VR grip 시 명령 발행 확인
ros2 topic hz /left_forward_position_controller/commands           # 100Hz 확인
```

### 4. 실제 로봇 VR 텔레오퍼레이션 (openarm)

```bash
# 터미널 1: CAN 초기화 + 로봇 bringup
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up

cd ~/Project/OpenarmXR
source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    use_fake_hardware:=false \
    robot_controller:=forward_position_controller \
    right_can_interface:=can0 \
    left_can_interface:=can1

# 터미널 2: VR bridge + remapper + teleop
cd ~/Project/OpenarmXR
source install/setup.bash
ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

> `openarm_bringup`은 `openarm_ros2` 패키지에 포함되어 있습니다. `openarm_ros2`, `openarm_can` 패키지가 빌드되어 있어야 합니다.

> joint_name_remapper가 `/joint_states`(openarm_ prefix)를 `/joint_states_remapped`(openarmx_ prefix)로 변환하여 teleop 노드에 전달합니다.

### 5. 실제 로봇 VR 텔레오퍼레이션 (openarmx)

```bash
# 터미널 1: 로봇 bringup (openarmx_bringup 필요)
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    use_fake_hardware:=false \
    robot_controller:=forward_position_controller

# 터미널 2: VR bridge + teleop
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 런치 파일 요약

| 런치 파일 | 모델 | 용도 |
|-----------|------|------|
| `openarm_teleop_vr_sim.launch.py` | openarm (변환 URDF) | VR 시뮬레이션 |
| `openarmx_teleop_vr_sim.launch.py` | openarmx | VR 시뮬레이션 |
| `openarm_teleop_vr_real.launch.py` | openarm (실제 로봇) | 실제 로봇 VR teleop |
| `teleop_vr.launch.py` | openarmx | 실제 로봇 VR teleop |
| `display_openarmx.launch.py` | openarmx | 모델 시각화 |
| `display_openarm.launch.py` | openarm | 모델 시각화 |

> `teleop_vr_sim.launch.py`는 `rci_openarm_controller` C++ 노드 실험용입니다. 현재 사용하지 않습니다.

## 데이터 흐름

```
VR 헤드셋 (Pico / Meta Quest)
    │  UDP (port 5100)
    ▼
openarmx_teleop_bridge_vr_node (C++)
    │  ROS2 토픽 (PoseStamped, Float32, Bool)
    ▼
openarmx_teleop_vr_node (Python, openarmx_arm_driver IK)
    │  Float64MultiArray
    ▼
forward_position_controller (ros2_control)
    │  관절 명령
    ▼
로봇 하드웨어 또는 fake hardware → joint_states → robot_state_publisher → RViz
```

## URDF 변환 재생성

openarm URDF가 변경되면 변환 URDF를 재생성해야 합니다:

```bash
# IK용 (ros2_control 없음)
python3 src/openarm_description/scripts/convert_to_openarmx_compat.py \
  src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10_bimanual.urdf \
  src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10_bimanual_openarmx_compat.urdf

# sim용 (ros2_control + fake hw 포함) — xacro 먼저 처리
ros2 run xacro xacro src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  arm_type:=v10 body_type:=v10 bimanual:=true ros2_control:=true \
  use_fake_hardware:=true fake_sensor_commands:=false > /tmp/raw.urdf

python3 src/openarm_description/scripts/convert_to_openarmx_compat.py \
  /tmp/raw.urdf \
  src/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10_bimanual_openarmx_compat_sim.urdf

colcon build --packages-select openarm_description
```
