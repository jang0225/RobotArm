# ROS 2 Dynamixel Robot Arm

ROS 2 Humble과 C++로 Dynamixel 로봇팔을 제어하는 프로젝트입니다. ID 1·2·3은 관절, ID 4는 그리퍼이며 제어 경로는 `ros2_control` 하나로 통일되어 있습니다.

설계 과정, 보정값, 트러블슈팅 및 포트폴리오용 기술 요약은 [`PROJECT_DEVELOPMENT_LOG.md`](PROJECT_DEVELOPMENT_LOG.md)에 정리되어 있습니다. FSS와 함께 운용하는 방법은 [`FSS_INTEGRATION.md`](FSS_INTEGRATION.md)를 참고하세요.

## 구성

```text
src/
├── robot_arm_hardware/    # Dynamixel SDK 기반 ros2_control 플러그인
├── robot_arm_bringup/     # URDF, controller 설정, launch
└── robot_arm_controller/  # degree 변환, FSS 안전 연동 및 선택적 모터 시험 노드
```

하드웨어 설정은 다음을 기준으로 합니다.

| ID | 모델 | 역할 | Operating/Drive Mode |
|---:|---|---|---|
| 1 | XD430-T350 | `joint1` | 3 / 0 |
| 2 | XD430-T350 | `joint2` | 3 / 0 |
| 3 | XM430-W350 | `joint3` | 3 / 0 |
| 4 | XM430-W350 | `gripper_joint` | 3 / 4 |

통신은 Protocol 2.0, 1 Mbps, 기본 장치 `/dev/ttyUSB0`을 사용합니다. Dynamixel Wizard는 ROS 노드를 실행하기 전에 종료해야 합니다.

## 설치와 빌드

```bash
cd ~/robotarm
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-dynamixel-sdk ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

시리얼 장치와 권한 확인:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo usermod -aG dialout "$USER"
```

그룹을 처음 추가했다면 로그아웃 후 다시 로그인합니다.

## 안전한 arm-only 실행

그리퍼를 제외하고 세 관절만 시험하려면 ID 1·2·3만 활성화하는 arm-only 모드를 사용합니다.

```bash
cd ~/robotarm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_bringup arm_only.launch.py
```

이 launch는 하드웨어 상태를 읽은 후 그 위치를 초기 목표로 설정하고 ID 1·2·3 torque를 활성화합니다. 로봇을 고정하고 주변을 비운 상태에서 실행하세요.

상태 확인:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

`/joint_states`는 ROS 표준에 따라 radian 단위입니다.

## 중앙 0도 기준 제어

사용자 명령은 degree이고 내부에서 radian과 Dynamixel tick으로 자동 변환됩니다.

| joint | 사용자 0도에 해당하는 절대각/tick | 허용 명령 범위 |
|---|---:|---:|
| `joint1` | 179.296875° / 2040 | -98.456875° ~ 98.543125° |
| `joint2` | 224.384766° / 2553 | -113.134766° ~ 113.135234° |
| `joint3` | 208.740234° / 2375 | -103.290234° ~ 103.209766° |
| `gripper_joint` | 임시 2305 tick | -5° ~ 5° |

한 관절을 3초 동안 5도로 이동:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1], positions_deg: [5.0], duration_sec: 3.0}"
```

네 모터를 동시에 이동:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3, gripper_joint], positions_deg: [5.0, -5.0, 8.0, 2.0], duration_sec: 3.0}"
```

`0°`는 각 모터의 보정된 중앙입니다. 현재 위치와 멀다면 작은 목표를 긴 시간으로 나누어 보내세요. 제한을 벗어난 유한한 degree 명령은 해당 관절의 최소·최대값으로 자동 제한됩니다. 한 관절이 제한에 걸리거나 잘못된 값을 받아도 다른 관절의 명령은 계속 처리됩니다.

## 선택적 반복 모터 시험

`fish_motion_node`는 여러 모터의 반복 동작을 빠르게 확인하기 위한 보조 시험 도구입니다. 핵심 제어 경로에는 사용되지 않습니다.

```bash
ros2 run robot_arm_controller fish_motion_node --ros-args \
  -p joint1_amplitude_deg:=4.0 \
  -p joint2_amplitude_deg:=8.0 \
  -p joint3_amplitude_deg:=12.0 \
  -p period_sec:=4.0 \
  -p cycles:=3 \
  -p include_gripper:=false
```

## 전체 4모터 실행

현재 ID 4는 실행 시 확인한 2305 tick을 임시 0도로 사용하고 `±5°`로 제한됩니다. 현재 USB 포트명을 확인한 뒤 실행합니다.

```bash
ros2 launch robot_arm_bringup ros2_control.launch.py device_name:=/dev/ttyUSB1
```

그리퍼 조립 후 절대 최소·최대 각도와 중앙 tick을 다시 측정하여 [`robot_arm.urdf.xacro`](src/robot_arm_bringup/urdf/robot_arm.urdf.xacro)의 임시 값을 교체하세요.

표준 radian 궤적은 `/arm_trajectory_controller/joint_trajectory`에 직접 보낼 수도 있습니다.

## FSS 연동 실행

FSS 코드를 수정하지 않고 RobotArm을 overlay workspace로 빌드할 수 있습니다. 통합 모드에서는 모든 로봇팔 토픽이 `/robot_arm` 아래에 배치되고, FSS가 `ACTIVE`이며 health가 정상일 때만 요청 궤적이 controller로 전달됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/FSS_FSW/install/setup.bash

cd ~/robotarm
colcon build --symlink-install
source install/setup.bash

ros2 launch robot_arm_bringup fss_robot_arm.launch.py \
  device_name:=/dev/serial/by-id/YOUR_U2D2_DEVICE \
  start_fss:=false \
  arm_only:=true
```

기본값은 이미 실행 중인 FSS에 RobotArm만 연결합니다. FSS 전체 시스템도 같은 launch에서 시작하려면 사전 점검 후 `start_fss:=true`를 명시하세요.

통합 모드의 degree 명령:

```bash
ros2 topic pub --once /robot_arm/joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3], positions_deg: [5.0, 10.0, -5.0], duration_sec: 5.0}"
```

상세한 빌드 순서, 토픽 계약, FSS 모드별 동작 및 제한사항은 [`FSS_INTEGRATION.md`](FSS_INTEGRATION.md)에 정리되어 있습니다.

## 안전 주의사항

- 모터에는 별도 전원과 U2D2 같은 통신 장치가 필요합니다.
- 전원을 넣기 전에 로봇을 고정하고 비상정지 수단을 준비하세요.
- Wizard에서 EEPROM을 변경할 때는 먼저 Torque Enable을 0으로 만드세요.
- 실제 링크 길이, 관성, 충돌 형상과 그리퍼 제한은 하드웨어 완성 후 반드시 보정하세요.
- 실사용 전 통신 watchdog, 전류·온도 감시, 충돌 방지를 추가하세요.
