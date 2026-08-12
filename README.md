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
| 4 | XM430-W350 | `gripper_joint` | 4 / 0 |

통신은 Protocol 2.0, 1 Mbps, 기본 장치 `/dev/ttyUSB0`을 사용합니다. Dynamixel Wizard는 ROS 노드를 실행하기 전에 종료해야 합니다.

ID 4는 Wizard에서 Torque Enable을 `0`으로 만든 뒤 Operating Mode를 `4`
(Extended Position Control), Drive Mode를 `0`으로 설정합니다. ID 1~3은
Operating Mode `3`을 유지합니다.

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

## 기준 0도 제어

사용자 명령은 degree이고 내부에서 radian과 Dynamixel tick으로 자동 변환됩니다.

| joint | 사용자 0도에 해당하는 절대각/tick | 허용 명령 범위 |
|---|---:|---:|
| `joint1` | 179.296875° / 2040 | -98.456875° ~ 98.543125° |
| `joint2` | 224.384766° / 2553 | -113.134766° ~ 113.135234° |
| `joint3` | 208.740234° / 2375 | -103.290234° ~ 103.209766° |
| `gripper_joint` | 202.587891° / 2305 tick | `0°`(닫힘) ~ `193.271484°`(열림) |

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

관절은 degree, 그리퍼는 cm로 한 번에 명령하려면 전용 combined 메시지를 사용한다.
`duration_sec`를 줄이면 같은 목표까지 더 빠르게 움직이며, 큰 이동은 우선 `5~8초`로
시험한 뒤 기구 하중과 간섭을 확인한다.

```bash
ros2 topic pub --once /arm_gripper_commands \
  robot_arm_controller/msg/ArmGripperCommand \
  "{joint1_deg: 25.0, joint2_deg: -35.0, joint3_deg: 40.0, gripper_opening_cm: 6.5, duration_sec: 5.0}"
```

정렬과 그리퍼 닫힘도 같은 단위 체계로 명령할 수 있다.

```bash
ros2 topic pub --once /arm_gripper_commands \
  robot_arm_controller/msg/ArmGripperCommand \
  "{joint1_deg: 0.0, joint2_deg: 0.0, joint3_deg: 0.0, gripper_opening_cm: 0.0, duration_sec: 5.0}"
```

`joint1`~`joint3`의 `0°`는 각 관절의 보정된 중앙이다. 반면 `gripper_joint`의
사용자 명령은 완전 닫힘을 `0°`, 열린 방향을 양수로 사용한다. 내부 제어 각도는
SDK로 직접 측정한 닫힘 `5849 tick`, 열림 `3650 tick`을 그대로 사용한다.
기구가 닫힘에서 열림까지 음의 방향으로 `193.271484°` 회전해야 하므로 ID 4에는
Extended Position Control mode 4가 필요하다.
현재 위치와 멀다면 작은 목표를 긴 시간으로 나누어 보내세요. 제한을 벗어난 유한한 degree 명령은
해당 관절의 최소·최대값으로 자동 제한됩니다. 한 관절이 제한에 걸리거나 잘못된 값을
받아도 다른 관절의 명령은 계속 처리됩니다.

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

## 그리퍼 열림 거리(cm) 제어

그리퍼는 `0 cm = 완전 닫힘`, `8.65 cm = 설정된 안전 최대 열림`으로 명령할 수 있다.
`gripper_opening_bridge`가 현재 위치를 사용하지 않고 cm를 실측 내부 절대각으로
변환해 trajectory controller 또는 FSS supervisor 입력으로 직접 전달한다.

```bash
ros2 topic pub --once /gripper_opening_cm \
  robot_arm_controller/msg/GripperCommandCm \
  "{opening_cm: 5.0, duration_sec: 3.0}"
```

명령은 `0.0`~`8.65 cm` 범위에서 자동 제한된다. 이 변환은 현재 선형 보정이다.
링크식 그리퍼의 실제 간격이 중간 위치에서 다르면, 여러 간격 측정값을 이용한 보정표로
확장해야 한다.

각 명령은 현재 위치로부터의 이동량이 아니라 절대 열림 목표다. 예를 들어 벌어진
상태에서 `opening_cm: 0.0`을 보내면 완전 닫힘 위치로 이동한다.

```text
0 cm  → motor 514.072266° → controller 311.484375° → tick 5849
8.65 cm → motor 320.800781° → controller 118.212891° → tick 3650
```

## 전체 4모터 실행

그리퍼 사용자 명령에서 `0°`는 닫힘, `193.271484°`는 열림 끝이다. signed 모터 각은
닫힘 `514.072266°`, 열림 `320.800781°`이며 내부 controller 좌표로 변환해 사용한다. 현재 USB 포트명을
확인한 뒤 실행합니다.

```bash
ros2 launch robot_arm_bringup ros2_control.launch.py device_name:=/dev/ttyUSB1
```

그리퍼 조립 후 절대 최소·최대 각도와 **완전 닫힘 기준 tick**을 측정하여
[`robot_arm_calibration.yaml`](src/robot_arm_bringup/config/robot_arm_calibration.yaml)을
수정하세요. 이 파일 하나가 Xacro, degree bridge, FSS 궤적 검증에 모두 적용됩니다.

기구물을 새로 조립하거나 링크 길이가 달라졌다면 모터 제한값만 수정하면 충분하지
않습니다. [`robot_arm.urdf.xacro`](src/robot_arm_bringup/urdf/robot_arm.urdf.xacro)의
`link1_length`, `link2_length`, `link3_length`와 각 link의 visual/collision geometry,
질량, 관성도 실제 사양으로 갱신해야 합니다. 이 값은 RViz 표현, MoveIt 2 경로 계획,
충돌 판정과 동역학 계산의 기준이 됩니다.

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

FSS 통합 launch에서는 로봇팔 hardware와 controller가 비활성 상태로 시작합니다.
정상 health의 `ACTIVE` mode가 확인된 뒤에만 torque와 controller가 활성화됩니다.
`SAFE_HOLD`와 일반 mode 이탈은 현재 위치를 유지하고,
`EMERGENCY_STOP` 또는 mode timeout은 controller를 정지한 뒤 hardware를
`inactive`로 전환하여 Dynamixel torque를 해제합니다.

통합 모드의 degree 명령:

```bash
ros2 topic pub --once /robot_arm/joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3], positions_deg: [5.0, 10.0, -5.0], duration_sec: 5.0}"
```

상세한 빌드 순서, 토픽 계약, FSS 모드별 동작 및 제한사항은 [`FSS_INTEGRATION.md`](FSS_INTEGRATION.md)에 정리되어 있습니다.

## 진단과 통신 watchdog

각 Dynamixel에는 500 ms Bus Watchdog이 활성화됩니다. 제어 프로세스나 USB 통신이
중단되어 instruction packet이 더 이상 도착하지 않으면 모터 자체가 정지합니다.
일시적인 read 누락은 이전 상태를 유지해 최대 3회까지 허용하지만, 연속 통신 실패나
Hardware Error Status는 전체 로봇팔 안전 오류로 처리합니다. 이는 관절 한계 포화와
별개이므로 한 관절의 각도 제한 도달은 여전히 다른 관절에 영향을 주지 않습니다.

온도, 전압, 전류, 통신 상태와 Hardware Error는 `/diagnostics`로 확인합니다.

```bash
ros2 topic echo /diagnostics
ros2 topic echo /dynamic_joint_states
```

표준 radian 궤적은 관절 이름, 배열 크기, 유한값, 시간 순서, 최대 속도와 각도 제한을
검증합니다. 범위를 벗어난 유한한 위치는 해당 관절만 포화되며 다른 관절 목표는
그대로 유지됩니다.

## 안전 주의사항

- 모터에는 별도 전원과 U2D2 같은 통신 장치가 필요합니다.
- 전원을 넣기 전에 로봇을 고정하고 비상정지 수단을 준비하세요.
- Wizard에서 EEPROM을 변경할 때는 먼저 Torque Enable을 0으로 만드세요.
- 링크를 변경하거나 재조립했다면 URDF의 길이, 질량, 관성, visual/collision 형상과
  calibration의 관절 제한을 함께 갱신하세요.
- 소프트웨어 Bus Watchdog과 torque 해제는 물리 비상정지 회로를 대체하지 않습니다.
- 실제 링크 치수와 collision geometry가 완성되기 전에는 자동 경로 계획을 사용하지 마세요.
