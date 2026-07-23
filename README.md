# ROS 2 Dynamixel Robot Arm

ROS 2 Humble과 C++로 Dynamixel 로봇팔을 제어하는 프로젝트입니다. ID 1·2는 관절, ID 3은 그리퍼이며 제어 경로는 `ros2_control` 하나로 통일되어 있습니다.

## 구성

```text
src/
├── robot_arm_hardware/    # Dynamixel SDK 기반 ros2_control 플러그인
├── robot_arm_bringup/     # URDF, controller 설정, launch
└── robot_arm_controller/  # degree 명령 변환 및 물고기 궤적 노드
```

하드웨어 설정은 다음을 기준으로 합니다.

| ID | 모델 | 역할 | Operating/Drive Mode |
|---:|---|---|---|
| 1 | XD430-T350 | `joint1` | 3 / 0 |
| 2 | XD430-T350 | `joint2` | 3 / 0 |
| 3 | XM430-W350 | `gripper_joint` | 3 / 0 |

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

그리퍼 범위를 아직 측정하지 않았으므로 현재는 ID 3에 torque를 걸지 않는 arm-only 모드를 사용합니다.

```bash
cd ~/robotarm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_arm_bringup arm_only.launch.py
```

이 launch는 하드웨어 상태를 읽은 후 그 위치를 초기 목표로 설정하고 ID 1·2 torque를 활성화합니다. 로봇을 고정하고 주변을 비운 상태에서 실행하세요.

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

한 관절을 3초 동안 5도로 이동:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1], positions_deg: [5.0], duration_sec: 3.0}"
```

두 관절을 동시에 이동:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2], positions_deg: [5.0, -5.0], duration_sec: 3.0}"
```

`[0.0, 0.0]`은 두 관절의 중앙입니다. 현재 위치와 멀다면 작은 목표를 긴 시간으로 나누어 보내세요. 제한을 벗어난 degree 명령은 변환 노드와 하드웨어 플러그인에서 거부됩니다.

## 물고기처럼 흔들리는 궤적

`arm_only.launch.py` 실행 중 다른 터미널에서 작고 느린 설정부터 시험합니다.

```bash
cd ~/robotarm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_arm_controller fish_motion_node --ros-args \
  -p joint1_amplitude_deg:=4.0 \
  -p joint2_amplitude_deg:=8.0 \
  -p phase_lag_deg:=60.0 \
  -p period_sec:=4.0 \
  -p cycles:=10 \
  -p center_duration_sec:=5.0
```

노드는 먼저 중앙으로 이동하고, 두 관절을 위상차를 두어 흔든 뒤 중앙으로 복귀합니다.

실행 옵션을 생략하면 `joint1 = 15°`, `joint2 = 30°`, 10회 왕복으로 동작합니다.

- `joint1_amplitude_deg`, `joint2_amplitude_deg`: 관절별 진폭
- `phase_lag_deg`: `joint2`가 뒤따르는 위상차
- `period_sec`: 한 번 왕복하는 시간
- `cycles`: 왕복 횟수(기본 10회, 최대 100회)
- `center_duration_sec`: 시작 시 중앙까지 이동하는 시간

## 전체 3축 실행

그리퍼 조립 후 절대 최소·최대 각도와 중앙 tick을 측정하여 [`robot_arm.urdf.xacro`](src/robot_arm_bringup/urdf/robot_arm.urdf.xacro)의 gripper 값을 수정한 다음 실행합니다.

```bash
ros2 launch robot_arm_bringup ros2_control.launch.py
```

표준 radian 궤적은 `/arm_trajectory_controller/joint_trajectory`에 직접 보낼 수도 있습니다. FSS나 MoveIt 2는 이 표준 controller 인터페이스에 연결하면 됩니다.

## 안전 주의사항

- 모터에는 별도 전원과 U2D2 같은 통신 장치가 필요합니다.
- 전원을 넣기 전에 로봇을 고정하고 비상정지 수단을 준비하세요.
- Wizard에서 EEPROM을 변경할 때는 먼저 Torque Enable을 0으로 만드세요.
- 실제 링크 길이, 관성, 충돌 형상과 그리퍼 제한은 하드웨어 완성 후 반드시 보정하세요.
- 실사용 전 통신 watchdog, 전류·온도 감시, 충돌 방지를 추가하세요.
