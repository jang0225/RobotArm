# ROS 2 Dynamixel Robot Arm

Dynamixel 모터 3개(관절 2개와 그리퍼 1개)로 구성한 로봇암을 ROS 2와 C++로 제어하는 시작 프로젝트입니다. 현재 구현은 Dynamixel SDK를 직접 사용하며 다음 인터페이스를 제공합니다.

- `joint_commands` (`sensor_msgs/msg/JointState`): joint 이름과 목표 위치(rad) 명령
- `joint_states` (`sensor_msgs/msg/JointState`): 현재 joint 위치(rad)
- `set_torque` (`std_srvs/srv/SetBool`): 전체 모터 torque on/off

## 프로젝트 구조

```text
src/robot_arm_controller/
├── CMakeLists.txt
├── package.xml
├── config/robot_arm.yaml
├── launch/robot_arm.launch.py
└── src/dynamixel_arm_node.cpp
```

## 1. 하드웨어 준비

PC와 Dynamixel 사이에 U2D2 같은 USB 시리얼 어댑터와 **별도 모터 전원**이 필요합니다. 모터 전원을 넣기 전에 로봇암을 받침대에 고정하고, 각 관절이 갑자기 움직여도 사람이나 케이블을 치지 않는지 확인하세요.

현재 설정은 Wizard에서 확인한 XD430-T350 두 대(ID 1, 2)와 XM430-W350 한 대(ID 3), Protocol 2.0, 1 Mbps를 기준으로 합니다. 실제 장치에서 아래 항목을 반드시 확인해야 합니다.

- 각 모터의 고유 ID와 baud rate
- Torque Enable, Goal Position, Present Position 주소
- 위치 데이터 길이(2 또는 4 byte)
- Position Control Mode 설정
- 1 tick당 각도, 각 관절의 영점과 회전 방향

Linux에서 시리얼 장치가 보이는지 확인합니다.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo usermod -aG dialout "$USER"
```

그룹 추가 후에는 로그아웃/로그인이 필요합니다. 장치 이름은 환경에 맞춰 `robot_arm.yaml`에서 변경하세요.

## 2. 의존성 및 빌드

ROS 2 환경을 먼저 source한 뒤 워크스페이스 루트에서 실행합니다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt install ros-$ROS_DISTRO-dynamixel-sdk
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. 로봇 설정

[`robot_arm.yaml`](src/robot_arm_controller/config/robot_arm.yaml)을 실제 로봇에 맞게 수정합니다. 처음에는 다음을 특히 확인하세요.

1. `motor_ids`와 `joint_names`의 순서
2. `direction` (`1.0` 또는 `-1.0`)
3. 로봇의 기계적 0 rad에 해당하는 `zero_position_ticks`
4. 충돌을 피할 수 있는 보수적인 `min_position_rad`, `max_position_rad`
5. 모터 모델에 맞는 control table 주소

안전을 위해 `auto_enable_torque` 기본값은 `false`입니다.

## 4. 실행 및 첫 동작

노드를 실행하면 먼저 모든 설정 ID에 ping을 보내며 하나라도 응답하지 않으면 시작을 중단합니다.
모델 번호와 Operating Mode도 확인하며 현재는 세 모터 모두 Position Control Mode 값 `3`을 요구합니다.

```bash
ros2 launch robot_arm_controller robot_arm.launch.py
```

다른 터미널에서 상태가 정상인지 확인합니다.

```bash
source install/setup.bash
ros2 topic echo /joint_states
```

로봇을 안전한 자세로 지지한 상태에서 torque를 켭니다.

```bash
ros2 service call /set_torque std_srvs/srv/SetBool "{data: true}"
```

한 관절에 작은 명령부터 보냅니다. 이름이 없는 관절은 움직이지 않습니다.

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
  "{name: [joint1], position: [0.1]}"
```

두 관절과 그리퍼를 함께 제어하는 예시입니다. 여기서 그리퍼 위치는 모터축의 radian 값이며 실제 개폐 폭이 아닙니다.

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, gripper_joint], position: [0.0, 0.2, 0.1]}"
```

그리퍼만 제어하는 예시:

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
  "{name: [gripper_joint], position: [0.1]}"
```

작업을 마치면 torque를 끕니다.

```bash
ros2 service call /set_torque std_srvs/srv/SetBool "{data: false}"
```

## 안전 동작

- 설정된 radian 제한 밖의 명령과 `NaN`/무한대 명령은 거부합니다.
- torque가 꺼진 동안 위치 명령은 무시합니다.
- 종료할 때 기본적으로 torque를 해제합니다.
- 통신 오류 시 해당 joint state는 `NaN`으로 발행합니다.

이 코드는 실험용 시작점입니다. 실제 장비에 적용할 때는 비상정지, 통신 watchdog, 속도/가속도 제한, 전류 및 온도 감시, self-collision 방지를 추가해야 합니다. MoveIt 2 또는 `ros2_control` 연동은 기본 하드웨어 통신이 검증된 다음 단계로 진행하는 것이 좋습니다.
