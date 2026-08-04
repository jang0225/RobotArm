# FSS–RobotArm 연동 가이드

> 대상 FSS 저장소: `SNU-SMRL/FSS_FSW`
>
> 최종 갱신: 2026-07-31

## 1. 연동 원칙

FSS_FSW 소스는 수정하지 않는다. FSS를 먼저 빌드한 underlay로 사용하고, RobotArm을 그 위에 overlay로 빌드한다.

RobotArm 쪽에 추가한 `fss_arm_supervisor_node`가 FSS의 기존 토픽을 구독하여 로봇팔 명령을 허용하거나 차단한다. 기존 FSS manager, GNC, valve driver 및 interface 정의에는 변경이 필요 없다.

```text
FSS_FSW
├── /system/mode ─────────┐
└── /system/health ───────┤
                          ▼
             fss_arm_supervisor_node
                          │
       요청 궤적 허용·차단 및 안전 hold
                          ▼
       arm_trajectory_controller
                          ▼
        ros2_control / Dynamixel
```

## 2. 추가된 구성요소

### `fss_arm_supervisor_node`

위치:

```text
src/robot_arm_controller/src/fss_arm_supervisor_node.cpp
```

역할:

- FSS `/system/mode` 구독
- FSS `/system/health` 구독
- 외부 로봇팔 궤적 요청을 내부 controller 토픽으로 전달
- FSS 모드 또는 health가 안전 조건을 벗어나면 새 명령 차단
- 일반 mode 이탈 또는 health 오류에서는 최신 관절 위치 hold
- `EMERGENCY_STOP`과 mode timeout에서는 controller를 중지하고 hardware inactive 전환
- 정상 `ACTIVE` 복귀 시 hardware와 controller를 순서대로 활성화
- 표준 궤적의 형식, 관절 제한, 시간과 최대 속도 검증
- mode와 health 메시지 timeout 감시
- 현재 게이트 상태를 `/robot_arm/fss_supervisor_status`로 발행

### `fss_robot_arm.launch.py`

위치:

```text
src/robot_arm_bringup/launch/fss_robot_arm.launch.py
```

다음 항목을 시작한다.

- namespaced RobotArm `ros2_control`
- degree 명령 변환 노드
- FSS arm supervisor
- `start_fss:=true`일 때만 FSS `fss_full.launch.py`

그리퍼가 완성되기 전까지 기본값 `arm_only:=true`를 사용한다.

### Namespaced controller 설정

통합 모드 전용 설정:

```text
src/robot_arm_bringup/config/controllers_fss.yaml
src/robot_arm_bringup/config/controllers_fss_arm_only.yaml
```

controller manager와 모든 로봇팔 토픽을 `/robot_arm` namespace에 둔다. FSS의 `/system`, `/navigation`, `/actuators`, `/gnc` 토픽과 이름이 겹치지 않는다.

## 3. 명령 게이트 정책

기본값에서는 다음 조건이 모두 참일 때만 로봇팔 궤적을 controller로 전달한다.

1. `/system/mode`를 최근 2.5초 안에 수신
2. FSS mode가 `ACTIVE`
3. `/system/health`를 최근 2.5초 안에 수신
4. `navigation_ok == true`
5. `actuators_ok == true`
6. `errors` 배열이 비어 있음

FSS 통합 구성은 hardware와 controller를 inactive로 시작하며, 정상 `ACTIVE`가
확인되기 전에는 torque를 켜지 않는다.

| FSS 상태 | 로봇팔 명령 | 전환 시 동작 |
|---|---|---|
| `ACTIVE` + 정상 health | 허용 | hardware/controller 활성화 후 검증된 궤적 전달 |
| `IDLE`, `PRESSURIZING`, `READY` | 차단 | 이미 활성 상태면 현재 위치 hold |
| `SAFE_HOLD` | 차단 | 현재 위치 hold |
| `EMERGENCY_STOP` | 차단 | controller 중지, hardware inactive, torque 해제 |
| mode timeout | 차단 | controller 중지, hardware inactive, torque 해제 |
| health timeout 또는 비정상 health | 차단 | 현재 위치 hold |

E-stop은 latch된다. 이후 mode가 바뀌어도 torque가 자동으로 켜지지 않으며,
정상 health의 `ACTIVE`가 다시 확인되어야 활성화된다.

`require_healthy:=false`는 벤치 디버깅용이다. 실제 FSS 운용에서는 기본값 `true`를 유지한다.

## 4. 토픽 계약

### FSS에서 받는 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/system/mode` | `fss_interfaces/msg/SystemMode` | FSS 운용 모드 |
| `/system/health` | `fss_interfaces/msg/SystemHealth` | FSS 건강 상태 |

### RobotArm 외부 인터페이스

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/robot_arm/joint_commands_deg` | `robot_arm_controller/msg/JointCommandDegrees` | 사용자 degree 명령 |
| `/robot_arm/gripper_opening_cm` | `robot_arm_controller/msg/GripperCommandCm` | 그리퍼 손가락 간격 명령 |
| `/robot_arm/requested_joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 표준 radian 궤적 요청 |
| `/robot_arm/joint_states` | `sensor_msgs/msg/JointState` | 관절 측정 상태 |
| `/robot_arm/fss_supervisor_status` | `std_msgs/msg/String` | 명령 허용·차단 상태 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 모터 통신·전류·전압·온도·오류 |

### 내부 전용 토픽

```text
/robot_arm/arm_trajectory_controller/joint_trajectory
```

이 토픽은 supervisor가 검증한 궤적을 controller에 전달하는 내부 경로다. FSS 통합 운용 중에는 이 토픽에 직접 publish하지 않는다. 직접 publish하면 supervisor의 안전 게이트를 우회한다.

## 5. 빌드 순서

`fss_arm_supervisor_node`는 `fss_interfaces`가 발견될 때만 선택적으로 빌드된다. 따라서 반드시 FSS를 먼저 빌드하고 source한 뒤 RobotArm을 빌드한다.

### FSS underlay

```bash
cd ~/FSS_FSW
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

기존 CMake cache에서 FSS를 찾지 못한 상태가 저장된 경우 다음처럼 강제 configure한다.

```bash
colcon build --symlink-install --cmake-force-configure
source install/setup.bash
```

### RobotArm overlay

```bash
cd ~/robotarm
source /opt/ros/humble/setup.bash
source ~/FSS_FSW/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

supervisor가 설치됐는지 확인한다.

```bash
ros2 pkg executables robot_arm_controller
```

목록에 다음 실행 파일이 있어야 한다.

```text
robot_arm_controller fss_arm_supervisor_node
```

FSS를 source하지 않고 RobotArm만 빌드하면 기존 독립 제어 기능은 정상 빌드되지만 선택적 supervisor 실행 파일은 생성되지 않는다.

## 6. USB 장치 이름 고정

FSS의 Xsens IMU와 RobotArm의 U2D2가 모두 `/dev/ttyUSB*`로 인식될 수 있다. 연결 순서에 따라 번호가 바뀔 수 있으므로 통합 운용에서는 `/dev/ttyUSB0` 같은 번호를 고정값으로 사용하지 않는다.

```bash
ls -l /dev/serial/by-id/
```

U2D2에 해당하는 경로를 찾아 launch argument로 전달한다.

```text
/dev/serial/by-id/usb-ROBOTIS_Co._Ltd._ROBOTIS_OpenCM_or_U2D2_...
```

실제 출력된 전체 경로를 그대로 사용해야 한다.

## 7. 통합 실행

Dynamixel Wizard를 종료하고 로봇팔을 고정한 뒤 실행한다.

### 권장: FSS와 RobotArm을 별도 launch로 실행

FSS는 기존 운용 절차에 따라 먼저 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/FSS_FSW/install/setup.bash
ros2 launch fss_bringup fss_full.launch.py
```

다른 터미널에서 RobotArm과 supervisor를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/FSS_FSW/install/setup.bash
source ~/robotarm/install/setup.bash

ros2 launch robot_arm_bringup fss_robot_arm.launch.py \
  device_name:=/dev/serial/by-id/YOUR_U2D2_DEVICE \
  start_fss:=false \
  arm_only:=true
```

### 선택: 하나의 launch에서 함께 시작

FSS preflight를 완료했고 모든 하드웨어를 함께 시작하려는 경우에만 사용한다.

```bash
ros2 launch robot_arm_bringup fss_robot_arm.launch.py \
  device_name:=/dev/serial/by-id/YOUR_U2D2_DEVICE \
  start_fss:=true \
  arm_only:=true
```

주요 launch argument:

| Argument | 기본값 | 설명 |
|---|---|---|
| `device_name` | `/dev/ttyUSB0` | U2D2 장치 경로. 통합 시 by-id 경로 권장 |
| `start_fss` | `false` | `true`일 때 FSS 전체 launch도 함께 시작 |
| `arm_only` | `true` | `true`: ID 1~3, `false`: ID 1~4 |
| `use_sim` | `false` | FSS SIL 사용 여부 |
| `cpu_pin` | `false` | FSS 실시간 노드 CPU pinning |
| `use_realsense` | `false` | FSS RealSense 실행 여부 |
| `require_healthy` | `true` | FSS health 정상 조건 적용 |

`use_sim:=true`는 FSS만 SIL로 실행한다. 현재 RobotArm에는 별도의 simulated hardware가 없으므로 RobotArm 쪽은 여전히 실제 U2D2와 Dynamixel을 요구한다.

## 8. 상태 확인

```bash
ros2 control list_controllers \
  --controller-manager /robot_arm/controller_manager

ros2 topic echo /robot_arm/joint_states
ros2 topic echo /robot_arm/fss_supervisor_status
ros2 topic echo /system/mode
ros2 topic echo /system/health
```

FSS가 아직 `ACTIVE`가 아니면 supervisor 상태는 다음과 비슷하다.

```text
data: "TORQUE_OFF: FSS mode is not ACTIVE"
```

정상 허용 상태:

```text
data: "COMMANDS_ENABLED"
```

## 9. FSS 모드 전환

FSS는 `IDLE → PRESSURIZING → READY → ACTIVE` 순서로 전환해야 한다. 각 단계에서 센서, PLC, 공압 및 비상정지 수단을 먼저 확인한다.

```bash
ros2 service call /system/set_mode \
  fss_interfaces/srv/SetSystemMode "{requested_mode: 1}"

ros2 service call /system/set_mode \
  fss_interfaces/srv/SetSystemMode "{requested_mode: 2}"

ros2 service call /system/set_mode \
  fss_interfaces/srv/SetSystemMode "{requested_mode: 3}"
```

`ACTIVE` 전환만을 위해 앞 단계를 생략하면 FSS manager가 요청을 거부한다.

## 10. 통합 모드에서 로봇팔 명령

### Degree 명령

```bash
ros2 topic pub --once /robot_arm/joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3], positions_deg: [5.0, 10.0, -5.0], duration_sec: 5.0}"
```

### 그리퍼 거리 명령

그리퍼가 포함된 `arm_only:=false` 구성에서는 닫힘을 `0 cm`, 최대 열림을 `13 cm`로
명령한다.

```bash
ros2 topic pub --once /robot_arm/gripper_opening_cm \
  robot_arm_controller/msg/GripperCommandCm \
  "{opening_cm: 5.0, duration_sec: 3.0}"
```

이 명령의 처리 흐름:

```text
/robot_arm/gripper_opening_cm
  → gripper_opening_bridge
  → /robot_arm/requested_joint_trajectory
  → fss_arm_supervisor_node
  → /robot_arm/arm_trajectory_controller/joint_trajectory
```

### 표준 radian 궤적

FSS 또는 별도 임무 노드가 표준 궤적을 만들 경우 supervisor 입력으로 보낸다.

```bash
ros2 topic pub --once /robot_arm/requested_joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3], points: [{positions: [0.1, 0.2, -0.1], time_from_start: {sec: 5}}]}"
```

Supervisor는 다음 항목을 controller 전달 전에 검사한다.

- 미등록 또는 중복 관절
- point 배열 크기와 NaN/Inf
- 증가하지 않는 `time_from_start`
- 설정된 최대 point 수와 전체 duration
- 보정 파일의 최대 속도
- 보정 파일의 관절 위치 제한

유한한 위치 제한 초과는 메시지 전체를 버리지 않고 해당 관절만 포화한다.
따라서 한 관절이 상한에 도달해도 다른 관절 목표는 계속 전달된다.

## 11. 검증 결과

2026-07-31 소프트웨어 검증으로 다음을 확인했다.

- `fss_interfaces` underlay 상태에서 supervisor C++ 빌드 성공
- RobotArm 세 패키지 전체 빌드 성공
- `/robot_arm` namespace와 namespaced controller parameter 로딩 성공
- `ACTIVE` + 정상 health에서 요청 궤적이 controller 토픽으로 전달됨
- joint position `[0.1, 0.2, 0.3] rad` 수신 후 차단 상태 전환 시 동일 위치의 hold 궤적 발행
- mode 미수신 또는 timeout 상태에서 새 요청 궤적 차단
- FSS lifecycle API를 사용하는 supervisor 포함 전체 C++ 빌드
- calibration에서 생성한 Xacro/URDF 파싱
- 제한 초과 관절만 포화되고 다른 관절 명령은 보존되는 회귀 테스트
- calibration 및 trajectory validator 전체 테스트 통과

```text
Summary: 10 tests, 0 errors, 0 failures, 0 skipped
```

실제 FSS와 Dynamixel을 동시에 연결한 하드웨어 통합 시험은 별도로 수행해야 한다.

## 12. 현재 제한사항

### Software torque-off와 물리 비상정지

Supervisor는 `EMERGENCY_STOP`과 mode timeout에서 controller-manager lifecycle을
통해 hardware를 inactive로 전환하고 Dynamixel torque를 해제한다. 또한 모터에는
500 ms Bus Watchdog을 설정한다.

이는 소프트웨어와 통신이 살아 있는 범위의 보호 기능이다. 전원 차단이 필요한
진짜 비상정지는 별도 물리 회로가 우선해야 한다. Torque가 풀렸을 때 링크가
중력으로 낙하할 수 있는 구조라면 기계식 브레이크나 지지 구조도 함께 검증해야 한다.

### FSS manager health 집계

FSS의 기존 `/system/health`는 navigation과 valve/PLC 상태를 집계하며 RobotArm 상태는 포함하지 않는다. 이 문서의 구조는 RobotArm supervisor가 FSS 상태를 받아 자체적으로 안전을 관리한다.

RobotArm 오류를 FSS manager의 공식 health와 `SAFE_HOLD` 전이에 포함하려면 향후 FSS interface 또는 manager 수정이 필요하다.

### 내부 토픽 우회

내부 controller 토픽에 직접 publish하면 supervisor를 우회할 수 있다. 운용 노드와 사용자 명령은 반드시 다음 중 하나를 사용한다.

```text
/robot_arm/joint_commands_deg
/robot_arm/gripper_opening_cm
/robot_arm/requested_joint_trajectory
```

### FSS 실행 파일 권한

임시 통합 검증 중 FSS를 `--symlink-install`로 새로 빌드했을 때 `fss_gnc/gnc_node.py`에 실행 권한이 없어 `fss_full.launch.py`가 해당 실행 파일을 찾지 못하는 경우를 확인했다. 이는 RobotArm 연동 코드가 아니라 FSS 패키징 또는 파일 권한 문제다.

동일한 오류가 발생하면 FSS 저장소 담당자와 실행 권한 및 `install(PROGRAMS ...)` 구성을 확인한다. FSS 소스를 임의로 수정하지 않고 담당 저장소에서 해결한 뒤 통합 시험을 재개한다.

## 13. 향후 작업

- RobotArm 상태를 FSS manager health에 포함하는 interface 협의
- 실제 임무용 로봇팔 command/action 계약 정의
- stable U2D2 udev rule 배포
- Jetson에서 FSS 제어 주기와 RobotArm 50Hz loop의 CPU 부하 측정
- rosbag 기반 FSS mode 전환 및 arm hold 회귀 테스트
- SROS2/DDS policy로 내부 controller topic publish 권한 제한
- 실제 하드웨어에서 E-stop, Bus Watchdog, 고온·저전압·통신 단절 시험
