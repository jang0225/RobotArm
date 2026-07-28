# ROS 2 Dynamixel Robot Arm 개발 기록

> 포트폴리오 작성과 기술 회고를 위한 문서
> 최종 갱신: 2026-07-28

## 1. 프로젝트 개요

ROS 2 Humble과 C++를 사용해 Dynamixel 기반 로봇팔을 제어하는 프로젝트다. 처음에는 관절 모터 2개와 그리퍼 모터 1개로 시작했지만, 기구 구성을 다시 확인하면서 관절 모터 3개와 그리퍼 모터 1개를 제어하는 구조로 확장했다.

주요 목표는 다음과 같다.

- Dynamixel 모터 4개를 하나의 ROS 2 하드웨어 시스템으로 관리
- 사용자가 degree 단위로 직관적으로 관절을 명령
- 각 모터의 중앙 위치를 사용자 기준 `0°`로 변환
- 실제 기구에서 측정한 관절 범위와 안전 여유를 소프트웨어에 반영
- 여러 관절의 시간 기반 궤적 제어 지원
- 한 관절의 제한 문제가 다른 관절의 동작을 중단시키지 않도록 결함 격리
- 향후 FSS 또는 MoveIt 2와 연동할 수 있는 표준 `ros2_control` 인터페이스 제공

## 2. 시스템 구성

### 하드웨어

| ID | 모델 | 역할 | Operating Mode | Drive Mode |
|---:|---|---|---:|---:|
| 1 | XD430-T350 | `joint1` | 3, Position Control | 0 |
| 2 | XD430-T350 | `joint2` | 3, Position Control | 0 |
| 3 | XM430-W350 | `joint3` | 3, Position Control | 0 |
| 4 | XM430-W350 | `gripper_joint` | 3, Position Control | 4 |

- Protocol: Dynamixel Protocol 2.0
- Baud rate: 1 Mbps
- Encoder resolution: 4096 tick/revolution
- 통신 장치: U2D2 계열 USB 시리얼 장치

### 소프트웨어

```text
src/
├── robot_arm_hardware/    # Dynamixel SDK 기반 ros2_control SystemInterface
├── robot_arm_bringup/     # Xacro/URDF, controller 설정, launch
└── robot_arm_controller/  # degree 명령 변환 및 궤적 생성 노드
```

제어 흐름은 다음과 같다.

```text
degree 명령 또는 궤적 생성 노드
        ↓
JointTrajectoryController
        ↓
robot_arm_hardware SystemInterface
        ↓
Dynamixel GroupSyncWrite / GroupSyncRead
        ↓
ID 1, 2, 3, 4
```

노드 구현과 `main()`을 분리 가능한 구조로 검토했지만, 현재 실행 파일 규모에서는 클래스와 `main()`을 같은 소스에 두고 패키지 역할을 명확히 분리했다. 하드웨어 접근은 별도의 `ros2_control` 플러그인으로 분리했기 때문에 향후 상위 제어기와 연결하기 쉽다.

## 3. 관절 보정

### Tick, degree, radian 변환

Dynamixel의 position 값은 tick 단위다. 1회전이 4096 tick이므로 현재 모델에서는 다음 값을 사용한다.

```text
1 tick = 360 / 4096 degree
       = 약 0.087890625 degree
       = 약 0.001533981 radian
```

로봇팔을 조작할 때 모터의 절대각보다 관절 중앙을 `0°`로 사용하는 편이 직관적이므로 다음 변환을 적용했다.

```text
관절각(rad) = direction × (현재 tick - 중앙 tick) × radians_per_tick

목표 tick = 중앙 tick
          + 목표 관절각(rad) / (direction × radians_per_tick)
```

### 측정 결과와 소프트웨어 제한

실제 기구의 가동 범위를 Dynamixel Wizard로 측정한 후 양 끝에서 2°씩 안전 여유를 적용했다.

| 관절 | 측정된 절대각 범위 | 적용 절대각 범위 | 중앙 tick | 사용자 명령 범위 |
|---|---|---|---:|---|
| `joint1` | 78.84° ~ 279.84° | 80.84° ~ 277.84° | 2040 | -98.456875° ~ 98.543125° |
| `joint2` | 109.25° ~ 339.52° | 111.25° ~ 337.52° | 2553 | -113.134766° ~ 113.135234° |
| `joint3` | 103.45° ~ 313.95° | 105.45° ~ 311.95° | 2375 | -103.290234° ~ 103.209766° |
| `gripper_joint` | 미측정 | 임시 범위 | 2305 | -5° ~ 5° |

`joint3`은 링크를 분리했다가 다시 조립하면서 가동 범위가 바뀌었다. 이전 보정값을 그대로 사용하지 않고 103.45°~313.95°를 다시 측정하여 중앙 tick과 제한을 재계산했다. 이를 통해 소프트웨어 보정값이 기구 조립 상태에 종속된다는 점을 확인했다.

## 4. 구현 과정

### 4.1 Dynamixel SDK 직접 제어에서 ros2_control 구조로 확장

초기에는 하나의 C++ 노드가 포트를 열고 모터를 직접 제어하는 형태를 검토했다. 이후 상위 제어 시스템과 궤적 제어기를 연결하기 위해 하드웨어 접근을 `hardware_interface::SystemInterface` 플러그인으로 분리했다.

하드웨어 플러그인의 주요 역할은 다음과 같다.

- 포트 열기와 baud rate 설정
- ID별 ping 및 모델 번호 검증
- Operating Mode와 Drive Mode 검증
- torque 활성화와 비활성화
- `GroupSyncRead`를 이용한 position/velocity 상태 읽기
- `GroupSyncWrite`를 이용한 position 목표 전송
- ROS radian과 Dynamixel tick 상호 변환

이 구조를 사용하면 상위 노드는 Dynamixel SDK 세부 구현을 알 필요 없이 표준 ROS 2 controller 인터페이스만 사용하면 된다.

### 4.2 Degree 명령 인터페이스

ROS 표준 관절 단위는 radian이지만 수동 시험과 보정 단계에서는 degree가 편리하다. 이를 위해 `JointCommandDegrees` 메시지와 `degree_trajectory_bridge` 노드를 추가했다.

예시:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3], positions_deg: [10.0, 20.0, -15.0], duration_sec: 5.0}"
```

변환 노드는 다음 작업을 수행한다.

- 관절 이름 검증
- degree 값의 유효성 검사
- 관절별 최소·최대 범위 적용
- degree를 radian으로 변환
- `JointTrajectory` 메시지 발행
- 일부 관절만 명령하는 partial joint goal 지원

### 4.3 물고기 형태의 다관절 궤적

단일 목표 위치뿐 아니라 여러 관절이 위상차를 두고 반복적으로 움직이는 `fish_motion_node`를 추가했다.

궤적 특징:

- 세 관절에 서로 다른 진폭 적용
- 관절 사이에 phase lag 적용
- 주기와 반복 횟수를 ROS parameter로 변경
- 시작 시 중앙으로 천천히 이동
- 시작과 끝에서 envelope를 0으로 만들어 갑작스러운 충격 감소
- 그리퍼 포함 여부 선택
- 완료 후 중앙 위치로 복귀

이 구현을 통해 개별 위치 명령에서 시간 기반 다관절 궤적 제어로 기능을 확장했다.

### 4.4 Arm-only와 전체 모터 실행 분리

그리퍼가 완성되지 않은 상태에서 ID 4까지 torque가 켜지는 것을 방지하기 위해 두 실행 구성을 제공했다.

- `arm_only.launch.py`: ID 1, 2, 3만 활성화
- `ros2_control.launch.py`: ID 1, 2, 3, 4 전체 활성화

이를 통해 미완성 하드웨어가 있어도 완성된 관절만 독립적으로 시험할 수 있게 했다.

## 5. 주요 트러블슈팅

### 5.1 `dynamixel_sdk/dynamixel_sdk.h`에 빨간 밑줄 표시

#### 증상

코드는 빌드되거나 의존성이 설치되어 있는데 VS Code IntelliSense에서 Dynamixel SDK 헤더를 찾지 못했다. 또한 `compile_commands.json`에서 특정 소스 파일을 찾지 못한다는 메시지가 나타났다.

#### 원인

- ROS 환경을 source하지 않은 터미널에서 VS Code 실행
- 패키지를 빌드하기 전에 생성된 오래된 `compile_commands.json`
- 현재 소스 경로와 빌드 데이터의 경로 불일치
- CMake 의존성과 편집기 `includePath`는 서로 별개의 설정

#### 대응

- `dynamixel_sdk`를 package 및 CMake 의존성으로 명시
- workspace를 다시 빌드해 compile database 갱신
- 빌드 후 `install/setup.bash` source
- 편집기 오류와 실제 컴파일 오류를 구분해 확인

### 5.2 `/dev/ttyUSB*`가 표시되지 않음

#### 증상

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

명령에 출력이 없고 `ros2_control_node`가 시작 중 종료됐다.

#### 원인

모터 또는 USB 통신 장치를 연결하지 않은 상태에서는 장치 파일이 생성되지 않는다. 또한 장치가 있어도 사용자가 `dialout` 그룹에 없거나 Dynamixel Wizard가 포트를 점유하면 ROS 노드가 포트를 열 수 없다.

#### 대응

```bash
sudo usermod -aG dialout "$USER"
```

그룹 추가 후 다시 로그인하고, 실행 전에 실제 장치 이름을 확인하며 Dynamixel Wizard를 종료하도록 했다. 포트가 `/dev/ttyUSB0`과 `/dev/ttyUSB1` 사이에서 달라질 수 있으므로 launch argument로 지정할 수 있게 구성했다.

### 5.3 Fish motion subscriber를 찾지 못함

#### 증상

```text
Trajectory controller subscriber was not found
```

#### 원인

`fish_motion_node`만 실행하고 `arm_trajectory_controller`가 활성화된 bringup launch를 먼저 실행하지 않았다.

#### 대응

하드웨어 launch를 먼저 실행하고 controller가 `active` 상태인지 확인한 후 궤적 노드를 실행하도록 실행 순서를 정리했다. 궤적 노드에도 subscriber 대기와 오류 메시지를 추가했다.

### 5.4 ID 1은 움직이지만 ID 2와 ID 3은 움직이지 않음

#### 증상

ID 1에 최대각 명령을 보낸 뒤 ID 2와 ID 3에 명령을 보내도 움직이지 않았다. 로그에는 다음 메시지가 반복됐다.

```text
State tolerances failed for joint 1
Position Error: 약 0.203, Position Tolerance: 0.200
Holding position due to state tolerance violation
```

#### 원인 분석

문제는 모터 1의 명령이 끝나지 않아서가 아니라 두 가지 동작이 겹쳐 발생했다.

1. degree에서 radian으로 변환할 때 부동소수점 반올림 차이로 정확한 최대값이 소프트웨어 제한을 극미세하게 초과할 수 있었다.
2. 기존 하드웨어 `write()`는 관절 하나라도 범위를 벗어나면 즉시 `ERROR`를 반환하여 전체 `GroupSyncWrite`를 취소했다.

따라서 ID 1의 명령 오류가 ID 2와 ID 3의 정상 명령까지 막았다. 모터가 목표를 따라가지 못하면서 관절 오차가 0.2 rad를 초과했고, 단일 `JointTrajectoryController`가 전체 궤적을 hold 상태로 전환했다.

#### 1차 대응

정확한 끝점에서 발생하는 미세한 변환 오차를 허용하기 위해 1 encoder tick만큼의 허용오차를 둔 뒤 범위 안으로 clamp했다.

#### 최종 대응: 관절별 포화 처리와 결함 격리

끝점 오차뿐 아니라 모든 상·하한 상황을 일관되게 처리하도록 정책 자체를 변경했다.

- 유한한 범위 초과 명령은 해당 관절의 최소·최대값으로 포화
- 한 관절의 범위 초과 때문에 `write()` 전체를 실패시키지 않음
- NaN/Inf 명령은 해당 관절만 현재 측정 위치에 유지
- 변경된 모터만 sync-write packet에 포함
- 한 모터의 packet 준비 실패 시 다른 모터의 준비를 계속 수행
- 제한 진입과 정상 범위 복귀를 관절별로 기록
- controller의 path tolerance를 `0.0`으로 설정해 한 관절의 추종 오차가 전체 궤적을 hold시키지 않도록 변경
- degree 변환 노드에서도 같은 포화 정책 적용

수정 후의 동작 예시는 다음과 같다.

```text
입력: joint1=150°, joint2=20°, joint3=-20°

처리:
joint1 → 허용 상한 98.543125°로 포화
joint2 → 20° 그대로 실행
joint3 → -20° 그대로 실행
```

이 변경으로 관절 제한은 여전히 지키면서 한 관절의 상한 도달이 다른 관절의 정상 동작을 막지 않게 했다.

## 6. 안전 설계

현재 적용된 안전 기능은 다음과 같다.

- 시작 시 모터 모델, ID, mode 검증
- 측정된 기구 한계보다 안쪽에 2° 소프트웨어 여유 적용
- degree 변환 노드와 하드웨어 플러그인의 이중 범위 제한
- 포트 설정과 초기 상태 읽기가 성공한 뒤 torque 활성화
- 활성화 시 현재 위치를 최초 명령으로 사용해 갑작스러운 이동 방지
- 종료 시 torque 비활성화
- 미완성 그리퍼는 임시 `±5°` 제한
- arm-only 모드로 그리퍼 torque 제외 가능
- 궤적 시작과 종료 구간에 부드러운 envelope 적용

`trajectory` path tolerance는 관절 간 결함 격리를 위해 비활성화했다. 따라서 실제 운용 단계에서는 `/joint_states` 기반의 관절별 watchdog, 전류·온도 감시 및 비상정지 로직을 별도로 추가해야 한다.

## 7. 검증 기록

### 정적 및 빌드 검증

2026-07-28 최종 수정 후 다음 명령으로 세 패키지의 전체 빌드를 확인했다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select robot_arm_hardware robot_arm_controller robot_arm_bringup
```

결과:

```text
Summary: 3 packages finished
```

추가로 `git diff --check`를 실행해 whitespace 오류가 없음을 확인했다.

### 하드웨어 검증 절차

1. Dynamixel Wizard 종료
2. USB 장치 이름 확인
3. `arm_only.launch.py` 실행
4. controller 상태가 `active`인지 확인
5. 작은 각도와 긴 duration으로 세 관절 시험
6. 범위 초과 명령을 포함한 다관절 메시지로 관절별 포화 처리 확인
7. `/joint_states`에서 나머지 관절이 목표를 계속 추종하는지 확인

시험 명령:

```bash
ros2 topic pub --once /joint_commands_deg \
  robot_arm_controller/msg/JointCommandDegrees \
  "{joint_names: [joint1, joint2, joint3], positions_deg: [150.0, 20.0, -20.0], duration_sec: 15.0}"
```

예상 결과:

- `joint1`: 상한으로 자동 제한되고 saturation 경고가 한 번 출력
- `joint2`: 20° 목표 계속 수행
- `joint3`: -20° 목표 계속 수행
- `Holding position due to state tolerance violation`으로 전체 궤적이 중단되지 않음

## 8. Git 변경 이력

주요 커밋:

| Commit | 내용 |
|---|---|
| `ba40097` | Initial commit |
| `5cc042c` | 초기 기능 변경 |
| `89fda6a` | Fish motion 수정 |
| `88919bb` | Add third arm joint and gripper motor |

2026-07-28 작업 내용에는 `joint3` 재보정과 관절별 포화·결함 격리 로직이 포함되어 있다. 커밋 전에는 `git diff`로 변경 범위를 확인하고 하드웨어 시험 결과를 이 문서에 추가한다.

## 9. 포트폴리오용 핵심 요약

### 한 줄 소개

ROS 2 `ros2_control`과 Dynamixel SDK를 이용해 3축 관절 및 그리퍼를 degree 단위와 시간 기반 궤적으로 제어하는 C++ 로봇팔 시스템을 개발했다.

### 담당 및 구현

- Dynamixel Protocol 2.0 기반 4모터 통신 구현
- C++ `ros2_control SystemInterface` 하드웨어 플러그인 설계
- URDF/Xacro 기반 관절 모델과 실측 제한 적용
- 중앙을 `0°`로 사용하는 degree-radian-tick 변환 계층 구현
- partial joint command와 다관절 trajectory 제어 구현
- 위상차 기반 물고기 형태 반복 궤적 생성
- 모터별 모델 및 제어 모드 시작 검증
- 관절별 command saturation을 통한 결함 격리

### 대표 문제 해결 사례

한 관절의 최대각 명령에서 발생한 부동소수점 경계 오차가 전체 sync-write를 취소하고 모든 관절을 정지시키는 문제를 로그와 데이터 흐름으로 추적했다. 단순히 허용오차를 늘리는 데 그치지 않고 관절별 포화, 비정상 값의 개별 hold, 변경 관절만 전송하는 방식으로 쓰기 정책을 재설계했다. 그 결과 한 관절의 제한 도달이 다른 관절의 제어를 방해하지 않도록 개선했다.

### 사용 기술

- C++17
- ROS 2 Humble
- ros2_control
- joint_trajectory_controller
- Dynamixel SDK
- Xacro / URDF
- Custom ROS 2 Message
- CMake / colcon
- Git / GitHub

## 10. 향후 개선 계획

- ID 4 그리퍼 조립 후 실제 중앙 tick과 가동 범위 재측정
- 실제 링크 질량, 관성 및 collision geometry 반영
- Dynamixel Hardware Error Status, 전류, 전압, 온도 모니터링
- 모터별 통신 및 추종 watchdog 추가
- 비상정지와 torque-off service 구현
- 다회전 및 방향 반전 관절을 위한 calibration 파일 분리
- 동일한 제한값이 Xacro와 controller 코드에 중복되지 않도록 단일 YAML 설정으로 통합
- MoveIt 2 연동과 충돌 없는 경로 계획
- FSS 입력을 `JointTrajectory` 또는 action interface로 연결
- 실제 하드웨어 반복 시험 결과와 오차 그래프 추가

## 11. 기록 갱신 양식

새로운 문제나 기능을 추가할 때 다음 형식으로 기록한다.

```text
날짜:
변경 목적:
증상:
원인:
수정 파일:
해결 방법:
검증 명령:
검증 결과:
남은 위험 또는 후속 작업:
```
