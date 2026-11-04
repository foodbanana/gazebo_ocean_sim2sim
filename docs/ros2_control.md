# ros2_control: WAM-V 위 UR5 + RH-P12-RN 제어

작성: 2026-11-02 · Gazebo Harmonic, ROS 2 Jazzy, gz_ros2_control 1.2.19

WAM-V 상판에 붙인 UR5 6축과 RH-P12-RN 그리퍼를 Gazebo 안에서 ros2_control 로 움직인다.

> **갱신 (인터페이스 단계)**: 정책·텔레옵이 쓰는 토픽 (`/joint_command`, `/gripper/command` raw 0~1150, 33 ms 보간,
> 그리퍼 등속 이동, 팔 6 개 `/joint_states`) 은 [`interface.md`](interface.md) 의 인터페이스 노드가 맞춘다.
> 그에 맞춰 컨트롤러를 forward_command_controller → **joint_trajectory_controller** 로 바꾸고, 물리를 **240 Hz** 로 바꿨다.
> 이 문서의 1·2·3·6 장은 지금 상태, 4·5 장의 게인 측정은 당시 (250 Hz, forward 컨트롤러, 보간 없음) 기록이다.

---

## 1. 실행

> launch 인자 전체 (drone_mode, wave_gain, prop_spin 등) 는 README 의 Run 표 참고. 아래는 ros2_control 관련만.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `world` | `empty_ocean` | Gazebo 월드 |
| `xacro_file` | `ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro` | 보트 + 팔 |
| `xacro_args` | (없음) | xacro 에 넘길 인자. 예: `xacro_args:="kinematics_params:=/path/calib.yaml"` |
| `headless` / `paused` | `False` | GUI 없이 / 일시정지로 시작 |

### 손으로 움직여 보기

명령은 인터페이스 토픽으로 보낸다 ([`interface.md`](interface.md) 1 장).

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    position: [0.3, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 1150.0}"   # 닫기 (0 = 열기)

ros2 control list_controllers
ros2 topic echo /joint_states                              # 팔 6 개, 120 Hz
ros2 topic echo /joint_state_broadcaster/joint_states      # 손가락 포함 10 개, 240 Hz
```

---

## 2. 구조

```text
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
 ├─ xacro wamv_with_arm.urdf.xacro  → URDF 1개 (/tmp/ocean_sim2sim_*.urdf, launch 가 끝나면 지움)
 ├─ ocean_sim2sim_gazebo/empty_ocean.launch.py  (urdf:= 위 파일)
 │    ├─ gz sim empty_ocean.sdf
 │    ├─ VRX: URDF → SDF → 원점에 스폰, /wamv/* · /clock 브리지
 │    └─ (모델 안) gz_ros2_control 플러그인 → controller_manager (240 Hz)
 ├─ robot_state_publisher  (robot_description, gz_ros2_control 이 여기서 URDF 를 읽음)
 ├─ spawner: joint_state_broadcaster, arm_state_broadcaster, arm_controller, gripper_controller
 └─ ocean_sim2sim_interface  (interface.md)
```

### 파일

| 패키지 | 파일 | 내용 |
|---|---|---|
| `ur5_rh_p12_description` (팔) | `urdf/ur5_rh_p12.ros2_control.xacro` | `<ros2_control>` 태그. 팔 6축 + `rh_r1_joint` = position 명령, mimic 3개 = 상태만. 시작값 = Isaac Sim 홈 자세 |
| | `config/ros2_controllers.yaml` | 컨트롤러 + `gz_ros_control` 게인 |
| `ocean_sim2sim_description` (보트) | `urdf/wamv_with_arm.urdf.xacro` | 팔의 control xacro 를 include 하고 `gz_ros2_control` 플러그인을 붙임 |
| `ocean_sim2sim_interface` | `config/ur5_rh_p12_interface.yaml` | 인터페이스 노드 설정 (interface.md) |
| `ocean_sim2sim_bringup` | `launch/wamv_arm.launch.py` | 위 구조를 한 번에 실행 |

### 팔 교체

팔에 딸린 것(관절 이름, 컨트롤러)은 팔 패키지 안에 있다. 보트 xacro 는 두 인자로 받기만 한다.

| `wamv_with_arm.urdf.xacro` 인자 | 기본값 |
|---|---|
| `arm_ros2_control_xacro` | `ur5_rh_p12_description/urdf/ur5_rh_p12.ros2_control.xacro` (빈 문자열 = 제어 없음) |
| `arm_controllers_yaml` | `ur5_rh_p12_description/config/ros2_controllers.yaml` |

다른 팔은 이 두 파일을 그 팔 패키지에 만들고 `arm_xacro`·`arm_root_link` 와 함께 넘긴다.
launch 의 spawner 가 부르는 컨트롤러 이름(`joint_state_broadcaster`, `arm_state_broadcaster`, `arm_controller`,
`gripper_controller`)은 yaml 과 맞추고, 인터페이스 설정은 `interface_config:=` 로 넘긴다.

---

## 3. 컨트롤러

| 이름 | 종류 | 관절 | 토픽 |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | 10 개 (팔 6 + 손가락 4), 240 Hz | → `/joint_state_broadcaster/joint_states` |
| `arm_state_broadcaster` | JointStateBroadcaster | 팔 6, position·velocity·effort, 120 Hz | → `/joint_states` |
| `arm_controller` | joint_trajectory_controller (position) | 팔 6 | `/arm_controller/joint_trajectory` (인터페이스 노드가 보냄) |
| `gripper_controller` | joint_trajectory_controller (position) | `rh_r1_joint` | `/gripper_controller/joint_trajectory` (인터페이스 노드가 보냄) |

- `controller_manager` `update_rate` 240 Hz = 물리 스텝 1/240 s
- `arm_controller`: `interpolate_from_desired_state: true` (새 궤적을 마지막 목표에서 시작 = Isaac Sim ArmBridge)
- 시작 자세: `initial_value` = Isaac Sim `home_pose` `[0, -1.5708, 1.5708, -1.5708, -1.5708, 0]`, 그리퍼 0 (열림).
  컨트롤러가 뜨기 전에도 이 자세에서 시작한다 (팔이 처지지 않음)

### 위치 명령이 Gazebo 에서 처리되는 방식

`gz_ros2_control` 은 position 명령을 **속도 명령으로 바꿔** 물리 엔진에 준다.

```text
관절 속도 명령 = position_proportional_gain × (목표 − 현재) × update_rate
```

= 1 차 지연 (시상수 = 1 / (gain × update_rate): 지금 240 Hz 에서 27.8 ms, 측정 당시 250 Hz 에서 26.7 ms). DART 는 이 속도를 관절 effort 한계 (UR5 150 / 28 Nm) 안에서 강하게 지킨다.
Isaac Sim 은 PD 토크 drive (stiffness·damping) 라 방식이 다르다 → 아래 시험으로 응답을 맞췄다.

---

## 4. 게인 결정: `position_proportional_gain = 0.15`

Isaac Sim drive 튜닝(`data_collection_ur5_gripper/docs/drive_tuning.md` 8장)과 **같은 시험**을 Gazebo 에서 돌려 비교했다.
파도 켬 (`empty_ocean`, gain 0.3), 물리 4 ms, Bullet 충돌 검출기.

- **시험 A**: 관절 하나씩 0.05 rad 계단 (50 Hz 텔레옵 한 스텝 최대 변화 ≈ 0.063 rad)
- **시험 C**: 50 Hz 계단 명령으로 만든 0.5 Hz 사인파, 진폭 0.3 rad. 지연 = 명령과 응답의 오차 제곱합이 최소가 되는 시간 이동

| 게인 | C 지연 | C RMS | A 정착 (2 %) | A 오버슈트 |
|---|---|---|---|---|
| 0.1 (기본) | 38 ms | 1.55° | 0.15 ~ 0.16 s | 0 % |
| **0.15 (채택)** | **26 ~ 30 ms** | **1.04 ~ 1.12°** | **0.10 ~ 0.11 s** | **0 %** |
| 0.2 | 22 ms | 0.82 ~ 0.88° | | |
| *Isaac Sim* | *19 ~ 34 ms* | *0.7 ~ 1.3°* | *0.02 ~ 0.08 s* | *최대 1.2 %* |
| *기준* | *< 40 ms* | | *< 0.5 s* | *< 2 %* |

0.15 = Isaac Sim 결과 범위의 가운데. 0.2 보다 덜 공격적.

게인과 오버슈트를 왜 이렇게 다뤘는지, 직접 재 보는 방법은 [`gain_tuning.md`](gain_tuning.md) (`drive_test.py`).

---

## 5. 검증 결과 (게인 0.15)

| 항목 | 결과 |
|---|---|
| 컨트롤러 | 3 개 모두 `active`, 시작 후 약 4 s |
| 홈 자세 유지 (파도 속 10 s) | 최대 오차 0.000° |
| 시험 A (0.05 rad) | 오버슈트 0 %, 정착 0.10 ~ 0.11 s, 정상상태 오차 0, 다른 관절 흔들림 0.000° |
| 시험 C | 지연 26 ~ 30 ms, RMS 1.04 ~ 1.12° |
| 그리퍼 mimic | 손가락 4 개가 함께 움직임. 움직이는 동안 최대 2.3° (한 스텝 늦음), 멈춘 뒤 0 |
| RTF | 1.00 |

### 참고: 큰 계단 (0.2 rad, 모든 관절 동시)

`shoulder_lift_joint` 가 최대 관절 속도 (π rad/s) 로 목표를 지나 0.56 rad 까지 갔다가 돌아옴 (오버슈트 약 155 %).
+0.2 rad 가 중력 방향이라 감속이 늦다. Isaac Sim 도 같은 0.2 rad 계단에서 13.6 % (토크 포화) 였고, 정책·텔레옵은
60 Hz 명령 + 33 ms 보간이라 이런 계단이 들어오지 않는다. 그래서 판정은 Isaac Sim 과 같이 시험 A·C 로 했다.

---

## 6. Isaac Sim 과 남은 차이

토픽·보간·그리퍼 속도·`/joint_states` 는 인터페이스 단계에서 맞췄다 ([`interface.md`](interface.md) 6 장).

| 항목 | Isaac Sim | Gazebo 지금 | 다음 |
|---|---|---|---|
| 그리퍼 힘 | max_force 2.28 Nm (실물 400 mA) | URDF effort 1000 (원본 값) | 드론 파지 단계 |
| 중력 처짐 | 홈 자세 0.033° (PD 라 약간 처짐) | 0.000° (속도 명령이 단단히 버팀) | 파지 단계에서 영향 확인 |

---

## 7. 알아 둘 것 (해 보고 찾음)

- **DART 는 mimic 조인트를 지원하지 않는다**: Gazebo 로그 `Attempting to create a mimic constraint ... does not support mimic constraints` 는 그대로 나온다.
  대신 `gz_ros2_control` 이 URDF `<mimic>` 을 읽어 손가락 3 개를 `rh_r1_joint` 에 맞춰 움직인다 (`Joint 'rh_r2' is mimicking joint 'rh_r1_joint'`).
  mimic 관절은 `<ros2_control>` 태그에 **상태 인터페이스만** 적어야 한다 (명령 인터페이스가 있으면 에러).
- **VRX 는 `JointPositionController` 플러그인을 전부 스러스터로 본다** (`vrx_gz/src/vrx_gz/model.py` `payload_from_sdf`): `<name>` 이 없으면 launch 가 죽는다.
  팔 제어에 그 플러그인을 쓰지 않고 `gz_ros2_control` 을 쓰는 이유 중 하나.
- `gz_ros2_control` 노드 이름은 `gz_ros_control`. 게인은 yaml 의 `gz_ros_control: ros__parameters: position_proportional_gain` 에 둔다
  (로그 `The position_proportional_gain has been set to: 0.15` 로 확인).
