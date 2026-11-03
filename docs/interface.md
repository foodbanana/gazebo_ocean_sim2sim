# 인터페이스: Isaac Sim · 실물과 같은 ROS 2 토픽

작성: 2026-11-02 · 패키지 `ocean_sim2sim_interface`

정책·텔레옵·녹화 도구가 **Isaac Sim 이나 실물과 똑같은 토픽**으로 Gazebo 를 쓸 수 있게 한다.
보내는 쪽은 상대가 실물인지, Isaac Sim 인지, Gazebo 인지 모른다.
기준 = Isaac Sim `data_collection_ur5_gripper` 의 `isaacsim/scripts/ros2_iface.py` (`ArmBridge`, `GripperBridge`),
`robot_drive.py` (`GripperProfile`), `docs/sim_ros2_interface.md`.

---

## 1. 실행과 토픽

```bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py     # 인터페이스 노드까지 같이 뜬다
```

| 토픽 | 타입 | 방향 | 주기 (sim) | 내용 |
|---|---|---|---|---|
| `/joint_command` | sensor_msgs/JointState | → sim | 보내는 쪽 (예 60 Hz) | 팔 6 관절 목표 position [rad]. 33 ms 선형 보간 |
| `/gripper/command` | std_msgs/Float64 | → sim | 바뀔 때 | raw 0 (열림) ~ 1150 (닫힘) |
| `/joint_states` | sensor_msgs/JointState | sim → | 120 Hz | 팔 6 관절 (`shoulder_pan_joint` … `wrist_3_joint` 순서), position·velocity·effort [Nm] |
| `/gripper/joint_states` | sensor_msgs/JointState | sim → | 루프 (카메라 프레임, 약 30 Hz) | `name ['rh_p12_rn']`, position = 현재 raw (0~1150) |
| `/gripper/target` | sensor_msgs/JointState | sim → | 위와 같은 stamp | position = 마지막 goal raw |
| `/clock` | rosgraph_msgs/Clock | sim → | 물리 스텝 | sim time (VRX 브리지) |
| `/protective_stop` | std_msgs/Bool | sim → | 루프 (위와 같은 때) | 보호 정지 중이면 true (stamp 없음, Isaac Sim 과 같음). 8 장 |

- 모든 stamp = sim time. `ros2 topic hz --use-sim-time <토픽>` 으로 sim 기준 주기를 본다
- 손목 카메라 `/cam/wrist/color/image_raw`·`camera_info`: [`wrist_camera.md`](wrist_camera.md)
- `/sim/reset`, `/sim/drone_kill`: [`scene.md`](scene.md). `/protective_stop`: 8 장

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    position: [0.3, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]}"
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 1150.0}"
ros2 topic echo /gripper/joint_states
```

---

## 2. 구조

```text
 정책 / 텔레옵                    ocean_sim2sim_interface (rclpy)                 Gazebo 안 controller_manager (240 Hz)
 ─────────────                   ──────────────────────────────                  ─────────────────────────────────
 /joint_command  ───────────────▶ 검사 (이름·개수·유한값·관절 한계)                arm_controller (JTC)
   JointState                     → JointTrajectory 1 점 (33 ms 뒤 도착) ───────▶  마지막 목표에서 33 ms 선형 보간
 /gripper/command ──────────────▶ 검사 (0~1150, 유한값)                           gripper_controller (JTC)
   Float64 raw                    → [실제 위치 @0 s, 목표 @거리/0.516] ─────────▶  등속 이동
                                  ◀──── /joint_state_broadcaster/joint_states (10 관절, 240 Hz)
 /gripper/joint_states ◀───────── 루프마다 (카메라 프레임, 약 30 Hz) present raw, target raw
 /gripper/target
 /joint_states ◀──────────────────────────────────────────────────────────────── arm_state_broadcaster (팔 6, 120 Hz)
```

| 파일 | 내용 |
|---|---|
| `ocean_sim2sim_interface/ocean_sim2sim_interface/interface_node.py` | 노드 |
| `ocean_sim2sim_interface/config/ur5_rh_p12_interface.yaml` | 팔 관절 이름, raw_max 1150, upper 1.1351 rad, 0.516 rad/s, 33 ms, 토픽 이름. **팔을 바꾸면 이 파일만 새로** (`interface_config:=`) |
| `ur5_rh_p12_description/config/ros2_controllers.yaml` | JTC 2 개, 브로드캐스터 2 개 |
| `ocean_sim2sim_bringup/launch/wamv_arm.launch.py` | 노드 실행. **노드가 끝나면 launch 전체 종료** |
| `ocean_sim2sim_gazebo/launch/empty_ocean.launch.py` | launch 가 끝날 때 Gazebo 서버도 끝냄 (7 장) |

관절 한계는 `/robot_description` (robot_state_publisher) 의 URDF 에서 읽는다.

### 루프: 그리퍼 상태와 명령을 처리하는 때 (= Isaac Sim 앱 루프)

Isaac Sim 은 **앱 (렌더) 루프 한 번 = 카메라 프레임 한 장** (30 Hz) 이고, 루프마다
1. `/gripper/joint_states`·`/gripper/target` 을 카메라 이미지와 **같은 stamp** 로 발행하고
2. 그 사이 받은 ROS 명령을 처리한다 → 명령은 최대 33 ms 기다리고, 한 루프 안의 명령은 **마지막 것만** 효과 (60 Hz 로 보내도 30 Hz 로 적용)

Gazebo 도 같게 한다. 루프의 시점 (tick) 은:

| `loop_tick_topic` | tick | 쓰는 때 |
|---|---|---|
| `/cam/wrist/color/camera_info` (기본) | **손목 카메라 프레임마다** (그 프레임 stamp) | 카메라가 있을 때. 이미지와 그리퍼 stamp 가 항상 같다 |
| `""` | `physics_hz / loop_hz` 스텝마다 (240 / 30 = 8) | 카메라가 없는 팔 |

tick T 에서 T 시각의 손가락 위치로 그리퍼 상태를 발행하고 (상태와 tick 이 다른 토픽이라 T 의 상태가 올 때까지 기다림),
`commands_on_loop: true` 면 그 뒤에 보관한 명령을 적용한다. `false` 면 명령은 도착하는 즉시 적용 (Isaac Sim 보다 25~30 ms 빠름).

카메라를 기준으로 하는 이유: Gazebo 가 카메라 프레임 시점을 스스로 정한다 (8 스텝 간격에 가끔 7 스텝, 약 30.3 Hz, [`wrist_camera.md`](wrist_camera.md) 4 장).
8 스텝마다로 따로 세면 이미지와 그리퍼 stamp 가 154 장 중 12 장만 맞았다.

---

## 3. 팔: `/joint_command`

Isaac Sim `ArmBridge` 와 같다.

1. **검사** — 하나라도 틀리면 에러로 중단 (조용히 넘어가지 않음, 3 장 끝):
   name 과 position 개수가 같은지 / 이름이 팔 6 관절과 같은지 (순서는 자유, 중복 없음) / 값이 유한한지 / 관절 한계 안인지
2. **보간** — 새 명령마다 "지금 보간 중인 목표" 에서 새 목표까지 33.3 ms 동안 직선.
   `joint_trajectory_controller` 에 `interpolate_from_desired_state: true` 를 주고 점 하나 (`time_from_start` 33.3 ms) 짜리 궤적을 보낸다.
   위치만 있는 점이라 JTC 가 선형 보간한다. 보간은 컨트롤러 안 (물리 스텝과 같은 주기) 에서 돈다

## 4. 그리퍼: `/gripper/command`

Isaac Sim `GripperBridge` + `GripperProfile` (실물 Dynamixel Profile Velocity 흉내) 와 같다.

1. **검사** — 유한값, 0 ≤ raw ≤ 1150. 아니면 에러로 중단 (실물 노드는 clamp 하지만 Isaac Sim 과 같이 에러)
2. **변환** — rad = raw × 1.13514578304 / 1150
3. **이동** — 목표가 **바뀔 때만** (같은 값 반복은 무시) 실제 손가락 위치에서 출발해 0.516 rad/s 등속:
   궤적 = `[(0 s, 실제 위치), (|목표 − 실제| / 0.516 s, 목표)]`. 물체를 쥐고 있다가 열 때 바로 움직인다
4. **상태** — 루프마다 (3 장): present raw = clamp(rh_r1 × 1150 / 1.1351, 0, 1150), target raw = 마지막 goal

**Gazebo 전용 보정 (`command_margin` 0.001 rad)**: 관절 목표를 한계보다 0.001 rad 아래로 둔다.
관절을 한계값 (1.13514578304) 에 **정확히** 보내면 DART 에서 관절이 한계에 붙어 다시 열리지 않았다
(측정: raw 1149 → 열림, raw 1150 → 닫힌 채로. 충돌 검사를 꺼도 같음). 실물·Isaac Sim 은 빈손으로 닫으면 손가락끼리
약 1.121 rad 에서 닿아 멈추므로 0.06° 차이는 영향이 없다. 그래서 빈손으로 끝까지 닫으면 present raw 가 1149 로 나온다.

---

## 5. 검증 결과

`ros2 run ocean_sim2sim_bringup drive_test.py` (명령은 모두 `/joint_command`·`/gripper/command` 로), 파도 켬, 물리 240 Hz.

| 항목 | Isaac Sim | Gazebo |
|---|---|---|
| `/joint_states` | 팔 6, 120 Hz, pos·vel·eff | 팔 6 (순서 같음), **120.0 Hz**, pos·vel·eff ✅ |
| `/gripper/joint_states`·`/gripper/target` | 30 Hz, 같은 stamp | **30.0 Hz**, 같은 stamp ✅ |
| 홈 자세 유지 10 s | | 0.000° ✅ |
| 0.03 rad 명령 (check_ros2 4 번), 보내는 때를 바꿔 8 번 | 움직이기 시작 33~50 ms, 50 % 50~67 ms, 오버슈트 0 % | **카메라 루프: 8~38 ms, 38~67 ms, 0 %** / 즉시: 8 ms, 38 ms |
| 보간된 목표 | 33 ms 직선 | 매 스텝 같은 폭으로 증가하는 직선, 9 스텝 (37.5 ms) 에 도착 |
| 시험 A (0.05 rad, 보간 포함, 즉시 적용) | | 오버슈트 0 %, 정착 0.121 s, 다른 관절 0.000° |
| 시험 C (50 Hz 사인, 보간 포함) | | 30 Hz 루프: 지연 58~62 ms, RMS 2.2~2.4° / 즉시: 46~50 ms, 1.8° |
| 그리퍼 닫기 / 열기 (시험 D) | 2.15 s / 2.15 s, 반응 0.033 s, 기준 1.98~2.42 s | **루프: 2.20 s / 2.19 s, 반응 0.029~0.033 s** ✅ / 즉시: 2.18 s, 0.017 s |
| 이미지와 그리퍼 stamp | 같음 | **91/91 같음** ✅ |
| 그리퍼 mimic | | 최대 0.2° |
| 잘못된 명령 (관절 한계 밖, 이름 틀림, raw 2000, NaN) | 에러로 중단 | FATAL 로그 → launch 전체 종료, 남는 프로세스 없음 ✅ |

---

## 6. Isaac Sim 과의 차이

| 항목 | Isaac Sim | Gazebo | 영향 |
|---|---|---|---|
| 명령을 받는 때 | 앱 루프 (30 Hz) 마다 한 번 → 최대 33 ms 기다림 | 같음 (카메라 프레임 루프, 3 장). 0.03 rad 명령의 50 % 도달 37.5~66.7 ms (Isaac Sim 50~67 ms) | 실물 UR servoJ (lookahead 0.1 s) 는 둘보다 느리다 |
| 물리 주기 | 120 Hz | 240 Hz | 정책이 보는 주기 (`/joint_states` 120 Hz, 30 Hz 토픽, 명령 30 Hz 적용) 는 같다. 물리 엔진 (PhysX / DART) 이 달라 물리 주기를 맞춰도 동역학이 같아지지 않으므로, 바깥 응답을 같은 시험으로 맞춘다 |
| 보간 도착 | 33.3 ms | 37.5 ms (한 스텝 늦게 도착) | 4 ms |
| `/joint_states` velocity | 관절각 차분 (PhysX 속도값이 실제보다 작아서) | Gazebo 관절 속도 | stage1 은 velocity 를 안 씀 |
| `/joint_states` effort | PhysX 관절 힘 | Gazebo 관절 effort | stage1 은 effort 를 안 씀 |
| `/joint_states` 120 Hz 스텝 | 매 물리 스텝 (120 Hz 물리) | 2 스텝마다 (240 Hz 물리). 오래 돌리면 홀수 스텝으로 한 칸 밀릴 수 있음 (7 장 시간 오차) | stamp 는 항상 실제 물리 스텝 시각 |
| QoS 깊이 | 3 s 분량 (`/joint_states` 360) | `/gripper/*` 90, `/joint_states` 는 브로드캐스터 기본값 | 녹화 시작 직후 누락이 보이면 늘림 |
| 빈손으로 끝까지 닫기 | 손가락끼리 닿아 약 1.121 rad 에서 멈춤 | 1.134 rad 까지 (원본 메시에서 손가락끼리 충돌 감지 안 됨, grasp.md 1-1·5 장) | +0.7° |

---

## 7. 알아 둘 것 (해 보고 찾음)

- **Gazebo 시간은 정수 나노초**: 물리 1/240 s = 4,166,666 ns (끝자리 잘림) 라서 sim time 이 정확한 1/30 s 격자에서 조금씩 벗어난다
  (약 100 s 에 16 µs). 처음에 "sim time 이 1/30 s 배수일 때 발행" 으로 짰다가 100 s 쯤부터 `/gripper/*` 가 끊겼다 →
  **물리 스텝 번호** (round(t × 240) 이 8 의 배수) 로 센다. 카메라가 있으면 루프는 카메라 프레임을 따른다 (3 장)
- **JTC + 관절 한계**: 4 장 `command_margin`
- **launch 가 스스로 끝날 때 Gazebo 서버가 남는 문제**: ros_gz_sim 은 Gazebo 를 `sh -c "ruby gz sim ..."` 으로 띄운다.
  launch 가 스스로 끝나면 (인터페이스 노드 에러 등) 그 래퍼에만 신호를 보내 실제 `gz sim` 서버가 남는다 (터미널 Ctrl+C 는 프로세스 그룹 전체에 가서 괜찮음).
  `empty_ocean.launch.py` 가 종료 때 같은 프로세스 그룹의 `gz sim` 에 SIGINT 를 보낸다
- **처음 보내는 메시지가 사라짐**: 퍼블리셔를 만들자마자 보내면 구독과 연결되기 전이라 버려진다.
  `ros2 topic pub --once -w 1` 처럼 구독자를 기다리거나, 코드에서는 `get_subscription_count()` 를 확인 (drive_test.py)
- **노드를 끌 때** `ExternalShutdownException` 은 정상 종료로 처리 (에러 아님)

---

## 8. 보호 정지 (`/protective_stop`)

작성: 2026-11-03. UR 보호 정지 흉내 = Isaac Sim `isaacsim/scripts/protective_stop.py`, `isaacsim/config/ros2_iface.yaml`
`protective_stop`, `docs/PLAN.md` 3-7. 기준값도 같다 (`config/ur5_rh_p12_interface.yaml` `protective_stop`).

**물리 스텝마다** 셋 중 하나라도 넘으면 정지:

| 검사 | 기준 | 값을 얻는 곳 (Gazebo) |
|---|---|---|
| 링크 접촉력 | 150 N | `/sim/robot_contacts` (ContactMonitor 플러그인). 손가락 아닌 링크 = 모든 접촉 (`total_N`: 배·드론·로봇 자신), 손가락 4 개 = **환경** 만 (`env_N`) → 드론을 세게 쥐거나 빈손으로 닫아 손가락끼리 닿아도 정지하지 않고, 손가락으로 배를 찍으면 정지 |
| 관절 목표 − 측정 | 5° | `/arm_controller/controller_state` (JTC) 의 `error` = 보간 중인 목표 − 측정 (팔 6 관절) |
| 관절 속도 | 200 °/s | `/joint_state_broadcaster/joint_states` 의 관절각 차분 / stamp 차 (보고된 속도값이 아님, Isaac Sim 과 같음) |

- 세 토픽 모두 물리 스텝마다 오고, **각 메시지가 올 때마다** 검사한다 (스텝을 건너뛰지 않음). 검사는 첫 컨트롤러 상태 (= arm_controller 활성) 부터
- **정지 (`mode: "on"`)**: 팔 JTC 에 점 하나 = 그 순간 측정 관절각 (한 스텝 뒤 도착) → 그 자리에 고정. 이후 `/joint_command` 는 검사만 하고 무시 (개수를 셈),
  `/gripper/command` 는 그대로 받음. 이유와 값을 한 번 ERROR 로그 (`PROTECTIVE STOP at t … s: force 212.3 N (rh_p12_rn_r2) > 150`).
  `/protective_stop` 은 루프 tick (그리퍼 토픽과 같은 때) 마다 true, 평소 false
- **해제 = `/sim/reset` 만** (Isaac Sim 과 같음): scene_node 가 리셋 시작에 부르는 `/ocean_sim2sim/hold` true 에서 해제하고 그때까지의 최댓값을 로그.
  hold 중 (리셋이 팔을 직접 움직이는 동안) 은 검사를 멈추고, hold false 뒤 다시 검사
- **`mode: "measure"`**: 정지하지 않고 최댓값만 기록 (Isaac Sim `--protective-stop measure`). 기준을 넘으면 종류별로 한 번 "would stop" 경고,
  최댓값은 리셋마다·노드 종료 때 로그. 정상 동작에서 기준의 여유를 잴 때 쓴다
- `enabled: false` = 끔 (`/protective_stop` 은 계속 false)

### 접촉력: ContactMonitor 플러그인 (`ocean_sim2sim_gz_plugins`)

gz-sim 시스템 플러그인 `OceanSim2simContactMonitor` 를 팔 xacro (`ur5_rh_p12.gazebo.xacro`) 가 **모델 단위** 로 붙인다
(팔은 배 모델 `wamv` 의 링크라서 배+팔 모델에 붙는다). 로봇 링크 (SDF 이름이 `robot_link_regex` 에 맞는 링크) 의 모든 collision 에
`ContactSensorData` 컴포넌트를 달면 (gz Contact 센서가 내부에서 하는 것) Physics 시스템이 매 스텝 접촉점·힘을 채운다 (DART 도 힘을 채움, 확인).
매 물리 스텝 gz 토픽 `/sim/robot_contacts` (gz.msgs.StringMsg) → ros_gz_bridge (scene_bridge) → ROS `std_msgs/String`:

```json
{"sec": 12, "nanosec": 345000000, "iter": 2963,
 "links": {"rh_p12_rn_r2": {"total_N": 3.21, "env_N": 0.0, "robot_N": 0.0, "drone_N": 3.21}}}
```

- sec·nanosec = 그 스텝의 sim time (joint_states stamp 와 같은 시계). 접촉이 없어도 매 스텝 발행 (`links` 비어 있음) →
  인터페이스는 최근 0.05 s 안에 온 메시지가 없으면 경고 (`contact_max_age`, 플러그인·브리지가 안 도는지)
- 상대 분류 (상대 collision 의 링크·모델 이름): **robot** = 로봇 링크, **drone** = 모델 이름이 `drone` (`drone_model_regex`),
  **env** = 그 밖 전부 = 배 (같은 모델의 로봇 아닌 링크) 와 다른 모델
- 힘 [N] = 접촉 쌍 (이 링크의 collision, 상대 collision) 마다 접촉점 힘의 벡터 합의 크기를 그룹별로 더한 것. `total_N` = 세 그룹 합.
  **Isaac Sim 과 다른 점**: Isaac 은 링크의 순 접촉력 (벡터 합) 이다. gz-sim 은 한 쌍의 같은 힘 (물리 엔진이 먼저 적은 쪽 기준) 을 두 collision 에
  똑같이 넣어 이 링크에 대한 부호를 알 수 없다 (상자 쌓기로 확인: 위·아래 링크가 같은 +z 힘을 받음). 그래서 쌍 크기의 합 =
  닿은 쌍이 하나면 정확, 여럿이 반대쪽에서 누르면 순 힘보다 큼 (정지 쪽으로 보수적)
- **링크 이름 = SDF 이름**: URDF → SDF 가 fixed joint 로 붙은 링크를 부모에 합친다. 이 팔의 로봇 링크:
  `shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`,
  `wrist_3_link` (손목 마운트·그리퍼 브래킷·그리퍼 베이스·손목 카메라 포함), `rh_p12_rn_l1`, `rh_p12_rn_l2`, `rh_p12_rn_r1`, `rh_p12_rn_r2`
  (손가락은 revolute 라 따로 남음 → Isaac 의 `finger_links` 그대로). 팔 베이스 (`robot_mount`, `base_link` …) 는 배 링크 `wamv/base_link` 에 합쳐져 환경으로 친다
  (Isaac 에서는 베이스도 로봇 링크였지만 테이블에 고정된 링크라 정지에 쓰이는 일은 없었음)
- 팔을 바꾸면: xacro 의 `robot_link_regex` 와 설정의 `protective_stop.robot_links`·`finger_links` 를 그 팔의 SDF 링크 이름으로.
  설정에 없는 링크의 접촉이 오면 에러로 중단 (설정 불일치를 조용히 넘기지 않음). xacro `contact_monitor_topic:=""` = 플러그인 없음

### 확인 (오프라인, 2026-11-03)

| 항목 | 결과 |
|---|---|
| 상자 쌓기 (배 1 kg 링크 위 로봇 링크 2 kg · 0.5 kg, 맨 위 드론 0.3 kg), DART 240 Hz | `arm_a`: env 27.44 N (= 2.8 kg × g), robot 7.84 N / `arm_b`: robot 7.84 N, drone 2.94 N ✅ |
| wamv_with_arm 모델 (제어 없이 팔이 갑판으로 쓰러짐) | 로봇 링크들에 env 접촉력, 배 링크는 보고 안 함 ✅ |
| 판정 함수 단위 시험 (가짜 데이터) | 드론 400 N 쥐기·손가락끼리 300 N → 정지 안 함, 손가락-환경 151 N → 정지, 손목 total 150.5 N → 정지, 오차 5.2°, 속도 201 °/s → 정지, measure 는 정지 안 함 ✅ |
| 노드 시험 (가짜 sim 토픽) | 200 N → 정지, 팔 궤적 = 측정 자세 1 개, 이후 `/joint_command` 무시, `/protective_stop` true, hold true → 해제, 다시 명령 적용 ✅ |

### Isaac Sim 과의 차이

| 항목 | Isaac Sim | Gazebo | 이유 |
|---|---|---|---|
| 접촉력 | PhysX 링크 순 접촉력 (벡터 합), 손가락은 환경 prim 마다 크기 합 | 접촉 쌍 크기의 합 (위) | gz-sim 접촉 메시지에서 힘의 부호를 알 수 없음 |
| 환경 | 테이블·바닥·3인칭 받침대 | 배 (팔 베이스 포함) + 드론이 아닌 모든 것 | 장면이 다름 |
| 정지 때 팔 고정 | drive 목표를 그 스텝 관절각으로 | JTC 점 1 개 (1/240 s 뒤 도착) | ROS 쪽에서 목표를 바꾸는 방법 |
| 검사 시점 | 물리 스텝 콜백 하나에서 셋 다 | 토픽마다 도착할 때 (모두 물리 스텝마다 옴). 정지 판정이 sim 보다 몇 ms (벽시계) 늦을 수 있음 | 값이 다른 토픽으로 옴 |
| 리셋 중 | 검사 계속 (순간이동 뒤 resync) | hold 중에는 검사 안 함 | 리셋이 팔을 JTC 로 2 s 동안 직접 움직임 |
| 발행 시각 | 1/30 s 배수 스텝 | 루프 tick (카메라 프레임, 그리퍼 토픽과 같음) | 3 장 루프 |


### 보호 정지 확인 (2026-11-03, 통합 시험)

| 시험 | 결과 |
|---|---|
| 정상 동작 (파지 데모 5 케이스, `mode: measure`) | 안 걸림. 최대: 접촉 0 N, 오차 2.0°, 속도 85.5 °/s (Isaac 정상 최대 2.9°, 180 °/s) |
| 손가락을 갑판으로 천천히 내림 | 491 N (rh_p12_rn_l2) 에서 정지, 팔 고정 |
| 손목 링크를 갑판으로 | 590 N (wrist_2_link) 에서 정지 |
| 손목 관절 1 rad 점프 | 오차 6.4° > 5° 로 정지 |
| 정지 뒤 `/sim/reset` | 성공, 팔이 시작 자세에 도착 (오차 0) |

- **리셋 복구**: 정지 상태 (갑판에 눌린 팔) 에서 바로 시작 자세로 옮기면 경로가 갑판에 걸려 멈췄다 (Isaac Sim 은 순간이동).
  scene_node 가 리셋 시작 때 (hold 가 정지를 풀기 전) 정지였는지 기억했다가 먼저 `reset_escape_pose` (홈, 팔을 들어 올림) 를 거친다.
  리셋 끝에 팔이 `init_pose` 에 `reset_arrive_tol` (0.02 rad) 안으로 왔는지 확인하고, 아니면 리셋 실패 (조용히 넘어가지 않음)
- 정지가 풀린 직후 리셋 전에 쌓인 메시지로 다시 걸리지 않게, 리셋이 끝난 뒤 0.1 s 이후 stamp 의 표본만 검사
