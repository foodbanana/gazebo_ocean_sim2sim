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

- 모든 stamp = sim time. `ros2 topic hz --use-sim-time <토픽>` 으로 sim 기준 주기를 본다
- 손목 카메라 `/cam/wrist/color/image_raw`·`camera_info`: [`wrist_camera.md`](wrist_camera.md)
- `/protective_stop`, `/sim/reset`, `/sim/drone_kill` 은 다음 단계에서 추가

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
