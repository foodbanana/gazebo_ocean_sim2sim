# 배 움직임 기록 (선박 움직임 예측기용)

작성: 2026-11-25

`moving_base_prediction` 의 사양 `docs/gazebo_ocean_recording/01_Recording_Spec.md` (이하 "사양") **1 단계** 를 이 레포에서 기록한다:
A (100 Hz 필수), C (실행마다 메타데이터), E1·E2·E3·E6 (한 번 측정·확인), 팔 움직임 실행에는 B5 (관절 각도)·B8 (팔 명령값).
패키지: `ocean_sim2sim_recording`. 예측기 쪽 레포는 읽기만 했다.

## 1. 실행

```bash
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash     # venv 필요 없음 (PX4 안 씀)

# 실행 1 회 = sim 시작부터 표 만들기까지 한 명령 (sim 을 직접 띄움: 다른 sim 이 돌고 있으면 거부)
ros2 run ocean_sim2sim_recording record_run.py gain03_dir0 --arm fixed    --duration 900
ros2 run ocean_sim2sim_recording record_run.py gain03_dir0 --arm scripted --seed 1 --duration 900
#   파도: --wave-gain 0.3 --wave-period 5 --wave-direction-deg 45   (빈칸 = world 값: empty_ocean gain 0.3, period 5, 방향 0)
#   잔잔한 물: --world empty_ocean_calm

# E2 (좌표계·부호, 잔잔한 물, 자체 sim)
ros2 run ocean_sim2sim_recording e2_sign_check.py

# 표 다시 만들기 (record_run.py --no-convert 뒤, 또는 변환 규칙을 바꾼 뒤)
ros2 run ocean_sim2sim_recording bag_to_table.py ocean_gazebo_recordings/<run>
```

`record_run.py` 순서:

1. `wamv_arm.launch.py headless:=true drone_mode:=none ground_truth:=true` (+ 파도 인자). **드론 없음** (예측기용 데이터)
2. "scene ready" (배가 자리 잡은 뒤, 시작 후 약 8 s sim) → `/sim/reset` (팔 init_pose, 그리퍼 열림) → 5 s 안정 (`--settle`)
3. `arm_motion_node.py` 시작 (아직 움직이지 않고 interface 와 bag 이 명령 topic 을 구독하기를 기다림)
4. `ros2 bag record -s mcap --use-sim-time` 로 아래 3 장의 topic 기록 시작 (bag 이 마지막: 기록 중에는 새 ROS 노드가 생기지 않음.
   새 노드가 들어오면 기존 topic 전달이 잠깐 멈출 수 있음, 시험 기록에서 `/joint_states` 0.34 s). 구독이 다 되면 팔 노드가 2 s 동안 init 자세를
   명령으로 보낸 뒤 (`lead_in`) `--duration` sim 초 동안 움직임
5. bag·팔·sim 종료, wall 1 초마다 RTF 기록 (`rtf.jsonl`)
6. `meta.json` (C + E3 + E6), 100 Hz 표 (`data_100hz.npz` / `.csv`), `e1_timing.json` (E1)

끝 코드 0 = 유효한 실행 (보호 정지 없음, 표 만들어짐). 실행 1 회 15 분이면 wall 약 16 분 (RTF 1.0), bag 약 280 MB (120 s 에 37 MB).
기록 중에는 같은 PC 에서 `ros2 topic echo`, rqt 같은 새 ROS 노드를 띄우지 않는다 (위 이유).

### 1.1 팔 동작 (`arm_motion_node.py`, 설정 `config/recording.yaml` `arm_motion`)

| 모드 | 내용 |
|---|---|
| `fixed` | 포획 준비 자세 (init_pose = Isaac Sim `under_drone`, 그리퍼 위쪽, 드론 자리 30 cm 아래) 유지, 그리퍼 열림. `/joint_command` 는 계속 init 자세로 보냄 (B8 이 모든 실행에 있음) |
| `scripted` | 포획과 비슷한 동작 반복: Isaac Sim 드론 목표 영역 (로봇 base 기준 x 0.42~0.68, y −0.22~0.22, z 0.68~0.82 m) 에서 무작위 잡을 점 → 그 15 cm 아래로 직선 이동 (0.10~0.30 m/s) → 위로 접근 (0.03~0.10 m/s) → 그리퍼 닫기·유지 (0.5~2 s)·열기 → 아래로 후퇴 → 30 % 확률로 init 위치 복귀 → 쉼 (0~1.5 s). 도구 yaw ±20°. 값은 모두 `--seed` 로 정해짐 (같은 seed = 같은 동작) |

- policy 가 아니라 **스크립트 동작** (사양: 어느 쪽인지 C 에 적음 → `meta.json` `arm.mode`).
- 명령은 텔레옵·정책과 같은 길: `/joint_command` 60 Hz → interface (30 Hz 루프, 33 ms 보간) → JTC. 관절 목표 속도 ≤ 1.5 rad/s, 측정값보다 0.08 rad 이상 앞서지 않음 (보호 정지 5°). IK = 명령 관절에 대한 DLS (`grasp_demo.py` 와 같음).
- 구간 시작마다 `/recording/arm_event` (JSON) 가 bag 에 들어감 (구간 이름, 잡을 점, 속도, 시간).
- 팔 반동 크기 (시험 기록, 파도 gain 0.3, 120 s): 같은 sim 시각의 팔 고정 실행과 비교해 roll 차이 표준편차 0.014° (최대 0.078°),
  pitch 0.020° (최대 0.097°), yaw 0.011° (최대 0.073°), 상하 0.21 mm (최대 1.1 mm).
  파도에 의한 pitch 최대-최소 2.1° 보다 훨씬 작다. 잔잔한 물에서 shoulder_pan 을 ±0.8 rad, 0.5 Hz 로 크게 흔들면 roll 최대-최소 0.5° (2026-11-25 측정) →
  포획 동작은 대부분 위아래·앞뒤라 배를 덜 흔든다.

## 2. 기록 파일

위치: **`ocean_gazebo_recordings/<YYYYMMDD_HHMMSS>_<이름>_<fixed|scripted>/`** (이 레포 맨 위, git 제외). `--out-dir` 로 바꿀 수 있음.

| 파일 | 내용 |
|---|---|
| `bag/` | rosbag2 **mcap** (원본 topic 그대로, bag 시각 = sim time) |
| `meta.json` | C: 실행 정보, 팔 모드·seed·설정, 배 상태, 바람·조류, 파도 (E6), 좌표계, 시뮬레이션 (물리 스텝, RTF, 버전), topic ↔ 항목 대응, E3 장착 위치, IMU 잡음 설정, 표 열 이름 |
| `data_100hz.npz` | 100 Hz 표 (numpy 배열, 아래 2.1) |
| `data_100hz.csv` | 같은 표, 첫 줄 = 열 이름 (사양 항목 번호 앞에 붙임, 예: `A3_gt_quat_xyzw_w`) |
| `e1_timing.json` | E1: topic 별 간격 분포, 빠진 샘플, 표 행 수, RTF 통계 |
| `rtf.jsonl` | wall 1 초마다 sim 시각과 RTF |
| `record_run.log`, `launch.log`, `bag_record.log`, `arm_motion.log` | 실행 로그 |

`e2_sign_check.py` 는 `ocean_gazebo_recordings/<시각>_e2_sign_check/e2_sign_check.json` 을 쓴다.

### 2.1 100 Hz 표 (`data_100hz.npz` 키)

시간 격자: **t = k × 0.01 s sim time (정수 ns 로 계산, 누적 오차 없음)**, 정답값과 IMU 가 둘 다 있는 구간. 열 채우는 규칙:

- 정답값 (240 Hz = 매 물리 스텝): 위치·속도 선형 보간, 자세 slerp
- IMU, 관절, 명령, 그리퍼: t 이전의 **가장 최근 샘플** (사양의 "그 시점의 가장 최근 값" 규칙, 미래 값을 쓰지 않음). 나이는 `imu_age_s`, `joint_cmd_age_s`
- `valid` = 정답값에 빈틈 없음 (물리 스텝 1.5 배 넘는 간격 없음) 그리고 IMU 샘플 나이 ≤ IMU 주기 2 배. 행은 지우지 않음 (받는 쪽에서 자름)

| 키 | 크기 | 항목 | 내용 | 단위·좌표계 |
|---|---|---|---|---|
| `t` | N | A1 | sim time | s |
| `gt_pos` | N×3 | A2 | WAM-V 본체 (`wamv/base_link`) 원점 위치 | m, world (ENU) |
| `gt_quat_xyzw` | N×4 | A3 | 본체 자세 (본체 → world 회전, w ≥ 0) | x, y, z, w |
| `gt_lin_vel_body` | N×3 | A4 | 본체 원점 선속도 | m/s, 본체 (FLU) |
| `gt_ang_vel_body` | N×3 | A5 | 본체 각속도 | rad/s, 본체 (FLU) |
| `imu_quat_xyzw` | N×4 | A6 | IMU 자세 (**잡음 없음**: gz IMU 에 자세 잡음 설정이 없음) | IMU → world (ENU) |
| `imu_ang_vel` | N×3 | A7 | IMU 각속도 (잡음 있음) | rad/s, IMU (= 본체 축, FLU) |
| `imu_lin_acc` | N×3 | A8 | IMU 가속도 = 비력 (정지 시 z ≈ +9.8, 잡음 있음) | m/s², IMU |
| `imu_age_s` | N | (E1) | 쓴 IMU 샘플의 나이 | s |
| `valid` | N | (E1) | 1 = 유효 | – |
| `joint_names` | 6 | – | 관절 이름 (순서) | – |
| `joint_pos` | N×6 | B5 | 팔 관절 각도 | rad |
| `joint_pos_age_s` | N | – | 쓴 관절 샘플의 나이 (120 Hz 면 ≤ 8.3 ms; 크면 그 구간 관절 샘플이 빠진 것) | s |
| `joint_cmd` | N×6 | B8 | 팔 명령 (관절 목표, `/joint_command`) | rad |
| `joint_cmd_age_s` | N | – | 쓴 명령의 나이 | s |
| `gripper_cmd_raw` | N | B9 | 그리퍼 명령 | raw 0 (열림) ~ 1150 (닫힘) |
| `gripper_pos_raw` | N | B10 | 그리퍼 위치 | raw (rad = raw / 1150 × 1.1351) |

## 3. topic ↔ 항목 번호

| topic | 형식 | 주기 (sim) | 항목 | 내용 |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 물리 스텝 | A1 | sim time |
| `/sim/boat_ground_truth` | `nav_msgs/Odometry` | 240 Hz (매 스텝) | **A2, A3, A4, A5** | `GroundTruthPublisher` (이 레포의 Gazebo 플러그인): `wamv/base_link` 의 world 자세 (`pose.pose`), 본체 좌표계 선속도·각속도 (`twist.twist`), 물리 엔진 값 그대로. frame_id `world`, child `wamv/base_link` |
| `/base/imu` | `sensor_msgs/Imu` | 100 Hz | **A6, A7, A8** | `orientation` (A6), `angular_velocity` (A7), `linear_acceleration` (A8). frame `wamv/wamv/imu_wamv_link/imu_wamv_sensor` |
| `/joint_states` | `sensor_msgs/JointState` | 120 Hz | **B5** | 팔 6 관절 position·velocity·effort |
| `/joint_command` | `sensor_msgs/JointState` | 60 Hz | **B8** | 팔 관절 목표 (arm_motion_node → interface) |
| `/gripper/command` | `std_msgs/Float64` | 이벤트 | B9 | 그리퍼 명령 raw (header 없음 → bag 시각 = sim time) |
| `/gripper/joint_states` | `sensor_msgs/JointState` | 약 30 Hz | B10 | 그리퍼 위치 raw |
| `/protective_stop` | `std_msgs/Bool` | 약 30 Hz | – | 확인용 (항상 false 여야 함) |
| `/recording/arm_event` | `std_msgs/String` | 이벤트 | (C) | 팔 동작 구간 시작 (JSON) |

없는 항목 (1 단계 범위 밖): B1 (로봇 베이스 pose = A2·A3 + E3 의 고정 변환으로 계산 가능), B2, B3·B4 (드론 없음), B6 (추진 없음), B7, B11, D, E4·E5·E7.

## 4. 좌표계와 부호 (E2)

| | 기록된 값 | 예측기 (AURA 형식: 본체 FRD, world NED, Z-Y-X) 로 바꾸기 |
|---|---|---|
| world | ENU (x 동, y 북, z 위). 배는 원점에서 선수 +x 로 시작 | NED = M · ENU, M = [[0,1,0],[1,0,0],[0,0,−1]] |
| 본체·IMU | FLU (x 선수, y 좌현, z 위) | FRD = F · FLU, F = diag(1, −1, −1) (각속도·가속도: x 그대로, y·z 부호 반대) |
| 자세 | 본체 → world 쿼터니언 | R_ned_frd = M · R_enu_flu · Fᵀ → roll 같음, **pitch 부호 반대**, yaw_NED = 90° − yaw_ENU |

`e2_sign_check.py` 측정 (잔잔한 물, 2026-11-25):

- 정지 IMU 가속도 (−0.07, 0.05, **+9.84**) m/s²: 비력이 IMU z (위) 쪽 = FLU. 작은 x·y 는 IMU 바이어스 (bias_mean 0.05) 와 배 기울기
- 정지 자세: pitch +0.64° (ENU/FLU) = 선수가 조금 내려감 (팔 무게가 앞쪽)
- **선수 들기** (`ApplyLinkWrench` 로 world −y 축 토크 500 N·m, 8 s): 선수 끝 +4.1 cm, pitch **−0.81°** (ENU/FLU) = **+0.81°** (NED/FRD 로 바꾼 값),
  드는 동안 IMU 자이로 y **−0.043 rad/s** (정답 각속도 y −0.043 rad/s). 정답 자세와 IMU 자세는 같은 값
- 결론: 기록 그대로는 "선수 들기 = pitch 음수, 자이로 y 음수". 위 변환 뒤 NED/FRD 에서 "선수 들기 = pitch 양수"

## 5. 장착 위치 (E3)

`meta.json` `mounting_e3` (robot_description 에서 계산). 본체 = `wamv/base_link` (VRX 선체 원점, URDF 질량 180 kg).

| | 본체 좌표계 위치 [m] | 방향 |
|---|---|---|
| IMU (`wamv/imu_wamv_link`) | (−0.15, 0, 1.30) | 본체와 같음 |
| 로봇팔 베이스 (`robot_mount`) | (0, 0, 1.30) | 본체와 같음 |
| TCP, init_pose 일 때 | (0.617, −0.007, 1.733), 본체 원점에서 1.84 m | 그리퍼 위쪽 |
| 플랜지 (`tool0`) → TCP (`rh_p12_rn_tcp`) | **0.125 m** (이 모델; 실제 그리퍼 약 0.151 m) | |

## 6. 파도 모델 (E6)

`meta.json` `waves` 에 실행마다 실제 파라미터 (`/vrx/wavefield/parameters` 한 메시지) 와 성분별 값이 들어간다.

- **VRX Wavefield, PMS** (Pierson-Moskowitz 스펙트럼에서 주파수 **3 개**를 뽑음). 높이 η = Σ aᵢ cos(kᵢ (dᵢ·x) − ωᵢ t) × (1 − e^(−t/τ)), τ = 2 s
- 성분 주기 = 피크 주기 × 1.1⁻¹, 1, 1.1 (period 5 s: 4.55 / 5.0 / 5.5 s), 방향 = 평균 방향 + (−0.4, 0, +0.4) rad (±23°)
- **위상이 모두 0, 난수 seed 없음** → 같은 파라미터면 매번 똑같은 파도. 맞놀이 약 50~55 s, 정확한 반복 550 s (period 5 s)
- gain 0.3, period 5 s: 성분 진폭 3.6 / 3.9 / 3.9 cm, Hs ≈ 0.19 m (본체 상하 최대-최소 측정 223 mm)
- 성분 수를 늘릴 수 없음: VRX PMS 계산의 주파수 간격 표가 3 칸 (4 이상은 배열 밖을 읽음), 수면 셰이더도 3 개
- **사양 "주의" 에 해당**: 거의 주기적이라 AR 이 지나치게 잘 맞을 수 있다. 결과는 AURA·Pohang 실측과 나란히 해석. 너무 쉽다고 나오면 성분이 많고 위상이 무작위인 파도 플러그인을 이 레포에 만든다 (다음 단계)
- 방향: 파도가 **나아가는** 쪽, world +x 에서 반시계. 배가 선수 +x 로 시작하므로 0° = 뒤에서 오는 파도 (following sea), 180° = 정면 (head sea), 90° = 좌현 쪽으로 진행
- 바람 0 m/s, 조류 없음, 배 추진 없음 (정지, 파도에 떠밀림: 124 s 에 x −0.20 m, y −0.02 m, yaw 변화 0.003°)

## 7. 기록 간격 (E1, 시험 기록 2026-11-25, gain 0.3, 120 s × 2: `ocean_gazebo_recordings/20261125_111528_pilot_gain03_dir0_fixed`, `..._111804_..._scripted`)

| | 결과 |
|---|---|
| 정답값 `/sim/boat_ground_truth` | 240.00 Hz, 간격 모두 4.1667 ms, 빠진 샘플 0 |
| IMU `/base/imu` | 평균 100.00 Hz, 간격 **8.33 ms 와 12.5 ms** (물리 240 Hz 격자 위의 100 Hz: 2 스텝 또는 3 스텝), 빠진 샘플 0 |
| 100 Hz 표의 IMU 나이 | 평균 7.5 ms, 최대 9.2 ms |
| `/joint_states` | 120 Hz (8.33 ms, 드물게 12.5 ms), 표의 관절 샘플 나이 최대 11.7 ms |
| `/joint_command` | 60.00 Hz, 간격 16.67 ms 고정 |
| 표 | 행 모두 valid (12386 / 12387 행), 보호 정지 0 |
| RTF | 평균 1.00 (wall 1 초 표본 중 최저 0.70 한 번, PC 의 순간 지연). lockstep 이라 RTF 가 1 보다 작아도 sim time·물리는 같음 |
| IMU 자이로 − 정답 각속도 | 축마다 표준편차 0.009 rad/s (= VRX 잡음 설정), 평균 ±0.002 rad/s (바이어스) |

정답값은 정확히 100 Hz 격자로 보간되므로 간격 문제가 없다. IMU 는 실제 샘플 시각 (8.3 / 12.5 ms) 그대로 두고 표에는 가장 최근 값을 쓴다.

## 8. 짝 맞추기 (팔 고정 ↔ 팔 움직임)

파도는 sim 시각만의 함수이고 시뮬레이션은 결정적이다. 같은 조건의 두 실행을 **같은 `t` 끼리** 비교하면, 팔이 움직이기 전까지 배 자세 차이가 0.00002° 이하 (시험 기록, 같은 계산 결과).
→ 사양 F 의 "같은 파도로 짝 맞추기" 가 따로 할 일 없이 된다. 차이 = 팔의 영향.

## 9. 본 기록 (사양 F) 예

조건마다 팔 고정 / 팔 움직임 한 쌍, 각 10~20 분:

```bash
for g in 0.15 0.3 0.6; do for d in 0 45 90; do
  ros2 run ocean_sim2sim_recording record_run.py g${g}_d${d} --arm fixed    --wave-gain $g --wave-direction-deg $d --duration 900
  ros2 run ocean_sim2sim_recording record_run.py g${g}_d${d} --arm scripted --wave-gain $g --wave-direction-deg $d --duration 900 --seed 1
done; done
```

- 파도 방향은 배 기준이 아니라 world 기준이다. 배는 추진 없이 떠 있고 124 s 시험에서 yaw 변화가 작았지만 (0.003°), 긴 실행에서는 `meta.json` 이 아니라 기록된 A3 로 실제 배 기준 방향을 확인한다
- 배 이동 (B6, 추진) 은 2 단계
