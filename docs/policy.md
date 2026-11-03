# 정책 추론·평가 (openpi 정책 클라이언트, Gazebo)

작성: 2026-11-03 · `ocean_sim2sim_policy` (`ocean_sim2sim_policy/client.py`, `scripts/policy_client_node.py`,
`scripts/policy_eval.py`, `scripts/fake_policy_server.py`, `config/policy_client.yaml`), `ocean_sim2sim_interface/observe.py`

Isaac Sim 프로젝트 `data_collection_ur5_gripper` 의 정책 클라이언트 (`policy/policy_client_node.py`, `policy/policy_client.yaml`,
문서 `docs/policy_client.md`·`docs/policy_inference_desktop.md`) 를 Gazebo 로 옮기고, 판정까지 하는 평가 실행기를 더한 것.
sim 은 텔레옵·정책 누가 명령을 보내는지 모른다 (`/joint_command`·`/gripper/command` 만 받음).

## 1. 구조

```text
+-----------------------------------------------------------------------------------------------------------+
| [A] GAZEBO STACK  ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py ...      (all stamps = sim time)    |
|     ocean + WAM-V + UR5 / RH-P12-RN / D435i, interface node, scene node (drone, /sim/reset, /sim/drone_kill)|
+-----------------------------------------------------------------------------------------------------------+
   |  /cam/wrist/color/image_raw   rgb8 640x480 30 Hz  (NO third view)   ^  /joint_command   JointState 60 Hz
   |  /joint_states                arm 6 joints 120 Hz                    |                   (stamp = sim time)
   |  /gripper/joint_states        present raw ~30 Hz                     |  /gripper/command  Float64 0 / 1150
   |  /base/imu  (option)          sensor_msgs/Imu 100 Hz                 |                   (on change)
   |  /clock, /protective_stop, /robot_description                        |
   |                               services: /sim/reset, /sim/drone_kill <--------------------+
   v                                                                      |                    |
+-----------------------------------------------------------------------------------------------------------+
| [B] POLICY CLIENT  ros2 run ocean_sim2sim_policy policy_client_node.py | policy_eval.py   (venv: openpi-client)|
|     1. observe: pair image(s) / joints / gripper (/ base_imu 40 ms mean) at t_obs = image stamp             |
|        state (7) = arm 6 [rad] + present / 1150,  cameras from config (wrist only),  prompt                  |
|     2. obs -> server -> action chunk (10, 7)   (next request in the background at chunk step 4)              |
|     3. execute on sim time: 60 Hz, linear interpolation between steps, skip steps already passed              |
|     4. safety (teleop values): 1.5 rad/s, lead <= 0.08 rad, URDF limits - 0.05 rad; stop on protective stop / |
|        desync > 0.2 rad / stale observation;  gripper: output > 0.5 -> 1150                                  |
|     policy_eval.py only: grasp judge (observe.py + grasp_judge.py) -> ~/ocean_sim2sim_reports/policy_eval_*/ |
+-----------------------------------------------------------------------------------------------------------+
   |  websocket ws://<host>:8000 (msgpack)                                ^
   v  obs: state, image(s) HxWx3 uint8, prompt                            |  actions (10, 7)
+-----------------------------------------------------------------------------------------------------------+
| [C] POLICY SERVER  ~/openpi: uv run scripts/serve_policy.py policy:checkpoint ...   (NOT in this repository) |
+-----------------------------------------------------------------------------------------------------------+

 [D] OPERATOR: watches Gazebo; drone grasped -> motors off (ros2 service call /sim/drone_kill ...), or --auto-kill
```

## 2. 설치 (한 번)

`openpi-client` (websocket + msgpack, JAX·GPU 없음) 를 워크스페이스 venv (`~/vrx_ws/.venv`, `--system-site-packages`, README) 에 넣는다.
시스템 Python 에는 설치하지 않는다.

```bash
# openpi 를 받아 둔 경우 (Isaac 프로젝트와 같은 코드)
~/vrx_ws/.venv/bin/python -m pip install -e ~/openpi/packages/openpi-client
# 아니면 GitHub 에서 (또는 자기 fork)
~/vrx_ws/.venv/bin/python -m pip install "openpi-client @ git+https://github.com/Physical-Intelligence/openpi.git#subdirectory=packages/openpi-client"
```

2026-11-03 설치 결과: venv 에 `openpi-client 0.1.0` (editable), `websockets 17.3`, `msgpack 1.2.3`, `dm-tree 0.1.10`,
`absl-py 2.5.1` 가 새로 들어감 (버전은 Isaac `~/openpi_client/venv` 와 같게 고정). `numpy 1.26.4`·`pillow 10.2.0`·`attrs`·`wrapt` 는
시스템 것을 그대로 씀 (`numpy<2` 조건을 시스템 1.26.4 가 만족 → 바꾸지 않음).

## 3. 실행

### 3.1 터미널 4 개

```bash
# 모든 터미널: cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash

# [A] 터미널 1: Gazebo (PX4 드론, 기본 파도). 잔잔한 물: world:=empty_ocean_calm
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py

# [C] 터미널 2: 정책 서버 (~/openpi, uv 환경. 이 레포 밖). "server listening on 0.0.0.0:8000" 까지
cd ~/openpi && XLA_PYTHON_CLIENT_PREALLOCATE=false uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_ur5 --policy.dir=checkpoints/pi05_ur5/full_first13_10k/9999
#   XLA_PYTHON_CLIENT_PREALLOCATE=false 필수 (없으면 JAX 가 GPU 75 % 를 미리 잡음). 다른 PC 면 클라이언트에 --host

# [B] 터미널 3: 평가 (판정·보고서) 또는 클라이언트만
ros2 run ocean_sim2sim_policy policy_eval.py --episodes 10 --duration 30              # 사람이 모터 정지
ros2 run ocean_sim2sim_policy policy_eval.py --episodes 10 --duration 30 --auto-kill 1.0
ros2 run ocean_sim2sim_policy policy_client_node.py --reset --episodes 5 --duration 30  # Isaac 과 같은 클라이언트 (판정 없음)

# [D] 터미널 4: 사람이 Gazebo 를 보고 잡혔으면 드론 모터 정지
ros2 service call /sim/drone_kill std_srvs/srv/Trigger
```

### 3.2 정책 없이 배관 시험 (가짜 서버)

`fake_policy_server.py` 는 openpi websocket 규약 그대로 상수 chunk (10, 7) 를 돌려준다. 기본은 받은 state 의 팔 6 값을
그대로 (로봇이 제자리), `--gripper 1.0` 이면 닫기, `--constant q1..q6 g` 면 그 값. 첫 관측의 키·모양을 출력한다.

```bash
ros2 run ocean_sim2sim_policy fake_policy_server.py --delay 0.18 --require observation/state observation/wrist_image
ros2 run ocean_sim2sim_policy policy_client_node.py --reset --duration 10
```

### 3.3 옵션

| `policy_client_node.py` | 뜻 |
|---|---|
| `--duration S` (필수) | 에피소드 길이 [sim s] |
| `--reset` / `--episodes N` | 에피소드마다 `/sim/reset` / N 번 (2 이상이면 `--reset` 필수) |
| `--auto-kill S` | 닫기 명령 S 초 뒤 그리퍼가 막혀 있으면 (present < 0.9) `/sim/drone_kill` 한 번 |
| `--host`, `--port`, `--prompt`, `--config` | 설정 덮어쓰기 |

| `policy_eval.py` | 뜻 |
|---|---|
| `--episodes N`, `--duration S` (필수), `--auto-kill S` | 위와 같음 (리셋은 항상, `--no-reset` 은 1 에피소드 디버그용) |
| `--operator-wait S` (기본 15) | 정책이 끝났는데 잡힌 채 모터가 돌면 사람의 모터 정지를 S sim s 기다림 (팔은 마지막 목표 유지). `--auto-kill` 이면 기다리지 않고 바로 끔 |
| `--hold-margin S` (기본 1) | 모터 정지 뒤 판정 hold_time (2.5 s) 에 더 기다리는 시간 |
| `--stop-on-result` | 판정이 나면 (held / failed) 정책 실행을 바로 끝냄 (기본 끔 = Isaac 처럼 duration 끝까지) |
| `--judge-config` | 판정 설정 (기본: 설치된 `ocean_sim2sim_bringup/config/grasp_demo.yaml` 의 `judge`, `control_hz`, 그리퍼 값) |
| `--report-dir` (기본 `~/ocean_sim2sim_reports`) | 보고서 위치 (실행 때 `~` 를 풂) |
| `--base-link`, `--tcp-link`, `--boat-link` | TCP 계산 링크 (grasp_demo.py 와 같은 기본값) |

## 4. 설정 `config/policy_client.yaml`

항목이 빠지면 에러로 멈춘다 (코드에 기본값 없음, Isaac 과 같음). 값은 Isaac `policy_client.yaml` 그대로, 새 항목만 아래.

| 항목 | 값 | 뜻 |
|---|---|---|
| `server.host` / `port` | localhost / 8000 | 정책 서버 |
| `prompt` | "pick up the drone" | 학습 데이터셋 task |
| `control_hz` / `replan_steps` / `publish_hz` | 25 / 4 / 60 | chunk 한 칸 간격 / 다음 추론 시작 칸 / 명령 주기 |
| `obs_max_age` | 0.066 | t_obs 보다 이만큼 넘게 오래된 입력 → 에러 (stage1 `--max-age`) |
| `gripper.*` | 0.5, 0, 1150, 1150 | 닫기 기준, 열기·닫기 raw, state[6] 스케일 |
| `max_joint_speed` / `max_lead` / `limit_margin` / `desync` | 1.5 / 0.08 / 0.05 / 0.2 | 텔레옵과 같은 안전 한계 |
| **`arm_joints`** (새) | UR5 6 관절 | state·action·`/joint_command` 의 관절 순서. 한계는 `/robot_description` 의 `<limit>` |
| **`cameras`** (새) | `{observation/wrist_image: /cam/wrist/color/image_raw}` | 관측 키 → 이미지 토픽 (rgb8). Gazebo 는 손목 카메라만 |
| **`blank_images`** (새) | `{}` | 검은 이미지로 보낼 관측 키 → `[높이, 너비]`. sim 에 없는 카메라가 필요한 체크포인트의 **배관 시험 전용** |
| **`base_imu.enabled`** (새) | false | `/base/imu` 를 관측에 넣을지 |
| `base_imu.topic` / `window` | /base/imu / 0.040 | 토픽 / 평균 구간 [s] (= 1 / 25 Hz) |
| `base_imu.mode` / `key` | key / observation/base_imu | `key`: obs[key] = (6,) float32, `state`: state 뒤에 붙여 (13,) |
| `topics.*` | Isaac 과 같음 + **`robot_description`** | 토픽·서비스 이름 |

### 4.1 카메라

- 지금 Gazebo 에는 third view 카메라가 없다 (손목만, 정책은 손목만으로 다시 미세조정 예정)
- `cameras` 에 있는데 sim 이 내지 않는 토픽 → sim 이 떠 있으면 10 s 뒤 바로 에러:
  `no publisher for {'observation/base_image': ...} -- camera(s) [...] are not in this sim (Gazebo has the wrist camera only) ...`
- 지금 체크포인트 (`pi05_ur5` first13, 카메라 2 대: `UR5Inputs` 가 `observation/base_image` 를 꼭 읽음) 로 배관만 볼 때:
  `blank_images: {observation/base_image: [480, 640]}`. 정책은 학습 때와 다른 입력을 보므로 결과는 의미 없음

### 4.2 base_imu (움직이는 베이스, 계획)

Isaac `moving_base_architecture.md` [4] 의 계획과 같은 규칙. Isaac stage1 (`lerobot_stage1_extract_bag.py --base-imu topic`) 과 같은 계산:

```text
6 값 = [angular_velocity.x, .y, .z (rad/s), linear_acceleration.x, .y, .z (m/s^2, 비력: 정지 = +9.81 위쪽)]
base_imu(t_obs) = header.stamp 가 (t_obs - 0.040, t_obs] 인 /base/imu 메시지들의 평균. 구간에 메시지가 없으면 에러
```

- t_obs 에도 들어간다: t_obs = min(카메라들의 최신 stamp, IMU 최신 stamp). IMU (100 Hz) 가 이미지보다 먼저 오므로
  보통 t_obs 는 Isaac 처럼 이미지 stamp 그대로이고, IMU 가 늦을 때만 구간이 다 찰 때까지 앞당긴다
- Gazebo IMU 는 VRX WAM-V IMU (100 Hz, 잡음, 보트 위치) → Isaac 학습 데이터 (120 Hz, 잡음 없음, 로봇 base) 와 다름 (`docs/drone_px4.md` 5장)
- 서버 쪽 openpi config 가 base_imu 를 받아야 의미가 있다 (Isaac 계획: state 7 + base_imu 6, `discrete_state_input`)

## 5. 클라이언트 안쪽 (Isaac 과 같음)

자세한 설명·예시는 Isaac `docs/policy_client.md` 와 같다. 요약:

- **스레드 3 개**: spin (SingleThreadedExecutor.spin_once 반복, 버퍼에 쌓기) / 메인 (60 Hz 루프) / 추론 (필요할 때만)
- **관측 짝짓기**: t_obs = 카메라들 최신 stamp 중 가장 이른 것. 관절·그리퍼·다른 카메라는 t_obs 이하에서 최신.
  0.066 s 넘게 오래되면 에러. state = [팔 6, present / 1150]
- **chunk 실행**: 칸 i = sim `t_obs + i / 25`. 지금 위치 x = (now − t_obs) × 25, 칸 ⌊x⌋ 와 ⌊x⌋+1 직선 보간, x ≥ 9 면 칸 9 유지
  (`held_frac`). x ≥ 4 에서 다음 추론을 뒤에서 시작, 오면 바로 교체 (`chunk_arrive_step_median`)
- **안전 한계**: 한 번에 1.5 rad/s × 1/60, 측정 ± 0.08 rad, 관절 한계 − 0.05 rad (`limited_frac`)
- **그리퍼**: 칸 ⌊x⌋ 출력 > 0.5 → 1150, 바뀔 때만 보냄
- **멈춤**: duration, 보호 정지 (`/protective_stop` true), desync (측정 − 보낸 목표 > 0.2 rad), 관측 에러
- **auto-kill**: 닫기 S 초 뒤 present < 0.9 → `/sim/drone_kill` 한 번

## 6. 평가 실행기 `policy_eval.py`

에피소드마다:

1. `/sim/reset` (scene 의 seed·드론 목록·offset 을 그대로 씀. 답 JSON 을 기록. 첫 드론이 뜨기 전 "not ready" 면 기다렸다 다시)
2. 드론 준비 (PX4 armed / 고정 드론 attached) → sim 0.2 s
3. 정책 클라이언트 실행 (`client.py`, `policy_client_node.py` 와 같은 코드) + 동시에 판정 스레드:
   `GraspJudge` (`ocean_sim2sim_interface/grasp_judge.py`, `grasp_demo.yaml` 의 `judge` 값) 를 `control_hz` (50 Hz, sim) 마다 갱신.
   관측은 `grasp_demo.py` 와 같은 출처 (`ocean_sim2sim_interface/observe.py` 로 공유): `/drone/odometry`, `/drone/contacts`
   (같은 sim 스텝 메시지 합침), TCP = `/robot_description` 체인 + `/sim/boat_odometry`, `/drone/px4/state` armed,
   `/gripper/joint_states`. 닫기 명령 여부는 `/gripper/command`
4. 모터 정지: `--auto-kill` (클라이언트 규칙) 또는 사람. 정책이 끝났는데 잡힌 채 모터가 돌면 `--auto-kill` 이면 바로 끄고,
   아니면 `--operator-wait` 동안 사람을 기다림
5. 모터 정지 뒤 hold_time 2.5 s 동안 판정 (held = 성공) → `finish`

판정 규칙은 `docs/grasp_demo.md` (grasped: 닫는 중 그리퍼가 멈춤 + 두 손가락 접촉 / held: 모터 정지 뒤 2.5 s 버팀 /
실패 이유 no_grasp_empty, no_grasp_contact, slip, dropped, opened_early, timeout:<상태>). 정책이 한 번도 잡지 못하면 `timeout:approach`.

**보고서** `~/ocean_sim2sim_reports/policy_eval_<시각>/`:

| 파일 | 내용 |
|---|---|
| `report.txt` | 로그 전체 + 요약 표 (에피소드, 드론 목록 점, 결과, 이유, 멈춤 이유, 잡은 시각, 모터 정지 시각, 추론 시간, held·limited, 그리퍼 바뀐 수, 보호 정지 수) + 성공률 |
| `summary.json`, `summary.csv` | 같은 표 (성공률, 결과별 개수) |
| `config.json` | 인자, 클라이언트 설정, 판정 설정, 서버 metadata |
| `ep<NN>_result.json` | 리셋 답, 클라이언트 결과 (+ chunk 마다 추론 wall 시간, 서버 `infer_ms`, 도착 칸, 그리퍼 바뀐 시각), 판정 결과·이벤트, 보호 정지 이벤트 (sim 시각, 켜짐/꺼짐), 링크별 최대 접촉력 |
| `ep<NN>_timeline.csv` | 판정 틱마다 TCP·드론 위치, 그리퍼 각, 손가락 힘, armed, 그리퍼 명령, 보호 정지, TCP 좌표계 드론 위치 |
| `ep<NN>_commands.csv` | 보낸 팔 명령 (sim 시각, chunk 위치 x, chunk 번호, 6 관절, 그리퍼 닫힘) |

## 7. Isaac Sim 과 다른 점

| | Isaac (`policy/policy_client_node.py`) | Gazebo (`ocean_sim2sim_policy`) | 이유 |
|---|---|---|---|
| 카메라 | wrist + third view 고정 | `cameras` 설정 (기본 wrist 만), 없는 카메라는 에러, `blank_images` (배관 시험만) | Gazebo 에 third view 없음, 손목만 정책 예정 |
| base_imu | 없음 (계획) | `base_imu` 옵션 (기본 끔), stage1 과 같은 40 ms 평균, t_obs 에 포함 | 움직이는 베이스 정책 대비 |
| 관절 이름·한계 | `teleop/ik_model.py` (URDF 파일) | `arm_joints` 설정 + `/robot_description` 의 `<limit>` | 레포 밖 파일 경로 없이, 팔 교체 가능 |
| `/sim/reset` | 실패면 에러 | "not ready" (첫 드론 이륙 전) 면 기다렸다 다시 (`grasp_demo.py` 와 같음) | Gazebo scene 은 첫 드론이 뜬 뒤에만 리셋 받음 |
| 리셋 답 | `drone_goal_base`, `drone_target.name` | `drone_pos_base`, `drone_target` (이름), `seed` | scene_node JSON |
| 결과 | 출력만 | 출력 + 결과 dict 에 `detail` (chunk 마다 추론 시간, 서버 `infer_ms`, 그리퍼 바뀐 시각) | 평가 보고서 |
| 평가용 고리 | 없음 | `run_episode(stop_check, trace)` (기본 None = Isaac 과 같은 동작) | `policy_eval.py` |
| 로그 | 한국어 | 영어 | 레포 코드 언어 |
| 판정 | 사람이 GUI 로 봄 | `policy_eval.py` 가 `GraspJudge` 로 (grasp_demo 와 같은 기준) | 자동 평가 |

그 외 (스레드, 짝짓기, 0.066 s, 25 Hz chunk, 60 Hz 보간, replan_steps, 그리퍼 0.5, 안전 한계, desync·보호 정지, auto-kill 규칙, 결과 키) 는 같다.

## 8. 오프라인 확인 (2026-11-03)

```bash
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
python3 -m pytest src/gazebo_ocean_sim2sim/ocean_sim2sim_policy/test      # 24 passed
```

- 단위 시험 (`test/test_client_offline.py`): 설정 검사, 짝짓기 (이미지 stamp 기준, 카메라 2 대면 늦은 쪽, 오래된 그리퍼·t_obs 이전 없음 에러),
  base_imu 구간 평균이 stage1 식과 같음 (key·state 모드, IMU 가 늦을 때 t_obs 앞당김, 빈 구간 에러), 검은 이미지, chunk 보간
  (칸 사이 직선, 0 이전·9 이후 유지, 그리퍼는 ⌊x⌋), Isaac 문서 5.3 칸 위치 예, 안전 한계, URDF 한계
- 가짜 sim (Python 으로 토픽·서비스 흉내, 레포 밖) + `fake_policy_server.py` 로 Gazebo 없이 (격리 ROS_DOMAIN_ID):
  `policy_client_node.py` 2 에피소드 (약 58 Hz 명령, 지연 0.15 s 에서 도착 약 5.5 칸), base_imu key (`float32[6]`)·state (`float32[13]`),
  `blank_images` (`observation/base_image uint8[480, 640, 3]`), 없는 카메라 에러, `policy_eval.py` 3 에피소드
  (닫기 → auto-kill → held 3/3), 사람 모터 정지 기다림 (held), 보호 정지 (stop protective_stop, 이벤트 기록)
- `grasp_demo.py` 를 `observe.py` 로 바꾼 뒤 옛 코드와 같은 가짜 입력에서 관측 dict·힘·TCP/드론 자세·armed 가 같음
- **Gazebo + 실제 정책 서버 통합 시험은 아직** (사용자가 함)

## 연결 시험 (2026-11-03, 지금 체크포인트 `pi05_ur5/full_first13_10k/9999`, third view 자리 = 검은 이미지)

카메라 2 대용 체크포인트라 성공률은 의미 없음 (배관 확인만). 잔잔한 물, PX4 드론 hover mid, 25 점 목록, 2 에피소드 × 30 s.

| 항목 | 결과 | Isaac Sim |
|---|---|---|
| 서버 + Gazebo GPU 메모리 | 9.9 GB / 16 GB | 정책 11.8 GB (Isaac GUI 포함) |
| 추론 왕복 (wall) | 중앙 0.125 s, 최대 0.151 s | 0.18~0.19 s |
| `/joint_command` | 30 s 에 1790 개 (약 60 Hz) | 60 Hz |
| 새 chunk 도착 칸 (중앙) | 2.2 칸 | 약 4 칸 |
| RTF (정책 실행 중) | **0.58** | 0.96 |
| 결과 | 0/2 (접근 실패), 그리퍼 열고 닫기 반복 (Isaac 에서도 본 약점) | |

- **시간 gap**: 클라이언트는 sim time 으로 돈다. Gazebo 가 RTF 0.58 로 느리면 같은 wall 추론 시간이 sim 에서는 짧아져
  (0.125 × 0.58 ≈ 0.07 s), chunk 가 Isaac 보다 일찍 (2.2 칸 vs 4 칸) 도착한다 → 정책 입장에서 지연이 Isaac 과 다름. 평가 때 맞출지 정할 것
- RTF 가 떨어지는 이유: 카메라 렌더링 + JAX 추론이 GPU 하나를 같이 씀 (정책 없이 호버 1.00, 드론을 잡고 있을 때 0.43~0.47)

## 추론 지연 (2026-11-03, 사용자 결정 A)

`policy_client.yaml` `latency: {mode: fixed, fixed_s: 0.185}`: 새 chunk 는 sim 시각 t_obs + 0.185 s 부터 쓴다 (먼저 오면 기다리고,
늦게 오면 `late_chunks` 로 셈). `mode: measured` = Isaac 클라이언트 그대로 (도착하면 바로).

- 이유: 클라이언트는 sim time 으로 도는데, Gazebo 는 로봇이 드론에 닿으면 RTF 가 0.35~0.5 로 떨어진다. 추론은 wall 0.13 s 라
  sim time 지연이 0.05~0.13 s 로 흔들리고 Isaac (약 0.18 s) 보다 짧아져, sim2sim gap 에 지연 차이가 섞인다
- 근거: 시뮬레이션 평가에서 실제 지연을 RTF 로 sim 시간으로 바꿔 고정 지연으로 넣는 방식 (Reflex 2026), action chunk 정책을 고정
  추론 지연으로 평가하고 지연이 결과를 크게 바꿈 (Real-Time Chunking, Physical Intelligence 2025)
- 0.185 s = Isaac Sim GUI 와 같이 잰 왕복 (`docs/policy_inference_desktop.md`). 실물에서 잰 값으로 바꾸거나 여러 값으로 민감도를 볼 수 있음

| | measured (이전) | fixed 0.185 s |
|---|---|---|
| 새 chunk 를 쓰기 시작하는 칸 (중앙) | 2.2 (RTF 0.58) / 3.4 (RTF 1.0) | 4.8~4.9 (RTF 와 무관) |
| 늦은 chunk | | 0~1 / 에피소드 (첫 추론) |

RTF 가 떨어지는 원인 (2026-11-03 측정): 정책 추론이 아니라 **로봇과 드론의 접촉 계산**. Gazebo + 정책 서버 + 클라이언트는 RTF 1.00 이고,
정책이 팔을 움직여 드론에 닿은 뒤 0.64 → 0.35 (드론을 잡고 있을 때 0.43~0.47 과 같은 현상). 이전에 본 0.43 은 남아 있던 서버 프로세스 탓
