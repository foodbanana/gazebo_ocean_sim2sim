# 텔레옵 (PICO VR 팔 · Radiolink 조종기 드론) 과 녹화

작성: 2026-11-04 · 패키지 `ocean_sim2sim_teleop` (팔 텔레옵·녹화), `ocean_sim2sim_drone` (조종기)

Isaac Sim (`data_collection_ur5_gripper`) 의 두 텔레옵과 녹화 도구를 Gazebo 로 옮겼다. 식·값·버튼·검사 규칙은 Isaac 과 같다.
Isaac 쪽 설명: `docs/pico_vr_data_collection.md` (PICO), `docs/drone_teleop.md` (조종기).

## 1. 구조

```text
 PILOT                                          ARM OPERATOR
 Radiolink transmitter                          PICO 4 Ultra controllers
        | radio                                        | headset -> Wi-Fi -> PC (XRoboToolkit RoboticsServiceProcess)
        v                                              v
 receiver -> Raspberry Pi Pico                  xr_teleop_node.py (ocean_sim2sim_teleop, XRoboToolkit venv python)
        | USB serial (RC,<10 values>, 142.9 Hz)        |  /joint_command, /gripper/command  ----------------+
        v                                              |  /teleop/request (A record, B reset, X drone_kill) |
 px4_bridge_node (ocean_sim2sim_drone)                 v                                                    |
   rc_input -> drone_rc -> PX4Commander         record_toggle.py (rosbag2)  <------ state topics ----------+
        | UDP 14540 MANUAL_CONTROL                     |  /sim/reset, /sim/drone_kill -> scene_node         |
        v                                              v                                                    v
 PX4 SITL (POSCTL: stick = velocity)            bags: <bag dir>/<YYYYMMDD_HHMMSS>_<task>/       interface_node -> arm, gripper
```

| 파일 | 역할 | Isaac 원본 |
|---|---|---|
| `ocean_sim2sim_drone/rc_input.py` | Pico 직렬 읽기, 값 검사, 스틱 환산 (−1 ~ 1, dead zone) | `isaacsim/scripts/rc_input.py` |
| `ocean_sim2sim_drone/drone_rc.py` | 넘기기·되돌리기 판단 (bridge 교환마다) | `isaacsim/scripts/drone_rc.py` |
| `ocean_sim2sim_drone/commander.py` | `set_sticks`·`handover`·`takeback`·`set_params` (MANUAL_CONTROL, PARAM_SET) | `isaacsim/scripts/drone_cmd.py` |
| `ocean_sim2sim_drone/config/rc_input.yaml` | 축 배치·범위, 넘길 때 PX4 파라미터 | `isaacsim/config/rc_input.yaml` |
| `ocean_sim2sim_teleop/xr_teleop.py`, `scripts/xr_teleop_node.py` | 컨트롤러 자세 → One Euro 필터 → 차분 IK → `/joint_command` | `teleop/xr_teleop_node.py` |
| `ocean_sim2sim_teleop/ik_model.py` | URDF 체인 `robot_mount` → `rh_p12_rn_tcp` FK·Jacobian·IK 한 스텝 | `teleop/ik_model.py` |
| `ocean_sim2sim_teleop/config/xr_teleop.yaml` | 배율·필터·버튼·안전 기준 | `teleop/xr_teleop.yaml` |
| `scripts/record_toggle.py`, `config/recorder.yaml` | rosbag2 녹화 토글 (키 r·k·d·q, 컨트롤러 A·B·X) | `record_toggle.py` (`--sim --base moving`) |
| `scripts/plot_bag_imu.py` | bag 의 `/base/imu`·`/base/odom` 그래프 (가속도, 각속도, roll·pitch·yaw 변화 (첫 값 기준), base 위치)·통계·CSV | 새로 |
| `scripts/check_xr_teleop.py`, `ocean_sim2sim_drone/scripts/check_rc.py` | 헤드셋·조종기 없이 시험 | `teleop/check_xr_teleop.py`, Isaac 가짜 직렬 시험 |

## 2. 처음 한 번만

- **PICO**: XRoboToolkit PC 서비스 (deb) 와 Python 바인딩 venv (`~/xrobotoolkit/venv`, `--system-site-packages`) 는 Isaac 레포의
  `docs/pico_vr_data_collection.md` 2 장대로 설치 (이 PC 에는 있음). 노드는 지금 python 이 `xrobotoolkit_sdk` 를 못 읽으면
  `$XR_PYTHON` 또는 `~/xrobotoolkit/venv/bin/python` 으로 스스로 다시 실행된다
- **조종기**: 계정을 `dialout` 그룹에 (`sudo usermod -aG dialout $USER` 뒤 다시 로그인). sim 을 sudo 로 띄우지 않는다.
  장치 확인 (sim 없이): `ros2 run ocean_sim2sim_drone rc_watch.py` (장치 자동, 첫 줄에 고른 장치) → 스틱을 움직이면 값이 바뀐다

## 3. 실행 (터미널마다 `source /opt/ros/jazzy/setup.bash && source ~/vrx_ws/install/setup.bash`, sim 터미널은 venv 도)

```bash
# 터미널 1: sim. 드론을 조종기로: drone_motion:=rc (기본 hover = 조종자 없이 사인파)
source ~/vrx_ws/.venv/bin/activate
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py drone_motion:=rc
#   Pico 장치는 자동으로 찾는다 (rc_input.yaml port: auto = /dev/serial/by-id 에서 이름에 usb-MicroPython_Board 가 든 것 하나).
#   없거나 둘 이상이면 찾은 목록과 함께 에러. 직접 주려면 rc_port:=/dev/ttyACM0. 파도: wave_gain:=..., 잔잔한 물: world:=empty_ocean_calm

# 터미널 2: PICO 팔 텔레옵 (PC 서비스 실행 중, 헤드셋 앱 WORKING · 오른손·왼손 Send)
pgrep -af RoboticsServiceProcess || env -u LD_LIBRARY_PATH bash /opt/apps/roboticsservice/runService.sh
ros2 run ocean_sim2sim_teleop xr_teleop_node.py

# 터미널 3: 손목 영상
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw

# 터미널 4: 녹화 (텔레옵 노드를 띄운 뒤)
ros2 run ocean_sim2sim_teleop record_toggle.py <작업명>          # bag: 이 레포의 ocean_gazebo_bags/ (git 제외, --bag-dir 로 바꿈)

# 녹화 뒤 IMU 보기
ros2 run ocean_sim2sim_teleop plot_bag_imu.py ~/vrx_ws/src/gazebo_ocean_sim2sim/ocean_gazebo_bags/<bag 이름> [--show] [--csv]
```

### 버튼·키 (Isaac 과 같음)

| 입력 | 하는 일 |
|---|---|
| **Y** (왼손) | 텔레옵 시작 / 멈춤. 누른 순간의 오른손 컨트롤러 자세가 기준 (앞 = 로봇 base +x, 위 = +z) |
| **A** (오른손) | 녹화 시작 / 종료. 리셋 뒤에만 시작 |
| **B** (오른손) | 에피소드 리셋 (`/sim/reset`). 녹화 중이면 실패로 보고 `_discarded/` 로 옮긴 뒤 리셋 |
| **X** (왼손) | 드론 모터 정지 (`/sim/drone_kill`) |
| 트리거 | 그리퍼 (0.6 넘게 = 닫기, 0.4 아래 = 열기) |
| 키 `r` / `k` / `d`→`y` / `q` (터미널 4) | 리셋+녹화·종료 / 모터 정지 / 방금 에피소드 버리기 / 종료 |

순서: **B** (리셋) → **Y** (텔레옵) → 잠깐 자세 유지 → **A** (녹화) → 잡기 → **X** (모터 정지) → **A** (녹화 끝) → 다음은 **B** 부터.
리셋하면 텔레옵은 스스로 멈춘다 (다시 Y).
sim 을 켠 직후에는 팔 컨트롤러가 늦게 켜질 수 있다 (spawner 의 첫 응답이 사라지면 60 s 기다림, 터미널의 `Configured and activated
gripper_controller` 줄). 그 전의 B / `r` 은 거절되고 (`arm controllers not up yet ... nothing changed`), 드론은 그대로 있다
(2026-11-04 전에는 리셋이 드론을 먼저 지운 뒤 실패해 드론이 사라졌다). 녹화 중에는 새 ROS 노드 (`ros2 topic echo`, rqt) 를 띄우지 않는다.

### 조종기 (드론)

리셋 (공중 시작) 이 끝나고 네 스틱이 가운데 근처 (0.1 안) 면 조종기에 넘어간다 (PX4 Position 모드: 스틱 = 속도, 놓으면 그 자리).
왼쪽 스틱 = 상승·하강 / yaw, 오른쪽 스틱 = 드론 기준 앞뒤·좌우. 스틱 끝 속도 수평 0.5 m/s, 수직 0.5 m/s, yaw 60 °/s.
신호가 끊기거나 (조종기 끔) 줄이 0.2 s 안 오면 그 자리 Offboard 호버로 되돌리고, 돌아오고 가운데면 다시 넘긴다.
조종기에는 모터 정지 스위치가 없다 → X 또는 `k`. 직렬 장치가 뽑히면 그 실행 동안 조종기를 끄고 드론은 호버 (Isaac 은 에러로 멈춤).

### 보조 화면 (Side view · Top view)

손목 영상만으로는 그리퍼와 드론의 앞뒤·좌우·높이를 맞추기 어렵다 (Isaac 2026-10-23 과 같은 이유) → Gazebo GUI 오른쪽에 두 화면.
Isaac `drone_scene.py` `GUI_AUX_VIEWS` 와 같은 위치·화각 (`ocean_sim2sim_description/urdf/aux_views.xacro`):

| 화면 | 카메라 (로봇 base 기준) | 보이는 것 |
|---|---|---|
| Side view | (0.40, −1.60, 0.64) m, 로봇 오른쪽에서 +y 쪽, 3.6° 아래 | 화면 오른쪽 = 로봇 앞. 그리퍼와 드론의 앞뒤 거리·높이 차 |
| Top view | (0.30, 0, 2.44) m, 82.8° 아래 | 화면 위 = 로봇 앞, 오른쪽 = 로봇 오른쪽. 드론이 그리퍼 바로 위인지 |

- 가로 화각 47.2°, 512 x 288, 15 Hz (`aux_views_rate`). 카메라는 로봇 base 에 붙어 보트와 같이 움직인다 (파도에도 그리퍼·드론 상대 위치가 그대로 보임)
- **조작자용**: 녹화하지 않고 정책 입력도 아니다 (gz 토픽 `/aux_view/side`, `/aux_view/top`, ROS 로 bridge 하지 않음)
- 켜기·끄기: `aux_views:=` 비움 (기본) = GUI 면 켬, headless 면 끔. `aux_views:=false` 로 끈다 (Isaac `--aux-views`)
- RTF (headless 로 카메라만 켜서 잼, 잔잔한 물, 드론 호버): 끔 1.00, 켬 1.00 (Isaac 은 0.98 → 0.82)

## 4. 녹화하는 것

`config/recorder.yaml` (Isaac `--sim --base moving` 과 같은 토픽, 손목 카메라만):

| 토픽 | 내용 |
|---|---|
| `/joint_states`, `/gripper/joint_states`, `/gripper/target` | 팔·그리퍼 상태, 그리퍼 action |
| `/joint_command`, `/gripper/command` | 팔 action (텔레옵 노드), 그리퍼 명령 |
| `/cam/wrist/color/image_raw`, `camera_info` | 손목 카메라 (RELIABLE) |
| `/base/imu` | 보트 IMU (로봇 base 옆 (−0.15, 0, 1.3), 100 Hz, VRX 잡음) |
| `/base/odom` | 로봇 base 위치·속도 (에피소드 시작 = 원점) |
| `/protective_stop`, `/clock` | 보호 정지, sim 시간 |

- 드론 (위치, 조종기 입력) 은 녹화하지 않는다 (Isaac 결정 2026-10-08). `episode.json` 에 리셋 응답 (seed, 드론 기준 위치, 공중 시작 단계 시간) 과 모터 정지 시각
- 녹화 시작 전에 상태 토픽마다 발행자가 있는지 확인한다 (없으면 녹화하지 않음)
- 저장할 때마다 그 bag 폴더에 `base_imu.png` (그래프), `base_imu.csv`, `base_odom.csv` 를 자동으로 만든다 (뒤에서, 창 없이. `--no-imu-plot` 로 끔). 창으로 보려면 `plot_bag_imu.py <bag> --show`
- bag 은 레포 최상위 `ocean_gazebo_bags/` 에 쌓인다 (`.gitignore`, Isaac 레포의 `bags/` 와 같음). 자동으로 지우지 않는다 (`df -h` 확인)

## 5. Isaac 과 다른 점

| 항목 | Isaac Sim | Gazebo | 이유 |
|---|---|---|---|
| 조종기 입력을 넣는 곳 | sim 프로세스 `DroneFlight.check` (앱 루프 30 Hz) | `px4_bridge_node` 교환마다 (120 Hz) → `MANUAL_CONTROL` 도 120 Hz | Gazebo 에서는 bridge 가 `PX4Commander` 를 가진다 |
| 넘기기 시작 시점 | 이륙 끝 (`cli_ready`) | 공중 시작 끝 (scene 이 마지막 `reference_cmd` 에 `ready: true`) | 앵커에서 들어 올리는 동안 넘기지 않게 |
| 리셋 중 텔레옵 | 팔 순간이동 → 측정이 목표와 달라 (desync) 스스로 멈춤 | interface 의 `/ocean_sim2sim/hold_state` (true = 리셋 중) 로 멈춤 | Gazebo 리셋은 팔을 2 s 에 걸쳐 옮기고, 앞섬이 제한된 관절 목표가 따라가 desync 가 안 걸렸다 (처음 시험 9 FAIL) |
| URDF | 빌드한 파일 | `/robot_description` (보트 + 팔, 같은 체인) | 로컬 경로 없이 |
| 로봇 base | 테이블 위 고정 (Stewart 움직이는 베이스 있음) | 보트 위 (파도에 움직임) | 손이 가만히 있어도 팔은 보트와 같이 움직인다 (2026-11-04 사용자 결정: 그대로) |
| 알림 반복 | 숫자만 바뀐 알림도 매 루프 다시 찍음 | 숫자를 뺀 글이 같으면 5 s 마다 | 시험 중 수백 줄 |
| 직렬 장치가 뽑힘 | 에러 | 조종기를 끄고 드론은 그 자리 호버 | sim 을 멈추지 않게 |
| 녹화 도구 | 실물·sim 공용, `--base fixed|moving` | sim, 움직이는 베이스만, 토픽은 설정 파일 | Gazebo 는 항상 보트 |

## 6. 시험 결과 (2026-11-04, headless)

### 조종기 (`check_rc.py`, 가짜 Pico = pty, `drone_motion:=rc`, 잔잔한 물) → 12/12 PASS

Isaac 의 가짜 직렬 시험 (`sim_ros2.py --drone-rc on`, 10/10) 과 같은 항목 + 줄 끊김.

| 시험 | Gazebo | Isaac (`docs/drone_teleop.md` 8) |
|---|---|---|
| 공중 시작 뒤 자동 넘김 | 넘김 (리셋 끝난 즉시, PARAM_SET 확인 뒤) | 넘김 |
| 넘긴 뒤 10 s 위치 | 28 ~ 30 mm | 최대 30 mm |
| 스틱 +0.6 을 2 s: 앞 / 오른쪽 / 위 / yaw | 185 ~ 193 mm / 228 ~ 249 mm / 361 ~ 377 mm / −39.6 ~ −40.2° (놓으면 0.02 m/s 아래) | 229 / 278 / 362 mm / 38.8° |
| 조종기 끔 (상태 값 1, 1) | 0.11 s 뒤 되돌림, Offboard 확인 0.1 ~ 1.1 s (모드는 1 Hz heartbeat), 3 s 동안 11 ~ 28 mm | 0.2 s 안에 되돌림 |
| 신호 돌아옴 + 가운데 | 다시 넘김 | 다시 넘김 |
| Pico 줄이 끊김 | 0.22 s 뒤 되돌림 (stale 0.2 s), 줄이 다시 오면 다시 넘김 | (시험 없음) |
| 스틱을 민 채 리셋 | 넘기지 않음 + 알림, 가운데 두면 넘김 | 같음 |
| 수동 모드에서 kill | disarm | 같음 |

### PICO 텔레옵 (`check_xr_teleop.py`, 만든 컨트롤러, 잔잔한 물, 드론 hover mid) → 13/13 PASS

| 시험 | 결과 |
|---|---|
| 1 끝을 80° 든 채 시작 | 거부 (명령 0) |
| 2 시작 | 명령 나감, 손을 안 움직이면 TCP 0.00 mm |
| 3 앞 10 cm / 4 위 8·오른쪽 5 cm | TCP [+90, 0, 0] / [+90, −45, +72] mm (배율 0.9, 오차 < 0.1 mm), 이동 중 명령 − 측정 최대 1.3° |
| 5 왼쪽 20° / 5b 손목 ±25° | +20.00° (위치 0.00 mm) / 25.0°·25.0°, 보호 정지 0 |
| 6 그리퍼 | 1150 → 1150 (히스테리시스) → 0 |
| 7 명령 간격 | 중앙값 16.66 ms (60 Hz), stamp 0·역행 0, 40 ms 넘은 것 0 |
| 7b 필터 | 손 떨림 1 mm·0.3° → 명령 TCP 떨림 0.41 mm (필터 끔 0.93 mm, 0.43 배) |
| 8 멈춤·다시 시작 | 멈춘 동안 명령 0, 손 30 cm 옮긴 뒤 다시 시작해도 TCP 0.00 mm |
| 9 리셋 | 텔레옵 스스로 멈춤 (`hold_state`), 리셋 뒤 명령 0, 팔 시작 자세 0.000° |
| 10 보호 정지 | 0 번 |
| 11 버튼 | A → record, B → reset, X → drone_kill 한 번씩 |

`hold_state` 를 넣기 전에는 9 가 FAIL (리셋 뒤에도 텔레옵 중, 팔이 옛 손 위치로 22° 돌아감) → 5 장.
IK 자기 점검 (Gazebo URDF): Jacobian 과 수치 미분 차이 5e-7, 무작위 목표 50/50 수렴.
홈 자세 TCP 가 Isaac URDF 와 2 mm (공구 축 0.5°) 다르다 (두 URDF 의 그리퍼·마운트 원점 차이. 기준이 시작 순간의 TCP 라 텔레옵에는 영향 없음).

### 녹화 (`record_toggle.py`, 파도 gain 0.3)

녹화 도구를 pty 로 띄워 키와 `/teleop/request` 를 넣음 → 11/11:
`r` → 리셋 (22 ~ 25 s sim) 뒤 녹화, `k` → 모터 정지 (시각이 `episode.json` `drone_kill` 에), `r` → 저장 (metadata.yaml 정상),
리셋 없이 A → 거부, B → 리셋만, A → 녹화, X → 모터 정지, A → 저장, `d` `y` → `_discarded/` 로 이동, `q` → 종료.

10.5 s 에피소드의 bag: `/base/imu` 1011 개 (100.0 Hz), `/base/odom` 302 개 (30 Hz), 손목 영상·camera_info 각 약 30 Hz,
`/joint_states` 약 110 Hz, `/clock`, `/gripper/*`, `/protective_stop` (텔레옵 노드가 없어 `/joint_command` 는 없음).
`plot_bag_imu.py` 그래프: 파도 5 s 주기의 pitch ±0.9° 와 상하 ±0.1 m, 자이로는 VRX 잡음 (표준편차 약 0.5 °/s) 이 대부분.

- `/base/imu` 는 평균 **100 Hz** 지만 간격이 8.33 ms 와 12.5 ms 로 섞인다: Gazebo 는 센서를 물리 스텝 (240 Hz = 4.17 ms) 에서만
  내보낸다 (10 ms 가 스텝의 배수가 아님). 실제 IMU 는 일정한 10 ms. 정확히 10 ms 간격이 필요하면 물리 200 Hz 같은 배수 주기가 필요 (지금은 그대로)
- 녹화기가 토픽을 찾는 데 토픽마다 0.2 ~ 1.7 s 가 걸려 그 전 메시지는 bag 에 없다 (Isaac 과 같은 현상, Isaac `--trim-idle`·`--skip-start` 로 처리).
  A 를 누른 뒤 1 ~ 2 s 지나고 움직이기 시작하면 된다
- 녹화 도구는 바쁜 동안 (리셋·종료·모터 정지) 과 그 뒤 0.3 s 에 온 컨트롤러 요청을 버린다 (Isaac 과 같음): `[RESET] done` 을 보고 다음 버튼
