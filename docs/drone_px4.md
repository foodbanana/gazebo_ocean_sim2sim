# PX4 SITL 드론 (Isaac Sim 과 같은 연결·추력 모델)

작성: 2026-11-03 · 패키지 `ocean_sim2sim_drone`

Isaac Sim (`data_collection_ur5_gripper` `docs/drone_flight.md` 13·14장) 의 PX4 드론을 Gazebo 로 옮겼다.
PX4 v1.16.0 실제 펌웨어가 드론을 날리고, MAVLink 연결·가상 센서·추력 모델·PX4 파라미터는 Isaac Sim 코드를 그대로 옮겼다.

## 1. 구조

```text
 Gazebo (물리 240 Hz)                              px4_bridge_node (Python venv)                 PX4 SITL v1.16.0
 DroneLockstep 플러그인 (C++)  ── TCP 4570 ──▶  2 스텝마다 (120 Hz) 상태 받음         ── TCP 4560 ──▶ EKF2 (모션캡처)
   상태 보내고 응답까지 기다림 ◀── 몸체 힘·토크 ──  가상 센서 (IMU 자세 차분, 기압, 지자기,   lockstep     위치→자세→각속도
   (lockstep), 매 스텝 적용                          모션캡처 100 Hz·20 ms·1 mm/0.3°)          ◀── 모터 u ──
                                                    모터 u → ω = 1000u + 100 → 지연 → kω²
 scene_node: /sim/reset, /sim/drone_kill ──────▶   PX4Commander (offboard 20 Hz)        ── UDP 14540 ─▶
                                                    CLI drone_cmd.py (UDP 14600)
```

| 파일 | 역할 | 출처 |
|---|---|---|
| `src/DroneLockstep.cc` | Gazebo 시스템: 1/120 s 마다 상태 보내고 응답 (힘·토크 6 + 프로펠러 관절 속도 4) 까지 기다림, 매 스텝 적용 (속도 = JointVelocityCmd, `<rotor_joints>`) | 새로 |
| `ocean_sim2sim_drone/flight.py` | 추력 모델 (한 스텝 지연, 모터 지연 12.5/25 ms, kω², 반토크, 공기저항) | Isaac `drone_flight.py` |
| `ocean_sim2sim_drone/sensors.py`, `pegasus_geo_mag.py` | 가상 센서 (같은 seed 면 Isaac 과 값이 같음) | Isaac `px4_sensors.py` (Pegasus, BSD-3) |
| `ocean_sim2sim_drone/px4_link.py` | PX4 실행·재시작, MAVLink HIL lockstep, PX4 파라미터 | Isaac `px4_bridge.py` |
| `ocean_sim2sim_drone/commander.py` | Offboard 목표·arm·kill, 사인파 hover (small/mid/big) | Isaac `drone_cmd.py`, `drone_flight.py` |
| `scripts/px4_bridge_node.py` | ROS 노드: 서비스 `/drone/px4/{restart,arm,kill,status}`, `/drone/px4/reference_cmd`, `/drone/px4/state` | 새로 |
| `config/drone_px4.yaml` | 포트, 모션캡처·EKF2 파라미터, hover 프리셋 (값마다 출처 주석) | Isaac `px4_sitl.yaml`, `drone_iris.yaml` |

드론 물리값 (질량 0.8 kg, 무게중심, 관성, 로터 위치·방향, 추력 상수) 은 Isaac Sim PhysX 에서 꺼낸
`ocean_sim2sim_gazebo/models/iris_075/meshes/collision_isaac/isaac_flight.yaml` 하나를 Gazebo 모델·브리지·PX4 파라미터가 같이 쓴다.
PX4 파라미터: `CA_ROTORi_PX/PY` = 로터 위치, `MC_ROLLRATE_K` 0.2456, `MC_PITCHRATE_K`·`MC_YAWRATE_K` 0.1484 (Isaac 로그와 같음).

## 2. 실행

```bash
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash   # venv: README
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py                   # drone_mode:=px4 drone_motion:=hover hover_preset:=mid (기본)
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py drone_mode:=fixed # PX4 없이 world 고정 드론 (예전)
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py prop_spin:=false   # 프로펠러 멈춤 (Isaac headless 기본). 기본 true = Isaac 규칙으로 돎
python3 src/gazebo_ocean_sim2sim/ocean_sim2sim_drone/scripts/drone_cmd.py status   # goto x y z | hold | land | kill
```

## 3. 공중 시작 (리셋)

바다에는 바닥이 없어 Isaac Sim 처럼 바닥에서 이륙하지 않는다 (2026-11-03 사용자 결정 C). `/sim/reset`:

1. PX4 armed 면 kill → 드론·앵커 지움 → 팔 시작 자세
2. 드론을 새 위치에 앵커로 고정해 띄움
3. `/drone/px4/restart`: PX4 새로 시작 (lockstep 연결 약 2 s) + EKF 대기 5 s (Isaac 과 같음)
4. `/drone/px4/arm`: PX4 원점을 실제 위치에 맞추고 Offboard·arm (약 2 s)
5. 목표를 10 cm 위로 → PX4 이륙 램프로 추력이 오름 → **총추력 = 무게 (7.85 N) 인 순간 앵커를 놓고** 목표를 원래 위치로
6. 안정 5 s (Isaac 과 같음) → 드론 움직임 시작 (hover mid, seed = 리셋 seed + 1,000,000, Isaac 과 같음)

| 항목 | Gazebo | Isaac Sim |
|---|---|---|
| 리셋 시간 (sim) | 22.6 s | 24 s |
| 놓은 직후 | 높이 −23 mm, 앞으로 +10 cm 쏠렸다 약 4 s 에 돌아옴 | (바닥 이륙) 앞으로 최대 20 cm 쏠렸다 3 s 에 돌아옴 |
| 안정 뒤 위치 오차 | 축마다 7~10 mm | 6~7 mm |
| 호버 RMS (5 s 이후) | 14~24 mm | 19.5 mm |
| RTF | 1.00 | 0.96 |

- 놓는 순간을 일찍 (무게의 90 %, 목표 +3 cm) 하면 높이가 −70 mm 까지 떨어져 더 나빴다
- 앞쪽 쏠림: 앵커에 묶인 동안 PX4 자세 적분이 쌓이는 것으로 보임 (놓기 직전 앞뒤 로터 속도 차이가 기하 균형보다 큼)

## 4. 파지 데모 (잔잔한 물, drone_motion static = Isaac grasp_demo, 2026-11-03)

| 케이스 | Isaac Sim (PX4) | Gazebo (PX4) | 기대대로 |
|---|---|---|---|
| success | held. 밀림 9.2 mm, 접촉력 27 N, 내려앉음 1.0 mm, 회전 0.9° | held. 밀림 10.6 mm, **48 N**, 0.02 mm, 0.2° | 맞음 |
| offset_y | held. 밀림 4.2 mm, 0.3 mm, 0.5° | held. 3.5 mm, 2.1 mm, 0.4° | 맞음 |
| offset_x | held. 밀림 4.4 mm, 1.4 mm, 1.7° | held. 10.3 mm, 2.2 mm, **7.0°** | 맞음 |
| empty | no_grasp_empty (64.1°) | no_grasp_empty (65.0°) | 맞음 |
| miss | no_grasp_empty (드론이 32 mm 밀려 빠짐) | **잡힘** (33°, 밀림 19.9 mm) → held | 틀림 |

- 잡은 채 모터 1 s: 드론 흔들림 0.03 mm (Isaac 0.03 mm), 손가락 힘 흔들림 0.16 N (Isaac 0.05 N)
- 고정 드론일 때와 달리 드론이 밀린다 (밀림 3.5~20 mm). miss 는 Isaac 보다 덜 밀려 (20 vs 32 mm) 몸체 아랫부분이 잡힘 → gap
- 접촉력이 Isaac 보다 크다 (33~48 N vs 27 N)

## 5. 보트 IMU (`/base/imu`)

VRX 기본 WAM-V IMU 그대로 (사용자 결정 2026-11-03): `wamv/base_link` 기준 (0.3, −0.2, 1.3), 100 Hz, VRX 잡음 (자이로 0.009 rad/s 등),
`sensor_msgs/Imu` 비력 (정지 = +9.81). Isaac Sim 학습 데이터 (`/base/imu` 120 Hz, 잡음 없음, 로봇 base 강체) 와 주기·잡음·위치가 다르다 → gap.
frame_id 는 Gazebo 이름 (`wamv/wamv/imu_wamv_link/imu_wamv_sensor`, Isaac `imu_link`).

`/base/odom` (`nav_msgs/Odometry`, Isaac Sim `ros2_iface.yaml` base 와 같은 형식, scene_node): 30 Hz, stamp = sim time 1/30 s 배수,
frame `odom` = 에피소드 시작 (리셋이 끝난 순간) 의 로봇 base 자세, child `robot_mount` (Isaac Stewart 의 "윗판 가운데 = 로봇 base 자리" 와 같은 뜻),
위치 R0ᵀ(p − p0), 자세 R0ᵀR, twist 는 child 좌표계. 보트 odometry (50 Hz) 가 올 때마다 지난 1/30 s 시각을 빠짐없이 냄 (직전 값 유지).
확인: 30.0 Hz, 간격 33.33 ms 고정, 파도에서 상하 71 mm.

## 6. 해 보고 고친 것

- **rclpy MultiThreadedExecutor 가 서비스 대기 중 빈 루프를 돌아 GIL 을 차지** → lockstep 이 10 배 느려짐 (RTF 0.07). EventsExecutor 로 1.00
- **Gazebo 시간은 정수 ns**: 1/240 s 두 번이 1/120 s 와 같지 않아 가끔 한 번 건너뜀 (12.5 ms) → 플러그인이 스텝 수를 셈
- **arm 직후 바로 놓으면 떨어짐**: PX4 가 착지 상태로 보고 공회전 (ω 101 rad/s) → 이륙 램프로 추력을 올린 뒤 놓음
- 이전 시험의 Gazebo 가 남아 두 서버가 돌면 모든 것이 느려지고 컨트롤러가 안 뜸 → 종료 확인

## 7. 프로펠러 링크·회전 뒤 확인 (2026-11-03)

드론을 Isaac Sim 구조 (몸체 + 로터 링크 4 개 + 회전 관절, 로터 충돌, 링크별 질량) 와 Isaac 메시·색 (빨간 테이프는 정확히) 으로 바꾼 뒤:

- 호버 RTF 1.00 (프로펠러 회전 켬·끔 같음). **잡고 있는 동안 RTF 0.43~0.47** — 이전 한 링크 모델도 0.47 로 같음 (새 모델 탓 아님).
  손가락 메시 ↔ 드론 볼록 조각 32 개 접촉 계산 (Isaac Sim 은 잡은 채 0.69). lockstep 이라 결과는 같고 시간만 더 걸림
- 잔잔한 물 success·offset_y held, offset_x slip (모터 정지 뒤 회전 10.4° > 10°)
- **모터 정지 뒤 드론이 두 손가락 접촉점을 잇는 축 둘레로 기우는 일이 잦다**: 0.12 s 에 10° 까지 가속하며 기울기도 함. 실행마다 1~10° (기준 10° 경계).
  로터 충돌을 빼도, 관절을 고정해도, 이전 한 링크 모델에서도 생김 (고정 드론에서 5.5°·10.7° 기록) → 모델 변경 탓이 아니라
  선 접촉 둘레 비틀림을 마찰이 버티지 못하는 접촉 처리 차이로 봄 (Isaac Sim 0.9~1.7°). gap 으로 기록, 원인은 더 확인 필요
- 판정기: 링크마다 접촉 센서가 같은 토픽 `/drone/contacts` 로 내므로 `grasp_demo.py` 가 같은 시각 메시지를 합침

## 8. 모터 정지 뒤 드론이 기우는 원인 (2026-11-03 조사)

**기록 (잔잔한 물, success, 원래 메시 손가락 + Bullet)**: 모터 정지 순간부터 드론이 손가락 축 (드론 y) 둘레로 돌다가 1~3 s 뒤 −6.8° 에서 멈춤
(넘어지는 것이 아니라 새 균형). 잡고 있는 동안 오른손가락 끝 (r2) 이 드론 볼록 조각에 **최대 4.8 mm 파고들고** (왼쪽 0.06 mm),
멈춘 뒤 r2 접촉점이 파지면 앞 끝 (x ≈ +12 mm, 파지면 ±13 mm) 에 몰림 → 손가락 끝 모서리를 축으로 무게중심 (x ≈ 0) 이 12 mm 어긋나 기움.

**확인 실험 (각 3 회, Gazebo 하나만 떠 있는 것 확인)**

| 조건 | 모터 정지 뒤 기울기 | 결과 | 잡은 동안 RTF |
|---|---|---|---|
| E0 지금 (원래 메시 손가락, Bullet) | 2.3°, 7.0°, 11.6° | held 2, slip 1 | 0.24~0.37 |
| E1 볼록 손가락 (`collision_meshes:=convex`), Bullet | 0.3°, 1.3° | held 2, 그리퍼가 멈추지 않아 grasp_timeout 1 | 0.70~0.76 |
| E2 원래 메시 손가락, **FCL** 충돌 검출 | 0.05°, 3.3°, 2.3° | **held 3** | **1.00** |
| (Isaac Sim) | 0.9~1.7° | | 0.69 |

→ **원인: Bullet 의 "메시 손가락 ↔ 볼록 드론 조각" 접촉 계산** (깊게 파고들고 접촉점이 파지면 가장자리로 몰림). 충돌 검출기나 손가락
충돌 형상을 바꾸면 크게 줄고, 접촉 계산이 가벼워져 RTF 도 오른다. 바꿀지는 다른 시험 (그리퍼 힘·마찰 G1~G4, 손가락끼리 충돌, 보호 정지,
파도 RTF) 을 다시 돌려 보고 정함. Bullet 을 고른 이유는 ODE 가 자기 충돌을 켜면 RTF 0.4 였기 때문 (FCL 은 그때 시험 안 함)

### 충돌 검출기 비교 (2026-11-04, DART 물리 엔진 안에서 검출기만 바꿈, 임시 world 복사본) → **Bullet 유지 (사용자 결정)**

물리 엔진은 계속 DART (`<physics type="dart">`). 바꿀 수 있는 것은 그 안의 충돌 검출기 (`<dart><collision_detector>`): bullet | fcl | ode | dart.

| 항목 | **bullet (지금)** | fcl | dart (DART 자체) | Isaac Sim |
|---|---|---|---|---|
| E 빈손 닫기 | 1.134 rad (손가락끼리 안 닿음) | 1.118 rad (닿음) | 1.134 rad (안 닿음) | 1.121 rad |
| G3 손가락별 수직력 (r2 / l2) | 26.6 / 26.6 N | 43~46 / 86~96 N (비대칭) | 접촉 없음 | 22~25 N |
| G3 미끄럼 힘 | 9.4~9.5 N | 9.5~9.8 N | (잡지 못함) | 19.1 N |
| G1 0.2 kg 미끄러짐 | 0.07 mm | 2.43 mm | | < 1 mm |
| G4 0.5 kg 미끄러짐 / 관절 떨림 | 0.14 mm / 0.0001° | 8.15 mm / 2.06° | | 박스 떨림 < 0.1 mm |
| H 0.8 / 0.96 / 1.2 kg 미끄러짐 | 0.19 / 0.23 / 0.28 mm | 15.4 / 2.8 / 12.5 mm | | |
| 보호 정지 (충돌·관절 점프·리셋 복구) | 동작 | 동작 | 접촉 판정 불가 | |
| PX4 파지 데모 (5 케이스) | 4/5 | 5/5 | | 5/5 |
| miss 케이스 | 20 mm 밀린 뒤 잡힘 | 32 mm 밀려 빠짐 (Isaac 과 같음) | | 32 mm 밀려 빠짐 |
| 모터 정지 뒤 기울기 | 1~10° | 0.01~3.0° | | 0.5~1.7° |
| 드론을 잡은 동안 RTF | 0.24~0.43 | 1.00 | | 0.69 |
| 정지 RTF | 1.00 | 1.00 | 0.42~0.51 | |

- **fcl**: 드론 (메시 손가락 ↔ 볼록 조각) 은 Isaac 에 더 가깝지만, 박스 (메시 ↔ 기본 도형) 에서 좌우 손가락 힘이 비대칭이고 미끄러짐·떨림이 수십 배 → 쓰지 않음
- **dart**: 메시를 지원하지 않음. `[DARTCollisionDetector] Attempting to check for an unsupported shape pair: [BoxShape] - [MeshShape]. Returning false.`
  가 몇 분에 수십만 번 (상자-메시, 원통-상자, 상자-원통, 원통-원통) → 손가락이 박스를 통과해 끝까지 닫힘, 접촉 센서 0 개. 에러 로그 때문에 RTF 0.42~0.51
- **ode**: 자기 충돌을 켜면 RTF 0.4 (처음 비교, 2026-11-02)
- 파도 (gain 0.3) 의 스크립트 데모 접근 실패 (오차 52~54 mm) 는 검출기와 무관 (bullet·fcl 같음, 스크립트 제어기 한계)
- Bullet 의 드론 기울기·접촉 중 RTF 저하는 gap 으로 기록 (원인: 위 8 장)
