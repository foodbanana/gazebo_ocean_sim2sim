# 드론 파지 데모와 판정 (Isaac Sim 과 같은 순서·기준)

작성: 2026-11-03 · `ocean_sim2sim_bringup/scripts/grasp_demo.py`, `config/grasp_demo.yaml`,
`ocean_sim2sim_interface/grasp_judge.py`, `ocean_sim2sim_interface/kinematics.py`

Isaac Sim `data_collection_ur5_gripper` `isaacsim/scripts/grasp_demo.py` + `grasp_judge.py` + `config/grasp_demo.yaml`
(2026-10-22 결과) 를 Gazebo 로 옮긴 것. 같은 순서로 드론을 아래에서 잡고, 같은 기준으로 판정한다.

## 1. 실행

```bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm
ros2 run ocean_sim2sim_bringup grasp_demo.py --case all        # success offset_y offset_x empty miss
```

- 케이스마다 `/sim/reset` → 드론은 `drone_pos_base` (0.6, 0, 0.738) = Isaac Sim 기본 위치. 데모 동안 scene 의
  `use_drone_targets`·`drone_offset` 을 끄고 끝나면 되돌린다 (`--drone targets` 면 scene 목록·오프셋 그대로)
- 결과: `~/ocean_sim2sim_reports/grasp_demo_<시각>/` (`report.txt`, `result_<케이스>.json`, `timeline_<케이스>.csv`,
  `commands_<케이스>.csv`)

## 2. 순서 (Isaac Sim 과 같음)

| 단계 | 내용 |
|---|---|
| S0 | 관절 보간으로 home_pose (Isaac Sim 시작 자세) |
| S1 | 관절 보간으로 seed_pose → IK 직선 이동으로 잡는 자세 15 cm 아래 |
| S2 | 3 cm/s 수직 상승. 남은 높이 3 cm 까지 드론을 따라감 |
| S3 | 그리퍼 닫기 → grasped 판정 뒤 1 s (모터 도는 채로 잡고 있음) |
| S4 | `/sim/drone_kill` → 2.5 s 버팀 → held 판정 |
| S5 | 2 s 유지 |

- 명령은 정책과 같은 길만 쓴다: `/joint_command` (50 Hz, 한 주기 0.063 rad 이하), `/gripper/command`, `/sim/reset`, `/sim/drone_kill`
- 잡는 위치 = Isaac Sim 계획기가 구한 TCP (드론 좌표계 [0, −1.3, −4.5] mm, 그리퍼 x = 드론 −x). 다시 계산하지 않고 그대로 씀
- 판정만 정책이 받지 않는 값을 쓴다: `/drone/odometry` (드론 몸체 자세), `/drone/contacts` (링크별 접촉력, 드론 32 조각 전체 접촉 센서)
- IK: `kinematics.py` (robot_description 에서 robot_mount → rh_p12_rn_tcp 사슬, 팔 교체 가능), 감쇠 최소제곱 (0.02).
  보트가 움직이므로 매 주기 로봇 베이스 기준으로 목표를 다시 구함. FK 는 Isaac Sim 기록과 같음 (홈 자세 TCP (0.487, 0.109))

## 3. Isaac Sim 과 다르게 한 것

| 항목 | Isaac Sim | Gazebo | 이유 |
|---|---|---|---|
| IK 오차 | **측정** 관절각에서 (오차를 명령에 계속 더함 = 적분, 중력 처짐 보상) | **명령** 관절각에서 (`ik_feedback: command`) | Isaac Sim 데모는 관절 목표를 drive 에 바로 넣음 (지연 거의 0). Gazebo 는 정책과 같은 인터페이스 (30 Hz 루프, 약 60 ms) 를 거쳐 측정값 IK 가 ±20 mm 로 흔들려 5 케이스 모두 드론에 부딪힘 |
| 관절 상태 | 물리 스텝마다 | `/joint_states` (120 Hz) + `/gripper/joint_states` (약 30 Hz) = 정책이 받는 것 | 240 Hz 전체 관절 토픽은 파이썬 노드가 최대 200 ms 늦게 처리 |
| 드론 | 비행 제어기로 호버 (밀리면 움직임) | world 에 고정 (모터 정지 = 떼어 냄) | PX4 드론 전까지 (scene.md) |
| drop 케이스 | 모터 정지 순간 그리퍼 힘 0.05 Nm | 없음 | Gazebo 는 실행 중 effort 한계를 못 바꿈 |
| empty 케이스 | 드론을 옮기고 기본 위치를 향해 잡음 | 같음 (데모가 리셋 전에 scene `drone_pos_base` 를 기본 + 오프셋으로 바꿨다 되돌림) | |

## 4. 해 보고 고친 것

- **그리퍼가 열림 한계 (정확히 0) 에 놓이면 닫히지 않는다** (DART, 닫힘 한계와 같은 문제). 리셋이 그리퍼를 0.0 으로 열어서
  닫기 명령에 1 s 동안 0° 그대로였다 → 인터페이스 `command_margin` 을 양쪽 한계에, scene 리셋은 `gripper_open` 0.001 rad 로 연다
- 측정값 IK 의 흔들림, 관절 상태 지연 (3 장)

## 5. 결과 (잔잔한 물, 1 회씩)

드론 충돌 형상 = **Isaac Sim PhysX 가 실제로 쓰는 볼록 조각** (scene.md 2 장, 2026-11-03). 그 전 (CoACD 조각) 결과는 아래 참고.

| 케이스 | Isaac Sim (2026-10-22) | Gazebo (Isaac Sim 조각) | 기대대로 | (참고) Gazebo CoACD 조각 |
|---|---|---|---|---|
| success | held. 닿는 각도 16.0°, 접촉력 27 N, 내려앉음 1.0 mm, 회전 0.9° | **held**. 18.9°, 37 N, 내려앉음 2.6 mm, 회전 1.6° | 맞음 | held, 회전 5.5° |
| offset_y (10 mm) | held. 0.3 mm, 0.5° | **held**. 2.2 mm, 1.6° | 맞음 | slip |
| offset_x (5 mm) | held. 1.4 mm, 1.7° | **held**. 2.2 mm, 0.4° | 맞음 | slip (회전 10.7°) |
| miss (4 cm 낮게) | no_grasp_empty (잡는 동안 드론이 32 mm 밀려 손가락이 끝까지 닫힘) | **잡힘** (24°) → 13 mm 미끄러짐 → slip | 틀림 | 잡힘 → slip |
| empty | no_grasp_empty (64.1°) | **no_grasp_empty** (65.0°) | 맞음 | 접근 실패 (그때는 다른 방식) |

- **드론 충돌 형상이 결과를 크게 바꿨다**: 같은 형상을 넣자 어긋나게 잡는 두 케이스가 실패 → 성공, 모터 정지 뒤 회전 5~11° → 0.4~1.6°
- **miss**: Isaac Sim 은 드론이 날고 있어 손가락에 밀려 빠져나감 (32 mm). Gazebo 드론은 world 에 고정이라 밀리지 않고 몸체 아래쪽
  (폭 41 mm, z = −30 mm) 이 잡힌다 → **고정된 드론 때문의 gap** (PX4 드론에서 다시 볼 것)
- 잡고 있는 동안 (모터 도는 채) 접촉력이 크고 흔들림: 평균 35~48 N, 표준편차 7~22 N (Isaac Sim 27 N ± 0.05). 닫는 순간 최대 140~220 N.
  고정된 드론을 손가락이 미는 것 → 고정 드론 gap
- 내려앉음 2.2~2.6 mm (Isaac Sim 0.3~1.4 mm)

### 드론 충돌 형상의 폭 (손가락 폭 |x| ≤ 13 mm 안, 잡는 방향)

| 높이 (드론 좌표계 z) | Isaac Sim PhysX 조각 (지금 Gazebo) | CoACD 조각 (이전) | 원본 메시 |
|---|---|---|---|
| 0 | 87.6 mm | 87.3 mm | 80.5 mm |
| −20 mm | 84.8 mm | 80.1 mm | 75.9 mm |
| −30 mm | 41.4 mm | 50.2 mm | 35.8 mm |

Isaac Sim 계획기 (잡는 높이 계산) 는 원본 메시로 폭 81.1 mm 를 썼지만, 시뮬레이션은 87.6 mm 조각과 충돌한다 (닿는 각도 예측 20.3° vs 실제 16.0°).

### 이전 결과 (CoACD 조각, 2026-11-03 오전)

| 케이스 | Gazebo |
|---|---|
| success | held. 18.6°, 39 N, 내려앉음 1.4 mm, 회전 5.5° |
| offset_y | slip (마지막 0.5 s 0.5003 mm > 0.5), 회전 6.3° |
| offset_x | slip (회전 10.7° > 10°) |
| miss | 잡힘 (23°) → 17 mm 미끄러짐 |
| empty | 접근 중 도달 실패 (17 mm) |

## 6. 파도 (empty_ocean, 2026-11-03) — Isaac Sim 기준 없음

Isaac Sim 의 grasp_demo 는 고정 테이블에서만 돌았다 (움직이는 베이스는 기구학 분석 `reach_moving_base.py` 뿐). 그래서 여기는 비교가 아니라
Gazebo 에서 무엇이 되는지 확인. 기본 위치 5 케이스 0/5, 드론 위치 목록 10 회 (success) 0/10. RTF 1.00.

| 실패 | 횟수 | 내용 |
|---|---|---|
| 접근 도달 실패 (오차 2~6 mm) | 6 | 도달 기준 1.5 mm·0.5° (Isaac Sim 고정 베이스 값) 를 파도 속 목표에 못 맞춤: 팔이 명령을 약 60 ms 늦게 따라가는 동안 목표가 움직임 |
| 접근 도달 실패 (오차 22~180 mm, 자세 8~49°) | 5 | 큰 실패. 원인 아직 안 봄 (IK 가 관절 한계·특이점 근처, 또는 이동 중 목표를 못 따라감) |
| 잡은 뒤 미끄러짐 (모터 정지 뒤 16~17 mm) | 4 | 잡는 동안 접촉력 100~350 N: **world 에 고정된 드론**을 파도에 흔들리는 그리퍼가 잡고 있어 서로 끌어당김. 정지 순간 튕김 |
| 한쪽 손가락만 닿음 | 1 | 접근 중 팔이 드론에 부딪힘 (S1_seed, 106 N) |

→ 파도 속 파지는 고정 드론으로는 물리적으로 맞지 않는다 (실제 드론은 제어기로 버티며 밀림). 결정 필요 (보고 참고).

### PX4 드론, 파도 (2026-11-03, `--reach-tol 6 1.5`, drone_motion static)

드론이 날게 되자 고정 드론의 "서로 끌어당김 (100~350 N)" 은 사라졌지만, 스크립트 데모는 여전히 거의 실패한다.

| 시험 | 결과 | 실패 내용 |
|---|---|---|
| 기본 위치 5 케이스 | 0/5 | 모두 접근 중 도달 실패 (오차 17~58 mm) |
| 25 점 목록 success 10 회 | 1/10 held | 도달 실패 7 (오차 23~138 mm), 잡은 뒤 slip 2 (회전 12~20°, 잡는 동안 드론이 최대 148 mm 밀림) |

- 원인: 드론은 world 에 떠 있고 보트가 흔들려 로봇 기준 목표가 계속 움직이는데, 팔은 명령을 약 60~100 ms 늦게 따라간다.
  "목표 n mm 안에 들어가면 다음 단계" 라는 도달 판정은 Isaac Sim 고정 테이블용이라 기준을 완화해도 맞지 않는다 → **스크립트 제어기의 한계**
  (Isaac Sim 도 움직이는 베이스에서는 grasp_demo 를 돌리지 않았고 기구학 분석만 함)
- 파도 속 평가는 손목 카메라를 보고 움직이는 정책으로 한다 (6단계)
- RTF 0.73~0.78 (잡은 드론과의 접촉이 있는 동안)
