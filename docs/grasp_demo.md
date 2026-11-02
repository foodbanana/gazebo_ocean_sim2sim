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
| empty 케이스 | 드론을 옮기고 기본 위치를 향해 잡음 | 드론은 기본 위치, 팔이 (드론 − 오프셋) 을 향함 | scene 리셋이 위치 지정을 안 받음 |

## 4. 해 보고 고친 것

- **그리퍼가 열림 한계 (정확히 0) 에 놓이면 닫히지 않는다** (DART, 닫힘 한계와 같은 문제). 리셋이 그리퍼를 0.0 으로 열어서
  닫기 명령에 1 s 동안 0° 그대로였다 → 인터페이스 `command_margin` 을 양쪽 한계에, scene 리셋은 `gripper_open` 0.001 rad 로 연다
- 측정값 IK 의 흔들림, 관절 상태 지연 (3 장)

## 5. 결과 (잔잔한 물, 2026-11-03, 1 회)

| 케이스 | Isaac Sim (2026-10-22) | Gazebo | 기대대로 |
|---|---|---|---|
| success | held. 닿는 각도 16.0°, 접촉력 27 N, 모터 정지 뒤 내려앉음 1.0 mm, 회전 0.9° | held. 18.6°, **39 N** (잡고 있는 동안 평균 49 N ± 6), 내려앉음 1.4 mm, **회전 5.5°** | 맞음 |
| offset_y (10 mm) | held. 내려앉음 0.3 mm | **slip** (마지막 0.5 s 동안 0.5003 mm 움직임 > 0.5), 회전 6.3° | 틀림 |
| offset_x (5 mm) | held. 회전 1.7° | **slip** (회전 10.7° > 10°) | 틀림 |
| empty | no_grasp_empty | 접근 중 도달 실패 (17 mm) — 3 장 다른 방식 때문, 다시 정해야 함 | 틀림 |
| miss (4 cm 낮게) | no_grasp_empty (손가락이 끝까지 닫힘) | **잡힘** (23°, 24 N) → 내려앉다 17 mm 미끄러짐 | 틀림 |

공통으로 보이는 gap:
- **드론 충돌 형상이 Isaac Sim 보다 두껍다**: 손가락 폭 (|x| ≤ 13 mm) 안에서 드론 몸체 폭이 Gazebo 조각 87 mm (z = 0),
  원본 메시 80.5 mm, Isaac Sim 계획기의 폭 81.1 mm. 4 cm 아래 (z = −30 mm) 는 조각 50 mm / 메시 36 mm → 닿는 각도가 작고
  (18.6° vs 16.0° 는 반대 방향이라 확인 필요), miss 가 잡힌다. CoACD 오목도 0.03 이 PhysX 분해보다 거칠다
- **고정된 드론**: 손가락이 밀어도 드론이 안 움직여 잡는 힘이 커지고 (39~70 N vs 27 N), 모터 정지 순간 튕기며 5~11° 돈다
