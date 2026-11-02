# 파지 물리: 그리퍼 힘·마찰과 Isaac Sim 과의 차이

작성: 2026-11-02 · Gazebo Harmonic (DART + Bullet 충돌 검출), gz_ros2_control 1.2.19

RH-P12-RN 그리퍼가 물체를 쥐는 힘과 미끄러짐을 Isaac Sim 과 비교한다.
기준 = Isaac Sim `data_collection_ur5_gripper` `isaacsim/scripts/grasp_tests.py`, `docs/PLAN.md` 1-6, `isaacsim/config/drive_gains.yaml`.

## 0. 원칙: 같은 입력값, 차이는 gap 으로 기록

목적은 **sim2sim gap 을 보는 것**이다. 그래서 물리 입력값은 Isaac Sim 과 **같은 값**을 넣고, 그래도 생기는 결과 차이를
보정해서 지우지 않고 **gap 으로 기록**한다. 같은 값을 넣을 수 없는 경우 (엔진에 없는 기능) 만 같은 구조·같은 의미가 되게 맞춘다.

| 항목 | Isaac Sim | Gazebo | 방식 |
|---|---|---|---|
| 그리퍼 힘 | 구동 관절 `rh_r1_joint` maxForce **2.28 Nm** (실물 400 mA) | `rh_r1_joint` effort 한계 **2.28 Nm** | 같은 값 |
| 손가락 결합 | 강체 mimic (r2·l1·l2 가 r1 을 따라감) | 따라가는 관절 effort 한계 1000 → r1 에 단단히 묶임 (1 장) | 같은 구조 |
| 마찰 | 손가락·물체 정지 **0.6** / 운동 0.5, 합치는 방식 min | 손가락·물체 **0.6** (DART 는 계수 하나, 합치는 방식 min) | 같은 값 (정지 마찰) |

## 1. 그리퍼 구조 (mimic)

DART 는 mimic 제약을 지원하지 않는다 (Gazebo 로그 `does not support mimic constraints`).
gz_ros2_control 이 대신 mimic 관절마다 속도 명령 `−(q_mimic − q_r1) × update_rate` 를 준다 (r1 의 **측정 위치**를 따라감,
`gz_ros2_control/src/gz_system.cpp`). DART 는 이 속도를 **그 관절의 URDF effort 한계까지만** 힘을 써서 지킨다.

- 처음 (모든 손가락 관절 같은 effort 한계): 물체를 사이에 두고 r1 과 l1 이 같은 힘으로 서로 밀어 **좌우 5° 비대칭**,
  원위 관절은 밀지 않아 파지면 힘이 절반, 당기면 손가락이 따로 돌아감
- 지금: `gripper_effort_limit` (r1) = 2.28, `gripper_mimic_effort_limit` (r2·l1·l2) = 1000 (원본 값)
  → 따라가는 관절이 r1 에 단단히 묶여 **좌우 대칭**, 쥐는 힘은 r1 하나 (2.28 Nm) 로 정해짐 = Isaac Sim 의 "구동 관절 하나 + mimic"
- 한 방향 결합: 따라가는 관절은 r1 을 따라가지만 r1 에 힘을 되돌려 주지 않는다 (Isaac Sim 강체 mimic 은 양방향)

인자 위치: `ur5_rh_p12_description/urdf/ur5_rh_p12_d435i.urdf.xacro` (기본 1000 = 원본, Isaac Sim 이 쓰는 값),
Gazebo 값은 `ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro` 가 먼저 선언 (`gripper_effort_limit` 2.28).
마찰은 `ur5_rh_p12_description/urdf/ur5_rh_p12.gazebo.xacro` `gripper_friction` 0.6, 드론은 `ocean_sim2sim_gazebo/config/iris_drone.yaml`.

## 2. 시험

```bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py world:=empty_ocean_calm   # 파도 0 + ApplyLinkWrench
ros2 run ocean_sim2sim_bringup grasp_test.py --tests E G3 G1 G4
```

- 팔 홈 자세 (공구가 아래), 박스를 임시 앵커로 손가락 사이에 고정 → 닫기 → 앵커 해제 → 마찰로만 버팀
- 박스 배치: 윗면이 TCP 에서 15 mm 공구 쪽 (`--top -0.015`) → **손가락 끝 파지면 (원위 링크)** 에 닿음 (Isaac Sim K3: 파지면 범위 안)
- 미끄러짐: 앵커를 푼 순간 TCP 에 있던 박스 위의 점이 **지금 TCP 좌표계**에서 움직인 거리 (잔잔한 물에서도 보트가 천천히 움직여서)
- 당김: 공구 축을 따라 손가락 밖으로 (아래로) 5 N/s 로 늘리는 힘 (`ApplyLinkWrench`), 1 mm 미끄러지는 순간의 힘 + 박스 무게

| 시험 | 내용 |
|---|---|
| E | 빈손으로 끝까지 닫기 |
| G3 | 60 × 160 × 100 mm, 0.8 kg 박스 당김 → 미끄럼 힘 |
| G1 | 40 mm 정육면체 0.2 kg, 3 s 유지 |
| G4 | 60 × 160 × 100 mm, 0.5 kg, 5 s 유지 |

## 3. 결과와 gap (기본값: r1 2.28 Nm, 마찰 0.6)

| 항목 | Isaac Sim | Gazebo | gap |
|---|---|---|---|
| 손가락당 파지면 수직력 (G3 박스) | 22~25 N | **26.6 N** (접촉 센서) | +10 % |
| **G3 미끄럼 힘** | **19.1 N** | **9.4~9.5 N** (3 회, ±0.05) | **−51 %** |
| 유효 마찰 (= 미끄럼 힘 / 수직력 합) | 0.386 (설정 0.6) | **0.18** (설정 0.6) | Gazebo 가 절반 |
| G1 0.2 kg 3 s | 미끄러짐 < 1 mm, 떨림 < 0.1 mm | 미끄러짐 0.07 mm, 박스 떨림 0.10 mm, 관절 떨림 0.0007° | 비슷 |
| G4 0.5 kg 5 s | 관절 떨림 < 0.01°, 박스 떨림 < 0.1 mm | 관절 0.0001°, 박스 0.22 mm, 미끄러짐 0.14 mm | 박스 떨림 2 배 |
| E 빈손 닫기 | 손가락끼리 닿아 1.121 rad | 1.134 rad (손가락끼리 충돌 안 됨, 5 장) | +0.7° |
| RTF | | 1.00 | |

**드론 무게와의 여유**: 드론 0.8 kg = 7.85 N. Gazebo 미끄럼 힘 9.4 N 은 무게의 **1.2 배** (Isaac Sim 19.1 N 은 2.4 배).
드론을 잡고 모터를 끈 뒤 파도·팔 움직임의 가속도까지 버틸지는 다음 시험 (드론 실제 파지) 에서 확인한다.

### 마찰계수에 따른 미끄럼 힘 (참고, 손가락·박스 같은 값, r1 2.28 Nm)

| 마찰 | 0.6 | 0.9 | 1.0 | 1.1 | 1.3 |
|---|---|---|---|---|---|
| 미끄럼 힘 [N] | 9.4 | 11.1 | 12.2 | 15.3 | 13.6 |

1.1 근처가 최대 (약 15 N). 그 이상에서는 박스가 미끄러지기 전에 손가락이 굴러 닫히며 함께 빠지는 경로가 먼저 생긴다.
Gazebo 에서는 마찰만으로 19.1 N 에 닿지 않는다 → 결과를 맞추는 보정 대신 같은 입력값 (0.6) 을 쓰고 차이를 기록했다.

## 4. 정지 마찰과 운동 마찰

PhysX 는 정지 (0.6, 미끄러지기 전) 와 운동 (0.5, 미끄러지는 중) 두 값을 쓰고, DART 는 하나다 (`mu`, `mu2` 는 두 **방향**의 값).
미끄러지기 시작하는 순간 (파지 성공 여부) 을 정하는 정지 마찰 0.6 을 썼다. 미끄러지는 중의 거동은 Gazebo 가 마찰을 그대로 유지해서 다르다.

## 5. 알아 둘 것 (해 보고 찾음)

- **gz_ros2_control mimic 은 "부모의 측정 위치 따라가기" + 그 관절의 effort 한계** (1 장). 모든 손가락에 같은 한계를 주면 비대칭·약한 파지
- **`ApplyLinkWrench` 의 persistent 힘은 누적된다** (새 메시지가 이전 값을 바꾸지 않고 더해짐, `persistentWrenches.push_back`).
  처음에 매번 전체 힘을 보내서 당김 시험 결과가 마찰과 상관없이 8~10 N 으로 나왔다 → 증가분만 보낸다
- **미끄러짐은 지금 TCP 기준으로** 재야 한다 (잔잔한 물에서도 TCP 가 0.5 s 에 0.2~0.3 mm 움직임)
- **박스를 너무 깊게 놓으면 근위 링크에 닿는다** (TCP 위 30 mm): 손가락 끝 파지면과 다른 곳으로 잡혀 힘·미끄러짐이 달라진다
- **손가락끼리 충돌이 감지되지 않는다**: 기하로는 1.12 rad 에서 닿는데 (메시·볼록 껍질 모두, `trimesh` + FCL 로 계산) Gazebo 는 토크 0 으로 1.134 까지 닫힘.
  메시끼리 충돌 자체는 된다 (메시 박스도 잡힘). DART 는 부모-자식 쌍만 빼는데 형제인 손가락끼리도 빠져 있음 → 원인 미확인.
  영향은 빈손으로 끝까지 닫을 때 (`/gripper/joint_states` 1149, Isaac Sim 약 1136) 뿐
- **충돌 형상은 미끄럼 힘에 영향이 거의 없었다**: 손가락을 볼록 껍질로 바꿔도 같음 (`gripper_collision_dir` 인자로 바꿔 볼 수 있음)
- 물리 엔진 bullet-featherstone (mimic 제약 지원) 은 그리퍼가 0.29 rad 에서 멈추고 접촉력도 안 나와서 쓰지 않았다
- 시작할 때 컨트롤러 로드 응답이 10 s 를 넘으면 spawner 가 재시도하다 실패 (`already loaded`) → launch 에서 `--service-call-timeout 60`
