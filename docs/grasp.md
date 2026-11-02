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
| 충돌 형상 | 모든 메시 충돌체 `convexHull`, 팔뚝은 convex decomposition (32 조각, 64 버텍스, 오차 2 %) | 같은 방식으로 만든 메시 (`collision_meshes:=convex`, 1-1 장) | 같은 형상 |

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

## 1-1. 충돌 형상 (Isaac Sim 과 같게)

PhysX 는 움직이는 링크의 삼각형 메시로 직접 충돌하지 않는다. Isaac Sim 로봇 USD 는 모든 메시 충돌체에
`physics:approximation = "convexHull"` 을 쓰고 (`isaacsim/assets/robots/ur5_rh_p12_d435i/payloads/instances.usda`),
팔뚝만 convex decomposition 으로 바꾼다 (`isaacsim/config/drive_gains.yaml` `collision_shapes`).
Gazebo (DART + Bullet) 는 메시를 그대로 쓰므로 같은 형상을 미리 만들어 둔다.

| 링크 | 형상 | 결과 |
|---|---|---|
| 그리퍼 base, r1, r2, l1, l2, 손목 마운트 판 | 메시의 볼록 껍질 | 예: base 496 → 196 면, r2 152 → 64 면 |
| UR5 base, shoulder, upper arm, wrist 1~3 | 메시의 볼록 껍질 | |
| UR5 forearm | CoACD convex decomposition (최대 32 조각, 조각당 64 버텍스, 오목도 0.02) | 9 조각 (CoACD 가 오차 안에서 더 적게 나눔) |

- 만드는 스크립트: `ur5_rh_p12_description/scripts/build_collision_hulls.py` → `meshes/collision_convex/`,
  `config/ur5_visual_parameters_convex.yaml` (ur_description 의 visual params 에서 충돌 메시만 바꾼 것)
- 인자: `ur5_rh_p12_d435i.urdf.xacro` `collision_meshes:=original|convex` (팔 xacro 기본 original = 원본 메시,
  Isaac Sim 이 import 하는 그대로). Gazebo 는 `wamv_with_arm.urdf.xacro` 가 `convex` 를 먼저 선언
- 팔뚝 조각 수: Isaac Sim 의 32 는 최대값. PhysX (V-HACD) 가 실제로 몇 조각을 만들었는지는 확인하지 않음
- UR 메시 라이선스: `meshes/collision_convex/ur5/LICENSE_ur_description` (BSD-3-Clause)

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
| H | 60 × 160 × 100 mm, 0.8 / 0.96 / 1.2 kg, 3 s 유지 (드론 무게 기준) |

## 3. 결과와 gap (기본값: r1 2.28 Nm, 마찰 0.6, 충돌 형상 convex)

| 항목 | Isaac Sim | Gazebo convex (기본) | gap | (참고) Gazebo 원본 메시 |
|---|---|---|---|---|
| E 빈손 닫기 | 손가락끼리 닿아 1.121 rad | **1.118 rad** (손가락끼리 닿음, r1 토크 2.29 Nm) | −0.2° | 1.134 rad (손가락끼리 안 닿음) |
| 손가락당 파지면 수직력 (G3 박스) | 22~25 N | 26.2~26.5 N (접촉 센서) | +10 % | 26.6 N |
| **G3 미끄럼 힘** | **19.1 N** | **9.3~9.4 N** (3 회) | **−51 %** | 9.4~9.5 N |
| 유효 마찰 (= 미끄럼 힘 / 수직력 합) | 0.386 (설정 0.6) | **0.18** (설정 0.6) | Gazebo 가 절반 | 0.18 |
| **G1 0.2 kg 정육면체 3 s** | 미끄러짐 < 1 mm, 떨림 < 0.1 mm | **미끄러짐 3.6~4.0 mm**, 박스 떨림 2.9~3.8 mm, 관절 떨림 0.3~2.8° (5 회) | 실패 | 0.07 mm, 0.10 mm, 0.0006° |
| G4 0.5 kg 5 s | 관절 떨림 < 0.01°, 박스 떨림 < 0.1 mm | 미끄러짐 0.12 mm, 박스 0.23 mm, 관절 0.005° | 박스 떨림 2 배 | 0.14 mm, 0.22 mm |
| H 0.8 kg 3 s | | 미끄러짐 0.20 mm, 박스 떨림 0.37 mm | | 0.19 mm |
| H 0.96 kg 3 s | | 0.22 mm, 0.43 mm | | 0.23 mm |
| H 1.2 kg 3 s | | 0.27 mm, 0.51 mm | | 0.28 mm |
| 팔 홈 자세 유지 | | 0.0000 rad | | |
| RTF | | 1.00 | | 1.00 |

- **충돌 형상이 바꾼 것**: 빈손 닫기 (손가락끼리 충돌, Isaac Sim 과 0.2° 차이) 와 작은 정육면체 G1
- **바꾸지 않은 것**: 미끄럼 힘 (−51 %), 큰 박스 유지 (G4, H). 미끄럼 힘 gap 은 충돌 형상 때문이 아니다
- **G1 (convex) 의 움직임**: 접촉 힘은 원본 메시와 비슷 (손가락 끝 링크 r2·l2 각 27~29 N) 이고 손가락은 0.779 rad 에서
  멈춤 (원본 0.765). 앵커를 푼 직후 박스가 **옆으로 (TCP y, 파지 방향·공구 축에 수직) 3.7 mm** 밀렸다가
  3 s 동안 0.7 mm 까지 천천히 돌아옴. 아래로 빠지는 미끄러짐이 아니다.
  볼록 껍질 파지면과 작은 박스 면의 접촉점이 Bullet convex-convex 에서 한쪽으로 치우친 것으로 보이며, 원인은 확인하지 않음.
  Isaac Sim 은 같은 형상으로 G1 을 통과하므로 **엔진 접촉 처리의 gap** 으로 기록한다

**드론 무게와의 여유**: 드론 0.8 kg = 7.85 N. Gazebo 미끄럼 힘 9.3 N 은 무게의 **1.2 배** (Isaac Sim 19.1 N 은 2.4 배).
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
- **원본 메시에서는 손가락끼리 충돌이 감지되지 않는다**: 기하로는 1.12 rad 에서 닿는데 Gazebo 는 토크 0 으로 1.134 까지 닫힘.
  볼록 껍질 (Isaac Sim 과 같은 형상) 에서는 1.118 rad 에서 닿아 멈춘다 → 기본값 convex
- **충돌 형상은 미끄럼 힘에는 영향이 없고, 작은 물체 (G1) 유지에는 영향이 크다** (3 장). `collision_meshes:=original` 로 비교 가능
- 물리 엔진 bullet-featherstone (mimic 제약 지원) 은 그리퍼가 0.29 rad 에서 멈추고 접촉력도 안 나와서 쓰지 않았다
- 시작할 때 컨트롤러 로드 응답이 10 s 를 넘으면 spawner 가 재시도하다 실패 (`already loaded`) → launch 에서 `--service-call-timeout 60`
