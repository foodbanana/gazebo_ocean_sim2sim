# 에피소드 씬: 잡을 드론

작성: 2026-11-02 · 패키지 `ocean_sim2sim_gazebo` (드론 모델), `ocean_sim2sim_interface` (`scene_node`)

**드론 (Iris 0.75 배)** 과 에피소드 서비스. `drone_mode:=fixed` = world 고정 호버 (이 문서), 기본 `drone_mode:=px4` = PX4 SITL 이 날림 (`docs/drone_px4.md`). Isaac Sim 드론 파지 씬과 같은 드론·위치·서비스를 쓴다.

---

## 1. 실행

`ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py` 에 포함 (`scene_node` + `scene_bridge`).
시작 후 보트가 자리를 잡으면 (3 s) 다시 5 s 동안 로봇 베이스 위치를 평균해서 드론을 띄운다 (sim 약 8 s).

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger          # 에피소드 리셋 (약 2.7 s)
ros2 service call /sim/drone_kill std_srvs/srv/Trigger     # 드론 모터 정지 (잡은 뒤)
```

| 서비스 | 동작 |
|---|---|
| `/sim/reset` (std_srvs/Trigger) | 에피소드 시작 상태로 (6 장). message = JSON |
| `/sim/drone_kill` (std_srvs/Trigger) | 드론을 앵커에서 떼어 떨어뜨림 (= 모터 정지). message = JSON `{"sim_time": ...}`. 이미 떨어져 있으면 `success=False` |

처음 드론이 뜨기 전 (sim 약 8 s) 에는 두 서비스 모두 `success=False` (`scene not ready yet`).

## 2. 드론 모델 (`ocean_sim2sim_gazebo/models/iris_075`)

Isaac Sim 이 실제로 쓰는 드론 (`drone_iris.yaml`) 과 같게: **3DR Iris 를 0.75 배**로 줄인 것.

| 항목 | Isaac Sim (`drone_iris.yaml`) | Gazebo |
|---|---|---|
| 외형 | Pegasus Iris (원래 PX4 Gazebo Iris) + 빨간 테이프 (`grasp_marker`) | **Isaac Sim 시각 메시 그대로** (몸체, 테이프 좌우, 프로펠러 4 개를 각 링크 좌표계 OBJ 로 꺼냄, `meshes/visual_isaac/`) |
| 색 | 몸체 Carpet_Gray, 앞 프로펠러 (rotor0·2) Linen_Blue (OmniPBR MDL), 뒤 프로펠러 (rotor1·3) 재질 없음, 테이프 UsdPreviewSurface (0.65, 0.02, 0.02) 거칠기 0.8 | 재질마다 단색 하나 (텍스처 없음). 메시에 UV 가 없어 Isaac Sim 은 텍스처를 uv (0, 0) 한 점에서만 읽음 → 그 색 × diffuse_tint: 몸체 (0.0162, 0.0165, 0.0170) 거칠기 1.0, 앞 프로펠러 (0.016, 0.070, 0.631) 거칠기 0.73, 뒤 프로펠러 = Isaac 기본 재질 회색 0.215 거칠기 0.5 (렌더링 비교로 확인), 테이프 같은 값 (`config/iris_drone.yaml`) |
| 크기 | 0.75 배: 전체 x −107~117, y −180~178 mm, 허리 폭 약 79 mm | 같음 (허리 폭 메시 78.4 mm) |
| 질량 | 몸체 0.78 + 프로펠러 4 × 0.005 = 0.8 kg (강체 5 개) | 같음: 링크 5 개, 링크마다 Isaac Sim PhysX 질량·무게중심·관성 (`isaac_flight.yaml` `links:`). 합 = Isaac 전체 값 (관성 차 < 1e-6 상대) |
| 프로펠러 | 강체 (충돌 있음), 관절 고정 (FixedJoint). `--prop-spin` (GUI 기본) 이면 revolute 로 두고 보여 주기용 회전: 추력 ≥ 0.1 N 100 rad/s, 0 초과 5 rad/s, 0 이면 정지 (× 방향) | 링크 `rotor0..3` (Isaac 로터 링크 위치·자세) + revolute `rotorN_joint` (추력 축, 한계·감쇠 없음). DroneLockstep 플러그인이 매 step 브리지가 준 속도를 JointVelocityCmd 로 (같은 규칙, 브리지 없으면 0 = 고정). launch 인자 `prop_spin` (기본 true). 충돌 = Isaac PhysX 프로펠러 볼록 껍질 (`rotorN_hull_00.stl`), 접촉 센서 `/drone/contacts` |
| 충돌 | 몸체+팔 메시 convex decomposition (PhysX) | **Isaac Sim PhysX 가 만든 조각 그대로** (32 조각, 조각당 버텍스 45 개 이하, `scripts/export_isaac_drone_collision.py` 로 꺼냄). 허리 폭 86.1 mm (x = 0 단면, 메시 78.4 mm). 이전: CoACD 32 조각 (`collision.source: coacd`) |
| 관성 | 형상으로 다시 계산 | Isaac Sim PhysX 값 (링크별) |

- 설정 `ocean_sim2sim_gazebo/config/iris_drone.yaml` → `scripts/build_iris_drone.py` 가 `model.sdf` 와 `meshes/collision/hull_*.stl` 생성
  (다시 만들 때만 `pip install trimesh scipy` 필요, 같은 설정이면 같은 결과)
- 충돌 조각 꺼내기 (Isaac Sim 드론 설정을 바꿨을 때만, Isaac Sim 필요):
  `~/isaacsim/python.sh ocean_sim2sim_gazebo/scripts/export_isaac_drone_collision.py --isaac-project ~/data_collection_ur5_gripper`
  → `models/iris_075/meshes/collision_isaac/` (`hull_*.stl`, `rotorN_hull_*.stl`, `isaac_flight.yaml`, 비교용 `raw_reference.stl`, `report.txt`)
  와 `meshes/visual_isaac/` (시각 메시 OBJ, `materials.yaml`). Isaac Sim 프로젝트의 드론 로더
  (`drone.add_drone`) 로 같은 USD·같은 덮어쓰기를 한 뒤 PhysX `request_convex_collision_representation` 으로 실제 조각을 받는다
- **왜 그대로 꺼냈나**: 직접 만든 CoACD 조각은 잡는 높이 폭은 비슷했지만 (87.3 vs 87.6 mm) 아래쪽 모양이 달라 (z = −30 mm 50 vs 41 mm)
  어긋나게 잡는 케이스가 실패했다. 같은 조각을 넣자 성공 (grasp_demo.md 5 장)
- 메시 출처·라이선스: `models/iris_075/README.md` (Pegasus / PX4, BSD)
- SDF 를 문자열로 생성하므로 메시 경로는 `model://iris_075/...`
- 처음에 Isaac Sim 의 데모용 간이 드론 (`drone_simple.yaml`, 박스·원통) 을 잘못 옮겼다가 바꿨다

## 3. 호버 = world 에 고정

- `drone_anchor`: world 에 fixed joint 로 고정된 빈 모델. 드론 모델의 `DetachableJoint` 가 드론 몸체를 여기에 붙인다
- 드론은 world 에 고정, **보트만 파도에 흔들린다** → 팔과 드론의 상대 위치가 실제 배처럼 흔들린다
- `/sim/drone_kill` → `/drone/detach` → 드론이 떨어진다 (잡고 있으면 무게가 그리퍼에 걸림)
- 손가락이 밀어도 드론은 밀리지 않는다 (실제 드론은 조금 밀림). PX4 드론으로 바꾸면 해결

## 4. 드론 위치

`ur5_rh_p12_interface.yaml` 의 `ocean_sim2sim_scene`:

| 파라미터 | 값 | 의미 |
|---|---|---|
| `drone_pos_base` | [0.60, 0.0, 0.738] | 로봇 베이스 (`robot_mount`) 기준, 수평 좌표 [m]. Isaac Sim `drone_pos` (world z − 테이블 0.762) |
| `spawn_delay` | 3.0 s | 시작 후 보트가 자리 잡는 시간 |
| `base_average` | 5.0 s | 로봇 베이스 위치를 평균하는 시간 (파도 한 주기) |

world 위치 = (평균 베이스 위치) + Rz(평균 베이스 yaw) · `drone_pos_base`, 드론은 수평 (yaw = 베이스 yaw).
베이스 위치 = 보트 world 위치 (`OdometryPublisher`, `/model/wamv/odometry`) + URDF 고정 관절 (`wamv/base_link` → `robot_mount`).
**평균하는 이유**: 드론은 world 에 고정되고 보트는 계속 흔들리므로, 한 순간의 베이스 위치로 정하면 파도가 그 순간 올려놓은 만큼 어긋난다
(한 번 3 cm 높게 놓임 → 손목 영상에서 드론이 60 px 위).

## 5. 확인

| 항목 | 결과 |
|---|---|
| 질량 | 0.800 kg (Isaac Sim 과 같음) |
| 고정 | 위치 그대로 (0.604, −0.000, 1.929) |
| kill | 떨어져 아래의 그리퍼 위에 걸림 (`under_drone` 자세였음) |
| 리셋 | 성공 (드론 지우고 목록 점에 다시 띄움) |
| RTF | 1.00 (Iris 메시 + 충돌 조각 32 개) |
| 손목 영상 (`under_drone` 자세) | Iris 를 아래에서 올려다본 모습: 가운데 몸체, X 자 팔 4 개, 팔 끝 모터, 프로펠러 날. 색은 Isaac Sim 이 그리는 색 (몸체 거의 검정, 앞 프로펠러 파랑, 뒤 프로펠러 회색, 아래쪽 빨간 테이프, 2026-11-03) |
| 위치 (이전 간이 드론으로 잼) | 드론 중심이 영상 중심 (332, 239) 근처, 파도로 ±15 px. 약 20 px (1.4 cm) 위 = 보트의 기본 피치 0.6° 로 팔 전체가 기울어서 |

## 6. 리셋 `/sim/reset`

Isaac Sim `sim_reset.EpisodeReset` 과 같은 순서. 한 번에 sim 약 2.7 s (Isaac Sim 은 PX4 재이륙까지 약 24 s).

1. 인터페이스 hold (`/ocean_sim2sim/hold`, std_srvs/SetBool): `/joint_command`·`/gripper/command` 는 검사만 하고 무시
2. 드론과 앵커를 지움 (잡고 있던 드론도). **먼저 떼고 `detached` 를 확인한 뒤** 지운다 (7 장)
3. 팔을 `init_pose` 로, 그리퍼를 열림으로 — `reset_arm_time` (2 s) 동안 부드럽게
   (Isaac Sim 은 순간이동. Gazebo 는 ROS 에서 관절을 순간이동시키는 방법이 없고, 컨트롤러 목표도 같이 바꿔야 해서)
4. 새 호버 위치 = 목록 점 + 축마다 균등분포 ±`drone_offset` (3 cm)
   - `use_drone_targets: true` (기본): `drone_targets_base` 25 점 (Isaac Sim `drone_targets_base`) 을 섞어 **한 바퀴씩** 고른다
   - `false`: `drone_pos_base` ± 오프셋
   - 섞는 순서 = `seed` 와 바퀴 번호, 오프셋 = `seed` + 리셋 번호 → **같은 seed = 같은 순서·위치** (`seed: -1` 이면 시작 시각)
5. 그 위치 (평균 로봇 베이스 기준, 4 장) 에 앵커 + 드론 생성, `attached` 확인
6. hold 해제, 그리퍼 목표 = 열림 (0)

실패하면 `success=False` + 이유, hold 는 켜진 채 (다음 리셋이 성공할 때까지 명령 무시, Isaac Sim 과 같음).

| `ocean_sim2sim_scene` 파라미터 | 기본값 |
|---|---|
| `init_pose` | Isaac Sim `under_drone` [−0.1889, −1.0776, 1.7304, −2.2242, 1.5700, −2.9566] |
| `reset_arm_time` | 2.0 s |
| `seed` | −1 (시작 시각) |
| `drone_offset` | 0.03 m |
| `use_drone_targets` | true |
| `drone_target_names`, `drone_targets_base` | 25 점 (이름 = base cm, 예 `x45_y-20_z70`) |

응답 예 (JSON):

```json
{"reset_count": 1, "seed": 7, "offset_seed": 7, "drone_target": "x45_y+00_z70",
 "drone_target_base": [0.45, 0.0, 0.7], "drone_offset": [-0.01057, -0.02095, 0.00906],
 "drone_pos_base": [0.43943, -0.02095, 0.70906], "drone_world": [0.43394, -0.02151, 1.89231], "drone_yaw": 0.0,
 "mean_robot_base": [...], "init_pose": [...], "drone_attached": true,
 "phases": {"remove_drone": 0.4, "arm_to_start": 2.308, "spawn_drone": 0.008},
 "sim_time": ..., "sim_s": 2.72, "wall_s": 2.757}
```

### 확인

| 항목 | 결과 |
|---|---|
| 리셋 3 번 | 모두 성공, 각 약 2.6~2.8 s. 팔 \|q − init_pose\| 0.000°, 그리퍼 열림, `/gripper/target` 0 |
| 리셋 중 명령 | 다른 자세·그리퍼 닫기를 60 Hz 로 보내도 무시 (리셋 뒤 0.000°, 목표 0) |
| kill 뒤 리셋 | 떨어진 드론을 지우고 새 위치에 다시 띄움 |
| 리셋 뒤 명령 | 다시 적용 (wrist_3 +0.100 rad) |
| 같은 seed (7) 로 3 번 실행 | 목표 점·오프셋 순서가 모두 같음 (x45_y+00_z70 → x55_y+20_z70 → x45_y−20_z80) |

## 7. 알아 둘 것

- **DetachableJoint 가 붙어 있는 드론을 지우면 Gazebo 가 죽는다** (segfault, DART `WeldJointConstraint::update` 가 지워진 몸체를 참조).
  떼라는 메시지를 보내고 `/drone/state` 가 `detached` 가 된 것을 확인한 뒤 지운다
- **앵커는 옮기지 않고 지웠다가 다시 만든다**: world 에 fixed joint 로 묶인 모델이라 위치 이동 서비스로 옮겨지지 않을 가능성이 커서
- **처음 드론을 띄우는 동안 리셋을 받으면 둘이 겹친다** → 처음 생성이 끝나 `attached` 를 확인한 뒤부터 리셋을 받는다

- **`DetachableJoint` 는 `<static>` 모델에 붙지 않는다** (드론이 고정되지 않고 흔들림). VRX 의 정박 플랫폼처럼 world 에 fixed joint 로 고정한 일반 모델을 쓴다
- **ros_gz_bridge 의 `Pose_V` → `TFMessage` 변환은 엔티티 이름을 버린다** (`child_frame_id` 가 빈 문자열) → 보트 위치는 `OdometryPublisher` 로 받는다
- Gazebo 로그 `Mesh construction from an SDF has not been implemented yet for dartsim` 은 디버그 메시지 (메시 충돌은 동작)
