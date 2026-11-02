# 에피소드 씬: 잡을 드론

작성: 2026-11-02 · 패키지 `ocean_sim2sim_gazebo` (드론 모델), `ocean_sim2sim_interface` (`scene_node`)

PX4 SITL 드론을 붙이기 전까지 쓰는 **임시 드론**과 에피소드 서비스. Isaac Sim 드론 파지 씬과 같은 드론·위치·서비스를 쓴다.

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
| `/sim/reset` (std_srvs/Trigger) | 에피소드 시작 상태로 (7 장). message = JSON |
| `/sim/drone_kill` (std_srvs/Trigger) | 드론을 앵커에서 떼어 떨어뜨림 (= 모터 정지). message = JSON `{"sim_time": ...}`. 이미 떨어져 있으면 `success=False` |

처음 드론이 뜨기 전 (sim 약 8 s) 에는 두 서비스 모두 `success=False` (`scene not ready yet`).

## 2. 드론 모델 (`ocean_sim2sim_gazebo/models/simple_drone`)

Isaac Sim `drone_simple.yaml` 과 같은 값 (`ocean_sim2sim_gazebo/config/simple_drone.yaml` 에 복사):

| 부분 | 크기 | 질량 |
|---|---|---|
| 몸체 박스 | 180 × 80 × 70 mm (폭 80 mm 방향으로 잡음, 그리퍼 최대 열림 약 107 mm) | 0.560 kg |
| 팔 4 | 몸체 옆면 → 모터 (±100, ±120) mm, 18 × 12 mm | 0.010 kg 씩 |
| 모터 4 | Ø28 × 22 mm | 0.040 kg 씩 |
| 프로펠러 원판 4 | Ø130 × 3 mm, z 48 mm | 0.005 kg 씩 |
| 다리 4 | Ø8 mm, 아래 끝 z −45 mm | 0.005 kg 씩 |
| **합** | | **0.800 kg**, 무게중심 z +8 mm |

강체 하나, 모든 도형이 시각·충돌 형상. 관성 = 도형별 (균일 밀도) 합.
yaml 을 바꾸면 `python3 ocean_sim2sim_gazebo/scripts/build_simple_drone.py` 로 `model.sdf` 를 다시 만든다.

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
| 질량 | 몸체 0.560 kg, 전체 0.800 kg (Isaac Sim 과 같음) |
| 고정 | 6 s 동안 위치 그대로 (0.626, −0.000, 1.955) |
| kill | 떨어져 상판 위로 (z 1.26) |
| 손목 영상 (`under_drone` 자세) | 드론 몸체·팔·모터가 가운데, 프로펠러 끝이 위에서 잘림 (Isaac Sim 기록과 같음). 몸체 중심 평균 (340, 218), 파도로 ±15 px. 영상 중심 (332, 239) 과 약 20 px (1.4 cm) 차이 = 보트의 기본 피치 0.6° 로 팔 전체가 기울어서 |

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
