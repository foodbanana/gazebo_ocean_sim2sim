# 에피소드 씬: 잡을 드론

작성: 2026-11-02 · 패키지 `ocean_sim2sim_gazebo` (드론 모델), `ocean_sim2sim_interface` (`scene_node`)

PX4 SITL 드론을 붙이기 전까지 쓰는 **임시 드론**과 에피소드 서비스. Isaac Sim 드론 파지 씬과 같은 드론·위치·서비스를 쓴다.

---

## 1. 실행

`ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py` 에 포함 (`scene_node` + `scene_bridge`).
시작 후 보트가 자리를 잡으면 (3 s) 다시 5 s 동안 로봇 베이스 위치를 평균해서 드론을 띄운다 (sim 약 8 s).

```bash
ros2 service call /sim/drone_kill std_srvs/srv/Trigger     # 드론 모터 정지 (잡은 뒤)
```

| 서비스 | 동작 |
|---|---|
| `/sim/drone_kill` (std_srvs/Trigger) | 드론을 앵커에서 떼어 떨어뜨림 (= 모터 정지). message = JSON `{"sim_time": ...}`. 이미 떨어져 있으면 `success=False` |
| `/sim/reset` | 5-2 단계에서 추가 |

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

## 6. 알아 둘 것

- **`DetachableJoint` 는 `<static>` 모델에 붙지 않는다** (드론이 고정되지 않고 흔들림). VRX 의 정박 플랫폼처럼 world 에 fixed joint 로 고정한 일반 모델을 쓴다
- **ros_gz_bridge 의 `Pose_V` → `TFMessage` 변환은 엔티티 이름을 버린다** (`child_frame_id` 가 빈 문자열) → 보트 위치는 `OdometryPublisher` 로 받는다
- Gazebo 로그 `Mesh construction from an SDF has not been implemented yet for dartsim` 은 디버그 메시지 (메시 충돌은 동작)
