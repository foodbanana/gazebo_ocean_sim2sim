# 손목 카메라 (D435i color)

작성: 2026-11-02 · Gazebo Harmonic (ogre2), ROS 2 Jazzy

정책 입력인 손목 카메라를 Isaac Sim 과 같게 만든다. 3 인칭 카메라는 쓰지 않는다 (바다에는 고정할 곳이 없음, 손목 카메라만으로 정책을 다시 학습).
기준 = Isaac Sim `data_collection_ur5_gripper/isaacsim/config/wrist_camera.yaml` (실물 D435i camera_info, 2026-09-30).

---

## 1. 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/cam/wrist/color/image_raw` | sensor_msgs/Image | 640 × 480 `rgb8`, frame_id `wrist_camera_color_optical_frame`, 약 30 Hz (4 장) |
| `/cam/wrist/color/camera_info` | sensor_msgs/CameraInfo | K = [618.551, 0, 332.276; 0, 618.551, 239.381; 0, 0, 1], plumb_bob, D = 0 |

```bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py
ros2 run rqt_image_view rqt_image_view /cam/wrist/color/image_raw
```

받는 쪽은 **RELIABLE** 로 받는다 (Isaac Sim 과 같은 이유: best effort 면 640 × 480 이미지가 조각 손실로 통째로 버려질 수 있음). 브리지 발행 큐 90 (= 3 s).

## 2. 파일

| 파일 | 내용 |
|---|---|
| `ur5_rh_p12_description/urdf/ur5_rh_p12.gazebo.xacro` | Gazebo 카메라 센서 (팔에 딸린 것이라 팔 패키지에) |
| `ur5_rh_p12_description/config/gz_bridge.yaml` | ros_gz_bridge 설정 (Gazebo → ROS 토픽 이름) |
| `ocean_sim2sim_description/urdf/wamv_with_arm.urdf.xacro` | 인자 `arm_gazebo_xacro` 로 위 센서 파일을 include |
| `ocean_sim2sim_bringup/launch/wamv_arm.launch.py` | 인자 `bridge_config` 로 브리지 실행 |

팔을 바꾸면 그 팔 패키지에 센서 xacro 와 브리지 yaml 을 만들어 `arm_gazebo_xacro:=` (xacro_args), `bridge_config:=` 로 넘긴다.

## 3. Isaac Sim 과 같게 한 것

| 항목 | Isaac Sim | Gazebo |
|---|---|---|
| 위치 | `wrist_camera_color_optical_frame` (t10 마운트) | 같은 프레임 (URDF 가 같음). 센서는 `wrist_camera_color_frame` (x 앞, z 위 = Gazebo 카메라 규약) 에 붙이고 frame_id 만 optical frame 으로 (`gz_frame_id`) |
| 해상도 | 640 × 480 | 같음 |
| 초점거리 | fx = fy = 618.551 (렌더러가 정사각 픽셀만 → fy 를 fx 로, 실물 fy 618.956) | 같음 (`<lens><intrinsics>`) |
| 주점 | cx 332.276, cy 239.381 | 같음 |
| 왜곡 | 없음 | 없음. 렌즈 타입 `gnomonical` (핀홀) 을 명시 (SDF 기본값은 stereographic) |
| clip | 0.01 ~ 10 m | 같음 |

**확인**:
- URDF 로 TCP (손가락 사이 파지점) 를 카메라에 투영하면 (332, 445). Isaac Sim 문서 (t10) 의 (332, 446) 과 같다
- `under_drone` 자세 (Isaac Sim 시작 자세) 에서 그리퍼를 닫으면 손가락 끝이 화면 아래 가운데, 투영한 TCP 자리에서 만난다.
  열면 손가락이 화면 좌우 아래 끝에 조금만 보인다 (Isaac Sim 기록과 같음)
- 영상은 stamp 시각의 물리 상태 (0 프레임 지연, Isaac Sim check_ros2 5 번): 잔잔한 물에서 손목을 돌리면 움직이기 시작한 뒤
  **첫 이미지**에서 바뀐다 (그 전 프레임 사이 차이 0.000~0.005, 첫 이미지 6.24)
- 카메라를 켜도 RTF 1.00

## 4. 프레임 시점 (Gazebo 가 정함)

`update_rate` 30 으로 두면 프레임이 **8 물리 스텝 간격에 가끔 7 스텝** (약 12 장에 한 번) 으로 나와 평균 **약 30.3 Hz** 다.

| `update_rate` | 실제 | 프레임 간격 [스텝] |
|---|---|---|
| 30 | 30.3 Hz | 7, 8 |
| 30.0000048 (= 정확히 8 스텝 주기 33,333,328 ns) | 30.3 Hz | 7, 8 |
| 29.5 | 30.3 Hz | 7, 8 |
| 28 | 28.6 Hz | 8, 9 |

`update_rate` 와 비례하지 않아서 설정으로 정확한 30.000 Hz 를 만들지 못했다 (Gazebo 렌더 스레드의 예약 방식, 원인 미확인).
대신 **인터페이스 노드의 루프가 카메라 프레임을 따르게** 했다 ([`interface.md`](interface.md) 3 장) →
이미지와 `/gripper/*` stamp 가 항상 같다 (91/91). 남는 차이는 주기가 1 % 빠른 것 (30.3 vs 30 Hz). 녹화 변환은 25 Hz 로 다시 샘플하므로 영향이 작다.

`update_rate` 를 바꿔 보려면: `xacro_args:="wrist_camera_update_rate:=29.5"`.

## 5. 남은 차이

| 항목 | Isaac Sim | Gazebo |
|---|---|---|
| 렌더러 | RTX (DLSS Performance) | ogre2 (기본 조명, 하늘 텍스처) → 색·밝기·그림자가 다르다. 정책을 손목 영상만으로 다시 학습할 때 감안 |
| 주기 | 30.0 Hz | 약 30.3 Hz (4 장) |
| 배경 | 테이블·실내 | 바다·하늘 (위를 보면 하늘과 구름) |
