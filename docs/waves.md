# 파도 세기별 평가 조건

작성: 2026-11-03

## 1. 파도 설정

VRX 파도는 world 의 `vrx::PublisherPlugin` 이 2 s 마다 `/vrx/wavefield/parameters` 로 보내는 값으로 정해진다
(보트 부력 `vrx::Surface` 와 파도 모양 `coast_waves` 가 같은 값을 읽음).

| world | gain | period [s] | direction | steepness |
|---|---|---|---|---|
| `empty_ocean` | 0.3 | 5 | 0 | 0 |
| `empty_ocean_calm` | 0 | 5 | 0 | 0 |

launch 인자로 바꿀 수 있다 (world 파일 복사본을 임시 폴더에 만들어 씀, 끝나면 지움):

```bash
ros2 launch ocean_sim2sim_bringup wamv_arm.launch.py wave_gain:=0.18          # wave_period:=5
```

바람: `USVWind` 평균 0 m/s (보트에만, 드론에는 바람 없음 = Isaac Sim 과 같음).

## 2. 세기별 로봇 베이스 움직임 (측정, 2026-11-03)

`/base/odom` (로봇 마운트) 과 `/base/imu` (당시 VRX IMU 위치 (0.3, −0.2, 1.3), 지금은 (−0.15, 0, 1.3)) 를 60 s 기록. drone_mode fixed (보트 움직임만 봄).

| gain | 상하 표준편차 | 상하 최대-최소 | 주기 | 앞뒤 최대-최소 | 좌우 | roll 최대-최소 | pitch 최대-최소 | 가속도 (중력 뺀) RMS | 자이로 RMS |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0 mm | 0 mm | - | 0 | 0 | 0° | 0° | 0.021 m/s² | 0.95 °/s |
| 0.1 | 14.8 mm | 74 mm | 4.9 s | 18 mm | 4 mm | 0.18° | 0.71° | 0.032 | 0.94 |
| 0.3 (empty_ocean) | 43.8 mm | 223 mm | 4.9 s | 110 mm | 19 mm | 0.55° | 2.14° | 0.076 | 1.12 |
| 0.5 | 73.0 mm | 372 mm | 4.9 s | 285 mm | 40 mm | 0.92° | 3.57° | 0.121 | 1.36 |
| 0.8 | 117.6 mm | 597 mm | 4.9 s | 686 mm | 85 mm | 1.48° | 5.72° | 0.193 | 1.81 |

- 상하 움직임은 gain 에 비례: **표준편차 ≈ 147 mm × gain**. 주기는 파도 주기 (5 s) 그대로
- 보트가 파도 방향 (x) 으로 앞뒤로 크게 움직이고 **조금씩 떠밀려 감** (앞뒤 최대-최소에 떠밀림이 포함). 드론은 world 에 떠 있으므로
  에피소드 동안 드론과 로봇 사이 거리가 바뀜 (30 s 에피소드 기준 gain 0.3 에서 수 cm)
- 각도 (roll·pitch) 는 작다: gain 0.8 에서도 pitch 최대-최소 5.7°
- 자이로 RMS 0.95 °/s 는 IMU 잡음 (VRX 설정, 정지 상태에서도)

## 3. Isaac Sim Stewart 프리셋과 비교

Isaac Sim 움직이는 베이스 (Stewart, `docs/moving_base.md`) 는 6 축 사인파: big 진폭 x 0.15, y 0.10, z 0.15 m, roll·pitch 6°, yaw 3°,
mid = big × 0.5, small = big × 0.25 (수집 주력 small, 일부 mid). 사인파 진폭 A 의 표준편차 = A / √2.

| Isaac 프리셋 | z 진폭 (표준편차) | roll·pitch 진폭 | 같은 상하 표준편차가 되는 Gazebo gain | 그 gain 의 pitch 최대-최소 (Isaac 최대-최소) |
|---|---|---|---|---|
| small | 37.5 mm (26.5 mm) | 1.5° | **0.18** | 약 1.3° (3°) |
| mid | 75 mm (53 mm) | 3° | **0.36** | 약 2.6° (6°) |
| big | 150 mm (106 mm) | 6° | **0.72** | 약 5.1° (12°) |

- 상하 움직임을 기준으로 맞추면 보트의 기울기는 Stewart 보다 작다 (절반 정도). 앞뒤는 보트가 더 크다 (떠밀림 포함)
- 움직임 모양이 다르다: Stewart 는 축마다 정해진 사인파, 보트는 파도 스펙트럼 + 떠밀림
- Isaac 의 잡기 가능 시간 비율 (기구학만): small 94 %, mid 26 %, big 4.5 % (big 은 시작 자세 충돌 위험으로 수집 안 함)

## 4. 평가 조건

**정하지 않음 (2026-11-03 사용자 결정)**: 실제 배의 IMU 데이터를 보고, Gazebo 파도 (gain·period·direction) 와 Isaac Sim Stewart 움직임을
그 데이터에 맞춰 나중에 정한다. 그 전까지는 `world:=empty_ocean_calm` (파도 없음) 과 필요한 gain 을 `wave_gain:=` 으로 직접 준다.
3 장의 "같은 상하 표준편차가 되는 gain" 은 참고 값.

## 5. 정해야 할 것

- **보트 떠밀림 → 그대로 둠 (2026-11-03 사용자 결정 (a))**: 보트는 파도에 떠밀리고, world 에 떠 있는 드론과의 거리가 에피소드 중 바뀐다.
  이것이 보려는 현상이고, 이를 다루는 알고리즘 (떠밀림·흔들림 보상) 을 따로 만들 예정. 추진기 자리 유지·리셋 때 제자리 옮기기는 하지 않음
