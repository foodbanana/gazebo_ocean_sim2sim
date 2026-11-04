# 회귀 시험 (한 명령)

작성: 2026-11-04 · `ocean_sim2sim_bringup/scripts/regression.py`, `ocean_sim2sim_bringup/config/regression.yaml`

설정 (물리 엔진·충돌 형상·마찰·그리퍼 힘·PX4 연결 등) 을 바꾼 뒤 지금까지 확인한 값이 그대로인지 한 번에 본다.
단계마다 headless bringup 을 새로 띄우고, 시험하고, 끈다 (단계끼리 상태가 섞이지 않음).

## 1. 실행

```bash
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash   # venv: PX4 단계에 pymavlink
ros2 run ocean_sim2sim_bringup regression.py                 # 전체 (약 10 분)
ros2 run ocean_sim2sim_bringup regression.py --quick         # 빠르게: E + G3 1 회 + 보호 정지 + 데모 success (약 5 분)
ros2 run ocean_sim2sim_bringup regression.py --only grip,pstop   # 고른 단계만 (grip, pstop, demo, waves)
```

- 시작할 때 **다른 Gazebo 서버가 떠 있으면 멈춘다** (두 서버가 돌면 결과가 틀어짐, drone_px4.md 6 장). 단계 사이에도 확인
- 끝나면 (Ctrl+C 포함) bringup 의 프로세스 그룹을 끄고, 남은 시뮬레이션 프로세스 (`gz sim`, `parameter_bridge`, `px4` …) 를 정리한다
- 종료 코드: PASS 항목이 모두 통과하면 0, 아니면 1

## 2. 단계와 확인 항목

| 단계 | bringup | 확인 |
|---|---|---|
| grip | `empty_ocean_calm`, `drone_mode:=fixed` | 가만히 있을 때 RTF, `grasp_test.py` E · G3 (2 회) · G1 · G4 · H 0.8 kg (grasp.md 3 장) |
| pstop | `empty_ocean_calm`, `drone_mode:=fixed` | 리셋 → shoulder_lift 를 +0.3 rad/s 로 배에 박기 → **접촉력**으로 정지 → 리셋 성공·팔이 시작 자세 (0.02 rad 안)·`/protective_stop` 해제 → wrist_1 1 rad 점프 → 정지 → 리셋 (interface.md 8 장) |
| demo | `empty_ocean_calm`, `drone_mode:=px4 drone_motion:=static` | 호버 RTF, `grasp_demo.py --case all`, 데모 동안 RTF (잡고 있을 때 / 아닐 때), 보호 정지가 한 번도 안 걸림 (drone_px4.md 4 장) |
| waves | `empty_ocean` (gain 0.3), `drone_mode:=px4 drone_motion:=hover` | 호버 RTF, 보호 정지 안 걸림. 파도 속 스크립트 데모는 하지 않음 (grasp_demo.md 6 장: 도달 판정의 한계) |

판정 (기댓값·허용 범위와 그 출처는 `regression.yaml` `checks`):

- **PASS/FAIL**: E 1.134 rad (±0.01), G3 미끄럼 힘 25~35 N (2026-11-04 다시 잰 27~32 N), 손가락당 수직력 23~30 N (26.6), G1·G4·H 미끄러짐
  0.5~0.6 mm 이하 (0.07·0.14·0.19), G4 관절 떨림 0.01° 이하, RTF 0.9 이상 (가만히·호버), 보호 정지 시나리오, 데모 success·offset_y (held)·empty (no_grasp_empty)
- **INFO** (실행마다 달라지는 것으로 알려진 값, 실패로 치지 않음): 데모 offset_x (모터 정지 뒤 회전 10° 경계에서 가끔 slip), miss
  (Isaac 과 다른 gap), success 회전, 잡고 있는 동안 RTF (0.25~0.5), G4·H 박스 떨림, 접촉 정지 때의 힘

## 3. 결과

`~/ocean_sim2sim_reports/regression_<시각>/` (`--report-dir` 로 바꿈):

| 파일 | 내용 |
|---|---|
| `report.txt` | 항목 표 (단계, 항목, PASS/FAIL/INFO, 값, 기대, 메모), 합계, 항목별 출처 |
| `results.json` | 같은 내용 + 단계별 시간·원자료 (grasp_test 파싱 결과, 보호 정지 로그 줄, 데모 결과 폴더) |
| `suite.log` | 진행 기록 |
| `<단계>/launch.log` | 그 단계 bringup 전체 출력 (보호 정지 이유는 여기서 읽음) |
| `grip/grasp_test.log`, `pstop/pstop.log`·`pstop.json`, `demo/grasp_demo.log`·`grasp_demo_<시각>/`, `*/rtf_*.jsonl` | 시험별 출력 |

- RTF = 1 s (벽시계) 마다 `/clock` sim 시간 증가 / 벽시계 증가, 중앙값을 씀. 데모의 "잡고 있는 동안" 은 타임라인에서 양쪽 손가락
  힘이 1 N 넘는 sim 시각이 들어 있는 표본
- 보호 정지 이유는 인터페이스 로그 `PROTECTIVE STOP at t … s: <이유>` 를 순서대로 읽는다 (첫 줄 = 충돌, 둘째 = 점프)

## 4. 첫 전체 실행 (2026-11-04)

전체 10.2 분 (grip 1.9, pstop 1.4, demo 5.7, waves 1.2 분; 단계마다 bringup 준비 30~70 s 포함). 24 PASS, **2 FAIL**, 9 INFO.

- **FAIL: G3 미끄럼 힘 27.3 / 30.4 N** (기록 9.3~9.5 N). GUI 로 띄워도 29.6 / 32.2 N. 수직력 (25~27 N)·G1·G4·H 는 기록과 같음.
  원인 (확인): `ApplyLinkWrench` persistent 힘은 메시지마다 **더해지고**, grasp_test 는 증가분만 보낸다. 브리지가 둘이면 같은 증가분이
  두 번 들어가 실제 힘이 두 배가 된다 — 같은 토픽 브리지를 하나 더 띄우면 보고된 당김이 19.4~22.6 → 11.5~11.9 N 으로 반이 됨.
  커밋 809056e 전에는 중단된 grasp_test 의 브리지가 남았으므로, 기록된 9.4 N (보고된 당김 1.6 N) 은 남은 브리지 약 12 개로 실제
  당김이 부풀려진 값으로 보인다 → → 2026-11-04 다시 재서 (27.3~32.3 N) grasp.md 3 장·drone_px4.md 8 장과 regression.yaml 범위 (25~35 N) 를 고침. 마찰계수 표는 무효로 표시 (다시 재지 않음).
  `regression.yaml` 의 범위는 다시 잴 때까지 기록값 그대로 둠 (다시 잴지 결정 필요)
- 보호 정지: 충돌 정지는 접촉력 (wrist_2_link 620~836 N, 기록 491~590 N), 점프 정지는 오차 6.4°, 리셋 모두 성공
- 데모: success·offset_y·offset_x held, empty no_grasp_empty, miss 는 이번에 **no_grasp_empty** (Isaac Sim 과 같음, 드론이 43 mm 밀림).
  잡고 있는 동안 RTF 중앙값 0.43 (최소 0.35), 그 밖 1.00. 호버 RTF 잔잔한 물·파도 모두 1.00
