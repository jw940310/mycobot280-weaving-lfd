# Trajectory CSV 데이터 스키마 v1 (작업물 좌표계 기준)

LfD 파이프라인의 표준 궤적 포맷. 합성 데이터(W3), DMP 학습(W4),
시뮬레이션 재생(W5), 실측 시연 데이터(W8)가 모두 이 스키마 하나를 쓴다.

참조 구현: `mycobot_280_lfd_data` 패키지 (`schema.py`의 load/save/validate).

---

## 1. 좌표계 정의

### workpiece 프레임 (모든 pose의 기준)
```
        Z (표면 법선, 위)
        │
        │
        o──────── X (용접선 진행 방향)
       /   ~~~~~~~~~~~~~~~> 위빙하며 전진
      Y (측방 = 위빙 진동 방향, 오른손 법칙: Y = Z × X)
```
- **원점**: 용접선(seam) 시작점
- **X축**: 용접선 진행 방향
- **Y축**: 작업 표면 위 측방 — 위빙 진동이 주로 실리는 축
- **Z축**: 작업 표면 법선(위쪽). 오른손 좌표계.

궤적을 workpiece 기준으로 기록하는 이유: 작업물이 어디에 놓이든
(로봇 기준 위치·방향이 바뀌어도) **같은 궤적 데이터**로 표현된다.
재생 시에는 `workpiece` 메타데이터(로봇 기준 자세) 또는 실측 tf
(W8: 카메라→ArUco)로 로봇 좌표계로 변환한다.

### tool(TCP) 프레임
- 기록 대상은 TCP(Tool Center Point)의 pose.
- 시뮬레이션 단계의 TCP = `joint6_flange` 원점. W8에서 펜/더미토치 장착 후
  TCP 캘리브레이션 값으로 교체 (스키마는 불변, `frames.tool`만 갱신).
- 자세 컨벤션: tool Z축이 작업 표면을 향하도록(= workpiece -Z와 정렬이
  기본 자세) 기록한다. 수직 용접 토치 기준.

---

## 2. 파일 포맷

단일 파일 = 단일 trial. 구조는 3부분:

```csv
# schema: mycobot_lfd_trajectory
# version: 1
# trial_id: line_0001
# shape: line
# source: synthetic
# frames:
#   reference: workpiece
#   tool: joint6_flange
# workpiece:
#   parent: g_base
#   xyz: [0.13, -0.06, 0.05]
#   rpy: [0.0, 0.0, 0.0]
# sample_rate_hz: 100.0
# weaving:
#   amplitude_m: 0.005
#   frequency_hz: 1.0
#   travel_speed_mps: 0.01
# created: '2026-07-16T14:00:00'
t,x,y,z,qw,qx,qy,qz
0.000000,0.000000,0.000000,0.008000,0.0000000,1.0000000,0.0000000,0.0000000
0.010000,0.000100,0.000314,0.008000,0.0000000,1.0000000,0.0000000,0.0000000
...
```

1. **메타데이터 블록**: `# `로 시작하는 연속 라인. `# ` 제거 후 YAML로 파싱.
2. **컬럼 헤더**: `t,x,y,z,qw,qx,qy,qz` 고정.
3. **데이터 행**: 시간순.

`pandas.read_csv(path, comment='#')`로도 바로 읽힌다.

---

## 3. 컬럼 명세

| 컬럼 | 단위 | 정의 |
|---|---|---|
| `t` | s | 0에서 시작, 순증가(strictly increasing). 균일 dt 권장 |
| `x y z` | m | TCP 위치 (workpiece 프레임) |
| `qw qx qy qz` | – | TCP 자세 단위 쿼터니언 (workpiece→tool 회전), **w-first** |

**쿼터니언 순서 주의**: w-first는 W4에서 사용할
movement_primitives/pytransform3d 관례다. ROS `geometry_msgs/Quaternion`은
(x,y,z,w)로 순서가 다르므로 반드시 `schema.py`의 변환 헬퍼
(`quat_wxyz_to_msg` / `quat_msg_to_wxyz`)를 거칠 것. (버그 1순위 항목)

속도/가속도 컬럼은 두지 않는다 — DMP 학습 시 수치 미분으로 유도
(포맷 단순성 우선, 잡음 처리는 학습 단계 책임).

---

## 4. 메타데이터 명세

| 키 | 필수 | 값 |
|---|---|---|
| `schema` | ✓ | `mycobot_lfd_trajectory` 고정 |
| `version` | ✓ | `1` |
| `trial_id` | ✓ | 파일명(확장자 제외)과 일치 권장. 예: `zigzag_0007` |
| `shape` | ✓ | `line` \| `zigzag` \| `crescent` \| `freeform` |
| `source` | ✓ | `synthetic` \| `human_demo` \| `robot_replay` |
| `frames.reference` | ✓ | `workpiece` 고정 (v1) |
| `frames.tool` | ✓ | TCP 링크/프레임 이름. 예: `joint6_flange` |
| `workpiece.parent` | ✓ | workpiece 프레임의 부모. 시뮬: `g_base`, 실측: `camera_link` 등 |
| `workpiece.xyz`, `.rpy` | ✓ | 부모 기준 workpiece 자세 [m], [rad]. 재생 시 static tf로 발행 |
| `sample_rate_hz` | ✓ | 목표 샘플레이트. 실측은 명목값 |
| `weaving.*` | – | 합성 파라미터(진폭[m]/주파수[Hz]/전진속도[m/s]). 실측은 생략 가능 |
| `created` | – | ISO 8601 |

---

## 5. 유효성 규칙 (validator가 검사)

1. 필수 메타데이터 키 존재, `schema`/`version` 일치
2. 컬럼 헤더가 명세와 정확히 일치
3. NaN/Inf 없음
4. `t[0] == 0` (±1e-9), `t` 순증가
5. dt 지터: `max|dt - median(dt)| / median(dt) < 0.5` (샘플 누락 감지)
6. 쿼터니언 노름 = 1 ± 1e-3
7. 위치 크기 상식 검사: `‖(x,y,z)‖ < 0.5 m` (myCobot 작업공간 스케일)
8. 행 수 ≥ 10

---

## 6. 파일 배치 규약

```
~/ros2_ws/data/trajectories/<source>/<shape>/<trial_id>.csv
  예: data/trajectories/synthetic/zigzag/zigzag_0007.csv
```
- `data/`는 git 추적 제외 (대량 trial). 스키마의 정본 예시는
  `mycobot_280_lfd_data/samples/line_sample.csv` 하나만 패키지에 포함.

---

## 7. 버전 정책

- 컬럼 추가/의미 변경 시 `version`을 올리고 validator에 마이그레이션 규칙 추가.
- v1은 pose-only. 후보 확장(v2+): 힘/토크 컬럼(실측), 품질 라벨, 세그먼트 태그.
