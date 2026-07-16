# 복습 노트: W1~W2 (환경 구축 → Gazebo 디지털 트윈)

> 대상: ROS2를 처음 접하는 상태에서 여기까지 온 사람 (= 우리)
> 목표: "지금 뭐가 떠 있고, 왜 이렇게 만들었는지"를 남에게 설명할 수 있는 수준

---

## 1. 이 프로젝트가 하려는 것 (큰 그림)

**LfD (Learning from Demonstration)**: 사람이 용접 위빙 동작을 시연하면,
로봇이 그 궤적을 학습해서 새로운 상황(다른 시작점, 다른 길이)에도
일반화해 재생하는 파이프라인.

```
[사람 시연] → [궤적 데이터] → [DMP 학습] → [일반화된 궤적] → [로봇 재생]
                 (W3)          (W4)           (W4)          (W5)
```

핵심 전략은 **시뮬레이션 퍼스트**: 6주차까지 실물 로봇 없이 위 파이프라인을
가상에서 완성한다. 그래서 W1~W2에 만든 것이 전부 "가상 로봇 + 가상 물리 +
모션 계획" 인프라다. 실물(myCobot 280)은 W7에 드라이버만 갈아끼운다.

---

## 2. 지금까지 만든 것 (커밋 순서 = 학습 순서)

| 커밋 | 내용 | 한 줄 요약 |
|---|---|---|
| display.launch.py | RViz2 표시 | "로봇 **모양**을 화면에 그린다" |
| fake controller | ros2_control mock | "가짜 모터로 **관절을 움직인다**" |
| MoveIt2 설정 | planning group 'arm' | "목표를 주면 **경로를 계산한다**" |
| SRDF/kinematics 튜닝 | 충돌 매트릭스 등 | "계산이 빠르고 안전해진다" |
| Gazebo 스폰 | 물리 시뮬레이션 | "**중력과 관성**이 있는 세계에 로봇을 놓는다" |
| gazebo_demo.launch.py | MoveIt↔Gazebo 연결 | "**계획→물리 실행** 루프 완성 (M1)" |

각 단계는 아래 계층을 하나씩 쌓은 것이다:

```
┌─────────────────────────────────────────────┐
│  MoveIt2 (move_group)      ← 어디로 어떻게 갈지 "계획"     │
├─────────────────────────────────────────────┤
│  ros2_control (컨트롤러)    ← 계획된 궤적을 "추종"          │
├─────────────────────────────────────────────┤
│  하드웨어 계층 (스왑 가능!)  ← 실제로 관절을 "구동"          │
│   mock(W1) / Gazebo(W2) / pymycobot(W7)      │
├─────────────────────────────────────────────┤
│  로봇 모델 (URDF/xacro)     ← 로봇이 "무엇인지" 정의        │
└─────────────────────────────────────────────┘
```

**이 프로젝트 설계의 핵심 한 가지만 기억한다면**: 하드웨어 계층만 플러그인으로
갈아끼우고(mock ↔ GazeboSystem ↔ 실기 드라이버) 위 계층은 전혀 손대지 않는
구조라는 것. W7에 실물이 와도 MoveIt/컨트롤러/LfD 코드는 그대로다.

---

## 3. 핵심 개념 사전 (노베이스용)

### ROS2 기초
- **노드(Node)**: 하나의 실행 프로그램. `move_group`, `robot_state_publisher`,
  `rviz2` 전부 노드다. `ros2 node list`로 확인.
- **토픽(Topic)**: 노드끼리 데이터를 흘려보내는 방송 채널.
  예: `/joint_states` (관절 각도 방송). `ros2 topic echo /joint_states`.
- **액션(Action)**: "시작→진행상황→결과"가 있는 긴 작업 요청.
  예: `/arm_controller/follow_joint_trajectory` (궤적 실행하고 끝나면 보고).
- **파라미터**: 노드에 주입하는 설정값. launch 파일에서 yaml/딕셔너리로 전달.
- **launch 파일**: 여러 노드를 설정과 함께 한 방에 띄우는 파이썬 스크립트.

### 로봇 모델
- **URDF**: 로봇의 링크(강체)와 조인트(관절)를 XML로 기술. 형상(mesh),
  질량/관성, 관절 축·한계(각도/속도/토크)까지 전부 여기 있다.
- **xacro**: URDF의 매크로 언어. 변수/수식/조건문이 가능해서
  `inertia_scale:=100` 같은 인자로 같은 모델의 변형을 만들 수 있다.
- **우리 파일 구조** (mycobot_280_lfd_description/urdf/):
  - `mycobot_280_m5.urdf.xacro` — 본체 (링크/조인트/관성)
  - `mycobot_280_m5.ros2_control.xacro` — ros2_control 하드웨어 정의 (스왑 지점!)
  - `mycobot_280_m5_system.urdf.xacro` — 본체 + mock hardware (RViz/MoveIt용)
  - `mycobot_280_m5_gazebo.urdf.xacro` — 본체 + GazeboSystem + world 고정
- **SRDF**: URDF에 없는 "의미" 정보. planning group(어떤 관절 묶음이 '팔'인지),
  충돌 검사 생략 쌍(인접 링크끼리는 검사 안 함) 등. MoveIt 전용.
- **TF**: 좌표계 나무. `robot_state_publisher`가 URDF + `/joint_states`를 조합해
  "지금 각 링크가 어디 있는지"를 계속 방송한다. RViz는 이걸 그린다.

### ros2_control (가장 헷갈리는 부분 — 그림으로)
```
              controllers.yaml (1kHz)
                     │
        ┌────────────▼────────────┐
        │    controller_manager   │  ← 컨트롤러들을 로드/실행하는 관리자
        ├──────────────┬──────────┤
        │ arm_controller│ joint_state_broadcaster
        │ (JTC: 궤적    │ (관절 상태를 /joint_states
        │  추종 컨트롤러)│  토픽으로 방송)
        └──────┬───────┴──────────┘
               │ position 명령 / position·velocity 상태
        ┌──────▼──────────────────┐
        │  hardware_interface     │  ← ★ 스왑 지점 (URDF <ros2_control> 태그)
        │  W1: mock (에코만 함)    │
        │  W2: GazeboSystem (PID→토크→물리) │
        │  W7: pymycobot 어댑터    │
        └─────────────────────────┘
```
- **JTC (JointTrajectoryController)**: 시각이 찍힌 waypoint 목록을 받아
  스플라인 보간하며 매 주기 목표 각도를 하드웨어에 흘려보낸다.
- W1의 mock은 "명령을 그대로 상태로 돌려주는" 가짜라 물리가 없었고,
  W2의 GazeboSystem은 PID로 **토크를 계산해 물리 엔진에 가한다**. 이 차이가
  W2 트러블슈팅의 발단이었다 (아래 5장).

### MoveIt2
- **move_group**: 모션 플래닝 서버. "flange를 (x,y,z)로" 요청하면
  ① IK로 목표 관절각 계산 → ② OMPL로 충돌 없는 경로 탐색 →
  ③ 시간 파라미터화(속도/가속 한계 반영) → ④ 컨트롤러로 실행 지시.
- **IK / FK**: 역기구학(끝단 위치→관절각) / 정기구학(관절각→끝단 위치).
- **OMPL**: 샘플링 기반 경로 탐색 라이브러리 (RRT 계열).
- **moveit_simple_controller_manager**: move_group이 "실행"을 넘길 대상 목록.
  우리는 `arm_controller/follow_joint_trajectory` 액션 하나다.

### Gazebo Classic
- **gzserver / gzclient**: 물리 계산(서버)과 3D 화면(클라이언트)이 분리.
  `gui:=false`면 서버만 돈다 — 그래서 자동 테스트 때 화면이 안 보였다.
- **ODE**: Gazebo Classic의 물리 엔진 (강체 동역학 + 접촉 + 구속).
- **spawn_entity.py**: `robot_description` 토픽의 URDF를 시뮬레이션 세계에
  생성(스폰)하는 노드.
- **gazebo_ros2_control 플러그인**: gzserver 안에서 controller_manager를
  통째로 실행한다. 그래서 gazebo 런치에는 `ros2_control_node`가 따로 없다.
- **use_sim_time**: 물리 시간은 벽시계와 다르게 흐를 수 있어서, move_group이
  Gazebo의 `/clock`을 따라야 궤적 타임스탬프가 어긋나지 않는다.

---

## 4. 전체 데이터 흐름 (M1 데모에서 벌어지는 일)

RViz에서 목표를 끌어다 놓고 Plan & Execute를 누르면:

```
RViz(목표 pose)
  │ ① MoveGroup 액션 요청
  ▼
move_group ─ ② IK → ③ OMPL 경로 탐색 → ④ 시간 파라미터화
  │ ⑤ FollowJointTrajectory 액션 (waypoint 목록)
  ▼
arm_controller (JTC) ─ ⑥ 스플라인 보간, 1kHz로 목표각 갱신
  │ ⑦ position 명령
  ▼
gazebo_ros2_control (GazeboSystem) ─ ⑧ PID: 오차 → 토크
  │ ⑨ SetForce
  ▼
ODE 물리 엔진 ─ 중력·관성·감쇠·접촉 통합 → 관절이 실제로 회전
  │ ⑩ 관절 각도/속도 읽기
  ▼
joint_state_broadcaster → /joint_states → robot_state_publisher → TF
  │                                                                │
  └────────── ⑪ move_group이 현재 상태 감시 ──────────┘
                                                        ▼
                                             RViz 화면의 로봇이 움직임
```

Gazebo 화면의 로봇과 RViz 화면의 로봇은 **같은 `/joint_states`를 보는
두 개의 뷰**다. Gazebo는 물리 세계 그 자체, RViz는 데이터 시각화.

---

## 5. 트러블슈팅 복습 (여기서 배운 게 제일 많다)

W2 Gazebo 연동에서 만난 문제들. 전부 "**가짜 모터(mock)에서 물리 세계로
넘어오면서** 생긴 것"이라는 공통점이 있다.

### 문제 1: 저관성 체인의 수치 불안정
- **증상**: 스폰 직후 팔이 떨거나 NaN으로 폭발.
- **원인**: myCobot은 링크가 매우 가벼워(관성 ~1e-5 kg·m²) 1ms 물리 스텝에서
  수치 오차가 상대적으로 거대해진다.
- **해결**: `inertia_scale=100` — 질량(중력)은 그대로 두고 회전 관성만 100배.
  위치 제어 궤적 재생에서는 운동학 정확도에 영향 없음.
- **교훈**: 시뮬레이션 물리는 근사다. 목적(궤적 재생)에 맞게 조정해도 된다.

### 문제 2: SetPosition 한계순환 (±17° 진동)
- **증상**: 관절이 목표 주변을 ±17°로 규칙적으로 왕복.
- **원인**: 위치를 물리 엔진에 강제로 쓰는(SetPosition) 방식이 저관성 체인과
  상호작용해 결정론적 진동 발생.
- **해결**: `use_pid=true` — 힘 기반 제어(PID가 토크 계산 → SetForce)로 전환.
- **교훈**: "운동학 강제"와 "동역학 제어"는 다르다. 물리 세계에선 후자가 정석.

### 문제 3: 토크 한계 포화로 수렴 불능 (J1/J2가 10~30° 오차에서 정지)
- **증상**: 궤적 goal은 SUCCESS인데 실제 관절은 목표에서 멀리 떨어져 있음.
  심지어 목표 반대 방향에 가 있기도.
- **원인**: URDF 토크 한계가 실기 기준 5 N·m인데, 관성을 100배 한 세계에서
  PID가 요구하는 토크는 그보다 훨씬 커서 출력이 항상 ±5로 잘림(포화).
  잘린 출력은 위치 오차 정보를 잃는다.
- **해결**: `effort_scale=100` — 관성을 키웠으면 토크 한계도 같은 배율로.
  (동역학적 일관성: 가속 능력 = 토크/관성 보존)
- **교훈**: 스케일링은 세트로 해야 한다. 하나만 바꾸면 다른 곳에서 터진다.
- **여담**: goal이 SUCCESS였던 이유 — JTC의 goal tolerance를 설정 안 하면
  "도달 검사 없음"이다. 성공 코드만 믿지 말고 실측하자.

### 문제 4: 정지 상태에서 로봇 전체가 부들부들 (사용자가 발견!)
- **증상**: 가만히 있어야 하는데 눈에 띄게 떨림. "땅에 고정이 안 된 것 같다".
- **원인 A (주범) — 지면 관통**: g_base 충돌 메시가 링크 프레임 아래로 3cm
  뻗어 있어, z=0에 고정하면 지면을 3cm 파고든다. 접촉력이 밀어올리고
  world 고정 조인트가 되당기는 "구속 싸움"이 로봇 전체를 상시 흔들었다.
  실측 베이스 높이 z=0.0324가 스모킹 건.
- **원인 B — PID D항의 노이즈 증폭**: 물리 엔진의 관절 속도 신호에는 고주파
  노이즈가 있는데, kd=15가 이를 큰 토크로 바꿔 채터링을 만들었다.
- **해결**: 베이스를 z=0.032로 올려 접촉 제거 + kd=0으로 하고 감쇠는
  URDF `<dynamics damping>`(물리 엔진이 처리)으로 이관.
- **교훈**: "고정이 안 된 것 같다"는 직관이 정확했다. 증상 관찰 → 정량 측정
  (`gz model -p`로 베이스 위치 실측) → 원인 확정의 순서가 중요.

### 문제 5: 감쇠를 넣었더니 더 심하게 발산
- **증상**: damping 추가 후 오히려 진폭 12°, 속도 100 rad/s급 폭주.
- **원인**: ODE 기본 감쇠는 **explicit**(직전 스텝 속도로 토크 계산) 방식이라
  안정 조건 `B < 2·I_local/dt`가 있다. 여기서 I는 조인트에 붙은 링크 하나의
  국소 관성이라, 가벼운 손목 링크들이 조건을 깨버렸다.
- **해결**: 조인트마다 `<implicitSpringDamper>true` — 감쇠를 구속 솔버
  내부에서 푸는 implicit 방식은 무조건 안정.
- **교훈**: 수치 적분에서 explicit/implicit 차이는 실전 문제다.
  "이론상 임계감쇠"도 이산화 방식이 틀리면 발산한다.

### 최종 상태 (검증 수치)
- 정지 진동: 전 관절 p2p ≤ 0.008°, 속도 0.000 (완전 정지)
- 베이스: 원점 오차 < 1e-5 m
- 6관절 동시 궤적 추종: 최대 0.32°
- 툴끝 8자 궤적(12cm, IK 161 waypoint): 완주, 경로 오차 상한 ~9mm
- M1: 관절 목표 0.74°, 데카르트 목표 4.78mm 도달

---

## 6. 직접 해보는 복습 (명령어 치트시트)

```bash
# 항상 먼저 (새 터미널마다)
cd ~/ros2_ws && source install/setup.bash

# ① 모델만 보기 (물리 없음, 슬라이더로 관절 조작)
ros2 launch mycobot_280_lfd_description display.launch.py

# ② MoveIt 데모 (mock hardware — 물리 없음, 플래닝만)
ros2 launch mycobot_280_lfd_moveit_config demo.launch.py

# ③ 풀스택 디지털 트윈 (Gazebo 물리 + MoveIt + RViz) ← M1 데모
ros2 launch mycobot_280_lfd_moveit_config gazebo_demo.launch.py
#    → RViz에서 주황색 마커 드래그 → Plan & Execute → Gazebo 로봇이 움직임

# 떠 있는 동안 다른 터미널에서 구조 확인
ros2 node list                        # 어떤 노드들이 떠 있나
ros2 topic echo /joint_states --once  # 관절 상태 실물 보기
ros2 control list_controllers         # 컨트롤러 상태 (active?)
ros2 action list                      # 액션 서버 목록
ros2 topic hz /joint_states           # 발행 주기 (≈ 물리 1kHz의 브로드캐스트)

# 모델이 어떻게 생성되는지 눈으로 확인
xacro src/mycobot_280_lfd_description/urdf/mycobot_280_m5_gazebo.urdf.xacro \
  inertia_scale:=100.0 effort_scale:=100.0 | less
```

**추천 복습 순서**: ①→②→③을 직접 띄워 보면서, 각 단계에서
`ros2 node list`가 어떻게 달라지는지 비교해 보기. ②와 ③의 차이가
곧 "mock ↔ Gazebo 하드웨어 스왑"이다.

---

## 7. 다음 주(W3) 예고와 연결점

W3는 "가상 궤적 데이터 설계 및 합성 생성":
- Trajectory **CSV 스키마** 정의 — LfD 데이터의 표준 포맷 만들기
- **Synthetic Data Generator** — 직선/Zigzag/Crescent 위빙 궤적을 코드로 합성
- **Workpiece 좌표계** — 궤적을 로봇 기준이 아니라 작업물 기준으로 기술 (tf2)

이번에 만든 8자 궤적 스크립트(FK/IK 골격)가 그대로 위빙 궤적 생성의
토대가 된다. W2에서 물리·제어를 다져놨기 때문에, W3부터는 "데이터와
알고리즘" 계층만 신경 쓰면 된다 — 그게 시뮬레이션 퍼스트의 이득이다.
