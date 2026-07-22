# DMP 연동 (W4 07.22)

합성 위빙 궤적(W3)을 DMP로 학습·재현하기 위한 `movement_primitives` 연동
기록. 학습 실험(07.23)과 파라미터 추출(07.24)의 출발점이다.

## 1. 의존성 설치

이 머신에는 `python3-pip`이 없고 세션 안에서 `sudo`도 못 쓴다. PyPI 배포판을
직접 받아 사용자 site-packages(`~/.local/lib/python3.10/site-packages`)에
푸는 방식으로 우회했다.

```bash
bash scripts/install_python_deps.sh      # 재실행 안전
ros2 run mycobot_280_lfd_dmp check_dmp_env
```

| 패키지 | 버전 | 배포 형태 | 비고 |
|---|---|---|---|
| `pytransform3d` | 3.15.0 | wheel (py3-none-any) | 그대로 unzip |
| `movement_primitives` | 0.9.1 | sdist만 존재 | 순수 파이썬 디렉터리만 복사 |

`numpy` 1.21.5 / `scipy` 1.8.0 / `lxml` 4.8.0은 우분투 기본 패키지로 이미
있었고, 위 두 라이브러리와 호환됐다.

### Cython 가속은 빌드하지 않았다

`movement_primitives`는 선택적으로 `dmp_fast`(Cython) 확장을 갖는데, 이
머신에는 Cython이 없다. 미존재 시 라이브러리가 순수 파이썬 스텝 함수로 자동
폴백하며, 위빙 궤적 규모(~400행, 4초)에서 학습+재현이 약 1.1초라 문제되지
않는다. 실시간 루프에 DMP를 넣게 되면(W5 이후) 그때 재검토할 것.

## 2. 연동 계층

궤적 스키마와 DMP 라이브러리 사이는 `mycobot_280_lfd_dmp.bridge` 한 곳으로만
연결한다. 학습 스크립트가 `movement_primitives`를 직접 부르지 않게 해서,
라이브러리 교체 시 수정 지점을 하나로 묶는 게 목적이다.

```python
from mycobot_280_lfd_dmp.bridge import fit, reproduce, to_trajectory

dmp = fit(traj)                                   # 학습
T, Y = reproduce(dmp, goal_y=new_goal)            # 일반화 재현
repro = to_trajectory(T, Y, 'line_0001_dmp', traj.meta)   # 스키마 v1 복원
```

**쿼터니언 규약이 맞아떨어진다**: 스키마 v1도 w-first, pytransform3d /
movement_primitives도 w-first라 변환 없이 이어붙인다. 순서 변환이 필요한
곳은 ROS 메시지 경계뿐이고 거기서는 `schema.quat_*` 헬퍼를 쓴다.

재현 궤적은 계측값이 아니므로 `source: synthetic`을 유지하고, 출처는 메타
데이터 `dmp:` 키(라이브러리, 원본 trial, 가중치 수)에 남긴다. 실물 로봇
재생분은 W7 이후 `robot_replay`로 기록한다.

## 3. 기동 과도응답과 정지 리드인

처음엔 재현 오차가 위빙 진폭 5mm의 약 25%(전체 RMSE 1.0~1.7mm)로 커 보였다.
원인은 표현력이 아니라 **경계조건**이었다: DMP는 **시작 속도를 0으로 강제**
하는데, 합성 데모는 위빙 위상 중간(=횡방향 속도 최대)에서 출발하므로 DMP가
정지 상태에서 따라잡는 과도 구간을 거친다. 그래서 오차가 **초기 20% 구간에
집중**됐고(line 기준 구간별 y오차 3.26→1.06→0.21→0.23→0.25 mm), 이를 확인한
근거:

- `n_weights_per_dim`을 30 → 120으로 올려도 RMSE가 1.04mm에서 **포화** —
  표현력 문제가 아니라는 증거. (기본값을 30으로 둔 근거)
- `allow_final_velocity`는 거의 무영향(1.622 → 1.621 mm) — 끝점이 아니라
  시작점 제약이 문제이므로.

### 해결: 아크 스타트 dwell (기본 적용)

합성 생성기가 각 데모 앞에 **정지 리드인**(`schema.with_lead_in`, 기본
0.5초)을 붙인다. 시작 속도가 0이 되어 DMP 제약과 맞아떨어지고, 과도가
사라진다. 용접의 아크 스타트 dwell과도 물리적으로 대응한다.
`generate_trajectories --lead-in 0` 으로 끌 수 있다.

리드인 0.5초 전/후 (line_0001):

| | 초기 20% RMSE | 전체 RMSE | 최대 오차 |
|---|---|---|---|
| 리드인 없음 | 3.52 mm | 1.67 mm | 4.40 mm |
| **리드인 0.5초** | **0.19 mm** | **0.28 mm** | **0.69 mm** |

리드인 적용 후 30 trial 전수 재현 RMSE는 **0.21~0.36 mm**, 최대 오차
0.5~1.3 mm로 전 구간 균일하다(정상상태 RMSE ≈ 전체 RMSE).

`bridge.reproduction_error()`는 전체값과 정상상태값(초기 20% 제외)을 함께
반환한다 — 리드인이 없는 데모(예: 실측 데이터)를 받았을 때도 추종 품질을
정직하게 재기 위한 진단 지표로 남겨둔다. `check_dmp_env`의 합격 기준(정상상태
1.0 mm)도 이 값에 건다.

RViz로 눈으로 확인:

```bash
ros2 launch mycobot_280_lfd_dmp dmp_compare.launch.py \
    demo_csv:=<데모.csv> dmp_csv:=<재현.csv>
```

파랑(데모)·주황(DMP 재현)이 시작 구간까지 포개지면 리드인이 먹은 것이다.
화면 원점에 고정된 좌표축은 workpiece 프레임(정지), 움직이는 축이
tcp_demo/tcp_dmp(동기 재생이라 겹쳐 보임)다.
