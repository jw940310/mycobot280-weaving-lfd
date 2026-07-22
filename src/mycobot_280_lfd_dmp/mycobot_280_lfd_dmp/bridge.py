"""스키마 v1 궤적 ↔ movement_primitives CartesianDMP 연동 계층 (W4).

이 모듈이 담당하는 유일한 일은 **표현 변환**이다. 학습 스크립트(07.23)와
파라미터 추출 노드(07.24)는 라이브러리를 직접 부르지 말고 여기를 거친다.
그래야 movement_primitives 버전이 바뀌거나 다른 DMP 구현으로 갈아탈 때
수정 지점이 이 파일 하나로 묶인다.

쿼터니언 규약: 스키마 v1도, pytransform3d/movement_primitives도 w-first라
변환 없이 그대로 이어붙인다 (ROS msg 경계에서만 schema.quat_* 헬퍼 필요).
"""
import numpy as np
from movement_primitives.dmp import CartesianDMP

from mycobot_280_lfd_data.schema import Trajectory, validate

# 위빙 궤적(~4초, 진폭 5mm) 기준 기본값. 30 이상에서는 재현 오차가 포화한다
# (병목은 표현력이 아님) — 표현력 한계가 아니라 데모/DMP 경계조건 문제였고,
# 그건 데모의 정지 리드인으로 해결했다. REPRODUCTION_NOTE 참고.
DEFAULT_N_WEIGHTS = 30

# 재현 오차 해석 노트. DMP는 시작 속도를 0으로 강제한다. 데모가 위빙 위상
# 중간(=속도 최대)에서 출발하면 초기 ~20%에 기동 과도가 몰렸는데, 지금은
# 합성 생성기(synth.py)가 각 데모 앞에 정지 리드인(아크 스타트 dwell,
# schema.with_lead_in)을 붙여 시작 속도를 0으로 맞추므로 그 과도가 사라진다
# (전 구간 균일, RMSE≈정상상태 RMSE). steady_rmse_mm는 리드인 없는 데모를
# 받았을 때도 추종 품질을 정직하게 재려고 남겨둔 진단 지표다.
REPRODUCTION_NOTE = ('리드인이 있으면 오차가 전 구간 균일하다. 리드인 없는 '
                     '데모는 초기 20%에 DMP 기동 과도가 몰린다(steady로 판단).')
TRANSIENT_RATIO = 0.2

POSE_COLUMNS = 7  # x, y, z, qw, qx, qy, qz


def to_arrays(traj):
    """Trajectory → (T, Y): T는 (N,) 시각 [s], Y는 (N×7) w-first 포즈."""
    return traj.t, np.column_stack([traj.positions, traj.quaternions_wxyz])


def fit(traj, n_weights_per_dim=DEFAULT_N_WEIGHTS, regularization=0.0,
        allow_final_velocity=False):
    """단일 trial을 CartesianDMP로 학습해 반환.

    execution_time/dt는 궤적에서 그대로 가져온다 — 메타데이터의
    sample_rate_hz가 아니라 실제 t 컬럼을 쓰므로 지터가 있어도 안전하다.
    """
    T, Y = to_arrays(traj)
    dmp = CartesianDMP(execution_time=float(T[-1]),
                       dt=float(np.median(np.diff(T))),
                       n_weights_per_dim=n_weights_per_dim)
    dmp.imitate(T, Y, regularization_coefficient=regularization,
                allow_final_velocity=allow_final_velocity)
    return dmp


def reproduce(dmp, start_y=None, goal_y=None, execution_time=None):
    """DMP를 적분해 (T, Y) 반환. 인자를 주면 시작/끝점/시간을 일반화한다.

    start_y/goal_y는 (7,) w-first 포즈. 일반화 테스트(07.27)의 진입점이다.
    """
    if execution_time is not None:
        # 프로퍼티 이름은 execution_time_ (뒤 언더스코어). 언더스코어 없는
        # dmp.execution_time = ... 는 엉뚱한 인스턴스 속성만 만들고 정준계는
        # 옛 _execution_time을 계속 써서 재생 시간이 안 바뀐다. setter는 이때
        # 포싱텀을 재초기화하며 가중치를 보존한다(movement_primitives _dmp.py).
        dmp.execution_time_ = float(execution_time)
    if start_y is not None or goal_y is not None:
        dmp.configure(
            start_y=None if start_y is None else np.asarray(start_y, float),
            goal_y=None if goal_y is None else np.asarray(goal_y, float))
    return dmp.open_loop()


def to_trajectory(T, Y, trial_id, source_meta, dmp_meta=None):
    """DMP 출력을 스키마 v1 Trajectory로 되돌린다 (원본 프레임 정보 승계).

    source_meta는 학습에 쓴 원본 trial의 meta. 좌표계/작업물 배치는 그대로
    물려받고, 어떤 DMP가 만들었는지는 'dmp' 키에 기록해 출처를 남긴다.
    source는 계측값이 아니므로 'synthetic'을 유지한다 (실로봇 재생분은
    W7 이후 'robot_replay'로 기록할 것).
    """
    Y = np.asarray(Y, float)
    if Y.ndim != 2 or Y.shape[1] != POSE_COLUMNS:
        raise ValueError(f'Y 형상 오류: {Y.shape} (기대 N×{POSE_COLUMNS})')

    dt = np.median(np.diff(T))
    meta = {k: source_meta[k] for k in
            ('schema', 'version', 'shape', 'source', 'frames', 'workpiece')
            if k in source_meta}
    meta['trial_id'] = trial_id
    meta['sample_rate_hz'] = round(float(1.0 / dt), 4)
    if 'weaving' in source_meta:
        meta['weaving'] = source_meta['weaving']
    meta['dmp'] = {'library': 'movement_primitives.CartesianDMP',
                   'source_trial': source_meta.get('trial_id'),
                   **(dmp_meta or {})}
    return Trajectory(meta=meta, data=np.column_stack([T, Y]))


def reproduction_error(Y_demo, Y_repro):
    """데모 대비 재현 위치 오차 [mm] 요약.

    전체값과 함께 기동 과도(초기 TRANSIENT_RATIO) 구간을 뺀 정상상태
    오차를 같이 낸다 — REPRODUCTION_NOTE 참고.
    """
    n = min(len(Y_demo), len(Y_repro))
    err = np.linalg.norm(Y_repro[:n, :3] - Y_demo[:n, :3], axis=1) * 1000.0
    tail = err[int(n * TRANSIENT_RATIO):]
    return {'rmse_mm': float(np.sqrt((err ** 2).mean())),
            'max_mm': float(err.max()),
            'steady_rmse_mm': float(np.sqrt((tail ** 2).mean())),
            'steady_max_mm': float(tail.max()),
            'n_samples': int(n)}


def roundtrip(traj, n_weights_per_dim=DEFAULT_N_WEIGHTS):
    """학습 → 재현 → Trajectory 복원을 한 번에. (재현 Trajectory, 오차) 반환.

    연동이 살아있는지 확인하는 가장 짧은 경로 — check_dmp_env가 이걸 쓴다.
    """
    T_demo, Y_demo = to_arrays(traj)
    dmp = fit(traj, n_weights_per_dim=n_weights_per_dim)
    T_repro, Y_repro = reproduce(dmp)
    repro = to_trajectory(
        T_repro, Y_repro, f"{traj.meta.get('trial_id', 'trial')}_dmp",
        traj.meta, {'n_weights_per_dim': n_weights_per_dim})
    issues = validate(repro)
    if issues:
        raise ValueError('재현 궤적이 스키마 검증 실패: ' + '; '.join(issues))
    return repro, reproduction_error(Y_demo, Y_repro)
