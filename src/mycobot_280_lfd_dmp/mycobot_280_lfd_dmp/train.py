"""합성 위빙 데이터로 DMP 스킬 모델 학습 (W4 07.23).

shape당 프로토타입 DMP 스킬 하나를 만든다. 여러 시연의 forcing weights를
평균하지 않고 **medoid**(위빙 파라미터가 데이터셋 평균에 가장 가까운 대표
시연)를 골라 그 하나로 학습한다. 이유:

  위빙은 진동 운동이고 시연마다 주파수가 ±지터로 다르다(이 데이터셋은
  약 15%). 위상 공간에서 forcing weights를 평균하면 서로 어긋난 진동이
  겹쳐 진폭이 뭉개진다(측정상 약 9% 감쇠). medoid는 진폭을 온전히 보존하며,
  진폭·주파수 같은 변이는 07.24에서 파라미터로 뽑아 변조하는 게 이 파이프
  라인의 설계다(schedule 07.24→07.27). 즉 프로토타입은 '깨끗한 정본 위빙',
  변이는 별도 파라미터.

품질 지표 주의: 프로토타입을 각 시연의 시작/끝점·시간으로 재타겟해 pose
점별 RMSE를 재면 값이 크게 나오는데(수 mm), 이는 모델 결함이 아니라 시연
간 위빙 '주파수' 차이로 생기는 위상 드리프트다 — 점별 RMSE는 주파수가 다른
진동을 비교하는 데 부적합하다. 모델 충실도는 medoid 자기적합 RMSE와 진폭
보존으로 판단한다. 주파수까지 정렬한 비교는 07.24 파라미터 추출의 몫이다.

  ros2 run mycobot_280_lfd_dmp train_dmp --shape all
  ros2 run mycobot_280_lfd_dmp train_dmp --shape line --out-dir ~/ros2_ws/data/models
"""
import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from mycobot_280_lfd_data.schema import load_csv

from .bridge import DEFAULT_N_WEIGHTS, fit, reproduce, reproduction_error, to_arrays
from .model import DMPSkill

SHAPES = ('line', 'zigzag', 'crescent')

# medoid 선택에 쓰는 위빙 파라미터 (메타데이터 weaving 블록)
WEAVE_PARAMS = ('amplitude_m', 'frequency_hz', 'travel_speed_mps')


def select_medoid(trials):
    """위빙 파라미터가 데이터셋 평균에 가장 가까운 시연의 인덱스.

    파라미터별 스케일이 달라 z-정규화 후 유클리드 거리로 대표성을 잰다.
    """
    P = np.array([[t.meta['weaving'][k] for k in WEAVE_PARAMS] for t in trials])
    z = (P - P.mean(axis=0)) / (P.std(axis=0) + 1e-12)
    return int(np.argmin(np.linalg.norm(z, axis=1)))


def weave_amplitude_mm(Y):
    """재현/시연 궤적의 위빙 진폭 근사 [mm] (횡방향 y의 반진폭)."""
    return float(Y[:, 1].max() - Y[:, 1].min()) * 1000.0 / 2.0


def train_shape(shape, trials, n_weights):
    """시연 리스트 → (DMPSkill 프로토타입, medoid 인덱스, medoid 자기적합 오차)."""
    mi = select_medoid(trials)
    medoid = trials[mi]
    T, Y = to_arrays(medoid)
    dmp = fit(medoid, n_weights_per_dim=n_weights)
    _, Yr = dmp.open_loop()

    skill = DMPSkill(
        shape=shape,
        n_weights_per_dim=n_weights,
        execution_time=float(dmp._execution_time),
        dt=float(np.median(np.diff(T))),
        # alpha_y/beta_y는 축별 (6,) 배열이지만 기본값은 전 축 균일 —
        # CartesianDMP 생성자가 받는 스칼라로 저장(첫 원소).
        alpha_y=float(np.asarray(dmp.alpha_y).flat[0]),
        beta_y=float(np.asarray(dmp.beta_y).flat[0]),
        start_y=dmp.start_y.copy(), goal_y=dmp.goal_y.copy(),
        weights=dmp.get_weights(),
        meta={'n_demos': len(trials),
              'medoid_trial': medoid.meta.get('trial_id'),
              'source_trials': [t.meta.get('trial_id') for t in trials],
              'lead_in_s': medoid.meta.get('lead_in_s'),
              'weaving': medoid.meta.get('weaving'),
              'trained': datetime.now().isoformat(timespec='seconds')})
    return skill, mi, reproduction_error(Y, Yr)


def retarget_errors(skill, trials):
    """프로토타입을 각 시연의 시작/끝점·시간으로 재타겟해 재현한 오차 리스트.

    주의: pose 점별 RMSE라 시연 간 위빙 주파수 차이(위상 드리프트)에 지배된다
    — 모듈 docstring 참고. 모델 결함 지표가 아니라 '주파수 정렬 없이 재타겟만
    했을 때'의 값이다.
    """
    errs = []
    for traj in trials:
        T, Y = to_arrays(traj)
        _, Yr = reproduce(skill.to_dmp(), start_y=Y[0], goal_y=Y[-1],
                          execution_time=float(T[-1]))
        errs.append(reproduction_error(Y, Yr))
    return errs


def load_trials(data_root, shape):
    """<data_root>/synthetic/<shape>/<shape>_*.csv 를 정렬해 로드."""
    paths = sorted((data_root / 'synthetic' / shape).glob(f'{shape}_*.csv'))
    trials = [load_csv(p) for p in paths]
    for t in trials:
        if t.meta.get('shape') != shape:
            raise ValueError(f"{t.meta.get('trial_id')}: shape 불일치")
    return trials


def main(argv=None):
    p = argparse.ArgumentParser(description='합성 위빙 데이터로 DMP 스킬 학습')
    p.add_argument('--shape', choices=SHAPES + ('all',), default='all')
    p.add_argument('--data-root', type=Path,
                   default=Path.home() / 'ros2_ws/data/trajectories')
    p.add_argument('--out-dir', type=Path,
                   default=Path.home() / 'ros2_ws/data/models')
    p.add_argument('--n-weights', type=int, default=DEFAULT_N_WEIGHTS)
    args = p.parse_args((argv if argv is not None else sys.argv)[1:])

    shapes = SHAPES if args.shape == 'all' else (args.shape,)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    failed = 0
    for shape in shapes:
        trials = load_trials(args.data_root, shape)
        if not trials:
            print(f'{shape}: 시연 없음 — 건너뜀')
            failed += 1
            continue

        skill, mi, medoid_err = train_shape(shape, trials, args.n_weights)
        out = args.out_dir / f'{shape}.npz'
        skill.save(out)

        # 저장→복원 왕복이 온전한지: 복원 모델로 medoid를 재현해 오차 재확인
        reloaded = DMPSkill.load(out)
        _, Yp = reproduce(reloaded.to_dmp())
        amp_proto = weave_amplitude_mm(Yp)
        amp_demos = np.array([weave_amplitude_mm(to_arrays(t)[1]) for t in trials])

        errs = retarget_errors(reloaded, trials)
        rt = np.array([e['rmse_mm'] for e in errs])
        print(f'{shape:9s} 시연 {len(trials):2d}건 → {out.name} '
              f"(medoid: {skill.meta['medoid_trial']})")
        print(f'    medoid 자기적합 RMSE  {medoid_err["rmse_mm"]:.3f} mm '
              f'(최대 {medoid_err["max_mm"]:.3f})')
        print(f'    위빙 진폭  프로토타입 {amp_proto:.3f} vs 시연평균 '
              f'{amp_demos.mean():.3f} mm (충실도 {amp_proto / amp_demos.mean() * 100:.0f}%)')
        print(f'    재타겟 pose RMSE {rt.mean():.3f} ± {rt.std():.3f} mm '
              f'— 시연 간 주파수 차이(위상 드리프트)에 지배, 07.24에서 해소')

    print(f'\n학습 {len(shapes) - failed}/{len(shapes)} shape 완료 → {args.out_dir}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
