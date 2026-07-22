"""DMP 연동 환경 점검 CLI (W4 07.22).

pip 없이 손으로 푼 의존성이라 "임포트는 되는데 동작이 다른" 상황을 조기에
잡는 게 목적이다. 버전 확인에서 그치지 않고 실제 합성 trial로 학습→재현→
스키마 검증까지 한 바퀴 돌린다.

ros2 run mycobot_280_lfd_dmp check_dmp_env [--data-root ...] [--per-shape 3]
"""
import argparse
import importlib
import sys
from pathlib import Path

from mycobot_280_lfd_data.schema import load_csv

from .bridge import DEFAULT_N_WEIGHTS, REPRODUCTION_NOTE, roundtrip

# (모듈명, 최소 기대 버전) — 최소 버전이 None이면 존재 여부만 확인
REQUIRED = (('numpy', None), ('scipy', None), ('yaml', None),
            ('pytransform3d', '3.15.0'), ('movement_primitives', '0.9.1'))

SHAPES = ('line', 'zigzag', 'crescent')

# 정상상태 RMSE 상한 [mm] — 위빙 진폭 5mm의 20%. 이보다 나쁘면 연동이
# 깨졌다고 보고 실패시킨다 (기동 과도 구간은 제외한 값이라 기준이 유효).
STEADY_RMSE_LIMIT_MM = 1.0


def check_imports():
    """의존성 임포트/버전 확인. (성공여부, 출력줄 리스트) 반환."""
    ok, lines = True, []
    for name, minimum in REQUIRED:
        try:
            mod = importlib.import_module(name)
        except ImportError as e:
            ok = False
            lines.append(f'  {name:20s} 없음 — {e}')
            continue
        version = getattr(mod, '__version__', '?')
        mark = ''
        if minimum and version != '?' and version < minimum:
            ok = False
            mark = f'  (기대 >= {minimum})'
        lines.append(f'  {name:20s} {version}{mark}')
    return ok, lines


def check_cython_accel():
    """선택적 Cython 가속 모듈(dmp_fast) 존재 여부 — 없어도 정상."""
    try:
        importlib.import_module('movement_primitives.dmp_fast')
        return 'Cython 가속(dmp_fast) 사용 가능'
    except ImportError:
        return 'Cython 가속(dmp_fast) 없음 — python 폴백 (위빙 규모에선 무해)'


def find_trials(data_root, per_shape):
    """shape별로 앞에서 per_shape개씩 trial CSV 경로를 모은다."""
    found = []
    for shape in SHAPES:
        paths = sorted((data_root / 'synthetic' / shape).glob(f'{shape}_*.csv'))
        found.extend(paths[:per_shape])
    return found


def main(argv=None):
    p = argparse.ArgumentParser(
        description='DMP 의존성 및 연동 라운드트립 점검')
    p.add_argument('--data-root', type=Path,
                   default=Path.home() / 'ros2_ws/data/trajectories',
                   help='합성 trial 루트 (<root>/synthetic/<shape>/)')
    p.add_argument('--per-shape', type=int, default=2,
                   help='shape당 점검할 trial 수')
    p.add_argument('--n-weights', type=int, default=DEFAULT_N_WEIGHTS)
    args = p.parse_args((argv if argv is not None else sys.argv)[1:])

    print('[의존성]')
    ok, lines = check_imports()
    for line in lines:
        print(line)
    print(f'  {check_cython_accel()}')
    if not ok:
        print('\n실패: 의존성 누락 — scripts/install_python_deps.sh 실행할 것')
        return 1

    trials = find_trials(args.data_root, args.per_shape)
    if not trials:
        print(f'\n실패: {args.data_root} 아래 합성 trial 없음 — '
              'ros2 run mycobot_280_lfd_data generate_trajectories 먼저 실행')
        return 1

    print(f'\n[라운드트립] 학습→재현→스키마 검증, '
          f'n_weights_per_dim={args.n_weights}')
    failed = 0
    for path in trials:
        try:
            _, err = roundtrip(load_csv(path), args.n_weights)
        except (OSError, ValueError) as e:
            failed += 1
            print(f'  {path.name:22s} 실패 — {e}')
            continue
        over = err['steady_rmse_mm'] > STEADY_RMSE_LIMIT_MM
        failed += over
        print(f"  {path.name:22s} RMSE {err['rmse_mm']:5.3f} mm  "
              f"(정상상태 {err['steady_rmse_mm']:5.3f} / "
              f"최대 {err['max_mm']:5.3f})"
              f"{'  <- 기준 초과' if over else ''}")

    print(f'\n주: {REPRODUCTION_NOTE}')
    if failed:
        print(f'실패 {failed}/{len(trials)}건 '
              f'(정상상태 RMSE 기준 {STEADY_RMSE_LIMIT_MM} mm)')
        return 1
    print(f'통과 {len(trials)}/{len(trials)}건 — DMP 연동 정상')
    return 0


if __name__ == '__main__':
    sys.exit(main())
