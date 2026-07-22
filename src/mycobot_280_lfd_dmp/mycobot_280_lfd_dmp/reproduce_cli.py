"""데모 궤적을 DMP로 학습·재현해 스키마 v1 CSV로 내보낸다 (W4 07.22).

연동 결과를 눈으로 확인하는 경로: 여기서 만든 CSV를 W3의
trajectory_tf_publisher에 물리면 RViz에서 데모와 겹쳐 볼 수 있다
(dmp_compare.launch.py가 그 조합을 자동화한다).

  ros2 run mycobot_280_lfd_dmp dmp_reproduce <입력.csv> -o <출력.csv>
  ros2 run mycobot_280_lfd_dmp dmp_reproduce <입력.csv> -o <출력.csv> --goal-shift 0.02 0 0
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from mycobot_280_lfd_data.schema import load_csv, save_csv, with_lead_in

from .bridge import (DEFAULT_N_WEIGHTS, fit, reproduce, reproduction_error,
                     to_arrays, to_trajectory)


def main(argv=None):
    p = argparse.ArgumentParser(
        description='DMP 학습 → 재현 → 스키마 v1 CSV 출력')
    p.add_argument('input', type=Path, help='데모 궤적 CSV')
    p.add_argument('-o', '--output', type=Path, required=True)
    p.add_argument('--n-weights', type=int, default=DEFAULT_N_WEIGHTS)
    p.add_argument('--goal-shift', type=float, nargs=3, metavar=('DX', 'DY', 'DZ'),
                   help='끝점을 [m]만큼 이동해 일반화 재현')
    p.add_argument('--execution-time', type=float,
                   help='재생 시간 [s] (기본: 데모와 동일)')
    p.add_argument('--lead-in', type=float, default=0.0, metavar='SEC',
                   help='학습 전 데모 앞에 정지 구간 SEC초 삽입 (기동 과도 완화). '
                        '--save-demo와 함께 쓰면 리드인 적용된 데모도 저장')
    p.add_argument('--save-demo', type=Path,
                   help='(리드인 적용된) 데모 궤적을 이 경로에 저장 — RViz 비교용')
    args = p.parse_args((argv if argv is not None else sys.argv)[1:])

    traj = load_csv(args.input)
    if args.lead_in > 0.0:
        traj = with_lead_in(traj, args.lead_in)
    T_demo, Y_demo = to_arrays(traj)
    if args.save_demo is not None:
        args.save_demo.parent.mkdir(parents=True, exist_ok=True)
        save_csv(traj, args.save_demo)
        print(f'{args.save_demo}  (데모, {len(traj.data)}행)')
    dmp = fit(traj, n_weights_per_dim=args.n_weights)

    goal_y = None
    if args.goal_shift is not None:
        goal_y = dmp.goal_y.copy()
        goal_y[:3] += np.asarray(args.goal_shift, float)

    T, Y = reproduce(dmp, goal_y=goal_y, execution_time=args.execution_time)

    dmp_meta = {'n_weights_per_dim': args.n_weights}
    if args.lead_in > 0.0:
        dmp_meta['lead_in_s'] = round(args.lead_in, 4)
    if args.goal_shift is not None:
        dmp_meta['goal_shift_m'] = [float(v) for v in args.goal_shift]
    if args.execution_time is not None:
        dmp_meta['execution_time_s'] = float(args.execution_time)

    repro = to_trajectory(T, Y, args.output.stem, traj.meta, dmp_meta)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    save_csv(repro, args.output)

    print(f'{args.output}  ({len(repro.data)}행)')
    # 일반화/시간 변경 시에는 데모와 다른 궤적이 정상이므로 오차 비교는 무의미
    if goal_y is None and args.execution_time is None:
        err = reproduction_error(Y_demo, Y)
        print(f"  재현 오차: RMSE {err['rmse_mm']:.3f} mm  "
              f"(정상상태 {err['steady_rmse_mm']:.3f} / "
              f"최대 {err['max_mm']:.3f})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
