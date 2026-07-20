"""합성 위빙 궤적 생성기 (W3): 스키마 v1 trial CSV 생성.

세 가지 위빙 패턴(line=정현파, zigzag=삼각파, crescent=반달 아크)을
파라미터 지터 + 저주파 대역 노이즈 + 자세 흔들림으로 변주해
사람 시연 유사 데이터를 만든다. W4 DMP 학습·검증용 입력.

ros2 run mycobot_280_lfd_data generate_trajectories --shape all --trials 2
"""
import argparse
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

from .schema import Trajectory, save_csv, validate

# 자세 기준: workpiece X축 180° 회전 → tool Z가 표면(-Z)을 향함 (수직 토치)
Q_BASE_WXYZ = np.array([0.0, 1.0, 0.0, 0.0])

GEN_SHAPES = ('line', 'zigzag', 'crescent')


def weave_xy(shape, t, amplitude, frequency, speed, crescent_bulge_ratio):
    """패턴별 XY 경로 [m]. X=용접선 진행, Y=위빙 진동.

    - line: y = A·sin — 매끄러운 정현파 위빙 (정본 샘플과 동일 정의)
    - zigzag: y = A·triangle — 모서리가 날카로운 삼각파
    - crescent: 반스트로크마다 +X로 볼록한 아크(가장자리 cusp), y는 ±A 왕복
    """
    phase = 2.0 * np.pi * frequency * t
    x = speed * t
    if shape == 'line':
        y = amplitude * np.sin(phase)
    elif shape == 'zigzag':
        y = amplitude * (2.0 / np.pi) * np.arcsin(np.sin(phase))
    elif shape == 'crescent':
        y = amplitude * np.cos(phase)
        x = x + crescent_bulge_ratio * amplitude * np.abs(np.sin(phase))
    else:
        raise ValueError(f'지원하지 않는 shape: {shape!r}')
    return x, y


def band_noise(rng, t, std, f_lo=0.15, f_hi=1.5, n_components=6):
    """저주파 정현파 합성 노이즈 (표준편차 std로 정규화). 손 떨림/드리프트 모사."""
    if std <= 0.0:
        return np.zeros_like(t)
    freqs = rng.uniform(f_lo, f_hi, n_components)
    phases = rng.uniform(0.0, 2.0 * np.pi, n_components)
    amps = rng.uniform(0.5, 1.0, n_components)
    sig = (amps[:, None] * np.sin(2.0 * np.pi * freqs[:, None] * t[None, :]
                                  + phases[:, None])).sum(axis=0)
    return sig * (std / sig.std())


def quat_mul_wxyz(q1, q2):
    """w-first 쿼터니언 곱 q1⊗q2. (N×4)×(4,) 또는 (N×4)×(N×4) 브로드캐스트."""
    w1, x1, y1, z1 = np.moveaxis(np.atleast_2d(q1), -1, 0)
    w2, x2, y2, z2 = np.moveaxis(np.atleast_2d(q2), -1, 0)
    return np.stack([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], axis=-1)


def wobble_quaternions(rng, t, std_rad):
    """기준 자세(Q_BASE)에 X/Y축 소각도 흔들림을 합성한 (N×4) w-first 쿼터니언."""
    rx = band_noise(rng, t, std_rad)
    ry = band_noise(rng, t, std_rad)
    angle = np.sqrt(rx ** 2 + ry ** 2)
    half = 0.5 * angle
    # sin(θ/2)/θ — θ→0에서 0.5로 수렴 (소각도 안전)
    k = np.where(angle > 1e-12, np.sin(half) / np.maximum(angle, 1e-12), 0.5)
    q_wobble = np.stack(
        [np.cos(half), k * rx, k * ry, np.zeros_like(t)], axis=-1)
    q = quat_mul_wxyz(q_wobble, Q_BASE_WXYZ)
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def generate_trial(shape, trial_id, cfg, rng):
    """단일 trial 생성. 지터 적용된 파라미터를 메타데이터에 기록해 반환."""
    j = cfg.jitter
    amplitude = cfg.amplitude * (1.0 + rng.uniform(-j, j))
    frequency = cfg.frequency * (1.0 + rng.uniform(-j, j))
    speed = cfg.speed * (1.0 + rng.uniform(-j, j))

    duration = cfg.seam_length / speed
    n_rows = int(round(duration * cfg.sample_rate)) + 1
    t = np.arange(n_rows) / cfg.sample_rate

    x, y = weave_xy(shape, t, amplitude, frequency, speed,
                    cfg.crescent_bulge)
    z = np.full_like(t, cfg.standoff)
    x = x + band_noise(rng, t, cfg.noise_std)
    y = y + band_noise(rng, t, cfg.noise_std)
    z = z + band_noise(rng, t, 0.5 * cfg.noise_std)  # 표면 법선 방향은 절반

    quats = wobble_quaternions(rng, t, np.deg2rad(cfg.ori_noise_deg))
    data = np.column_stack([t, x, y, z, quats])

    meta = {
        'schema': 'mycobot_lfd_trajectory',
        'version': 1,
        'trial_id': trial_id,
        'shape': shape,
        'source': 'synthetic',
        'frames': {'reference': 'workpiece', 'tool': cfg.tool_frame},
        'workpiece': {'parent': cfg.workpiece_parent,
                      'xyz': [round(v, 6) for v in cfg.workpiece_xyz],
                      'rpy': [round(v, 6) for v in cfg.workpiece_rpy]},
        'sample_rate_hz': float(cfg.sample_rate),
        'weaving': {'amplitude_m': round(float(amplitude), 6),
                    'frequency_hz': round(float(frequency), 4),
                    'travel_speed_mps': round(float(speed), 6)},
        'generator': {'tool': 'mycobot_280_lfd_data.synth',
                      'base_seed': cfg.seed,
                      'seam_length_m': cfg.seam_length,
                      'param_jitter': cfg.jitter,
                      'noise_std_m': cfg.noise_std,
                      'ori_noise_deg': cfg.ori_noise_deg},
        'created': datetime.now().isoformat(timespec='seconds'),
    }
    return Trajectory(meta=meta, data=data)


def next_index(shape_dir, shape):
    """기존 파일과 겹치지 않게 다음 trial 번호를 찾는다 (증분 생성 지원)."""
    pattern = re.compile(rf'^{shape}_(\d+)\.csv$')
    indices = [int(m.group(1)) for p in shape_dir.glob(f'{shape}_*.csv')
               if (m := pattern.match(p.name))]
    return max(indices, default=0) + 1


def main(argv=None):
    p = argparse.ArgumentParser(
        description='합성 위빙 궤적 생성기 — 스키마 v1 CSV trial 생성')
    p.add_argument('--shape', choices=GEN_SHAPES + ('all',), default='all')
    p.add_argument('--trials', type=int, default=1, help='shape당 trial 수')
    p.add_argument('--out-root', type=Path,
                   default=Path.home() / 'ros2_ws/data/trajectories',
                   help='배치 규약 루트 (<root>/synthetic/<shape>/)')
    p.add_argument('--seed', type=int, default=42)
    # 공칭 위빙 파라미터 (trial마다 ±jitter 비율로 변주)
    p.add_argument('--seam-length', type=float, default=0.04, help='[m]')
    p.add_argument('--amplitude', type=float, default=0.005, help='[m]')
    p.add_argument('--frequency', type=float, default=1.0, help='[Hz]')
    p.add_argument('--speed', type=float, default=0.01, help='전진 [m/s]')
    p.add_argument('--sample-rate', type=float, default=100.0, help='[Hz]')
    p.add_argument('--standoff', type=float, default=0.008,
                   help='TCP 표면 이격 z [m]')
    p.add_argument('--crescent-bulge', type=float, default=0.5,
                   help='crescent 전진 볼록량 (amplitude 비율)')
    # 시연 다양성
    p.add_argument('--jitter', type=float, default=0.08,
                   help='amplitude/frequency/speed 상대 지터 (±비율)')
    p.add_argument('--noise-std', type=float, default=0.0003,
                   help='위치 저주파 노이즈 표준편차 [m]')
    p.add_argument('--ori-noise-deg', type=float, default=1.0,
                   help='자세 흔들림 표준편차 [deg]')
    # 프레임 메타데이터 (Gazebo 월드의 workpiece 배치와 일치시킬 것)
    p.add_argument('--workpiece-parent', default='g_base')
    p.add_argument('--workpiece-xyz', type=float, nargs=3,
                   default=[0.13, -0.06, 0.05])
    p.add_argument('--workpiece-rpy', type=float, nargs=3,
                   default=[0.0, 0.0, 0.0])
    p.add_argument('--tool-frame', default='joint6_flange')
    cfg = p.parse_args((argv if argv is not None else sys.argv)[1:])

    shapes = GEN_SHAPES if cfg.shape == 'all' else (cfg.shape,)
    seed_seq = np.random.SeedSequence(cfg.seed)
    children = seed_seq.spawn(len(shapes) * cfg.trials)

    written, failed = [], 0
    for si, shape in enumerate(shapes):
        shape_dir = cfg.out_root / 'synthetic' / shape
        shape_dir.mkdir(parents=True, exist_ok=True)
        start = next_index(shape_dir, shape)
        for k in range(cfg.trials):
            rng = np.random.default_rng(children[si * cfg.trials + k])
            trial_id = f'{shape}_{start + k:04d}'
            traj = generate_trial(shape, trial_id, cfg, rng)
            issues = validate(traj)
            if issues:
                failed += 1
                print(f'{trial_id}: 생성 결과가 유효성 검사 실패 — 저장 안 함')
                for issue in issues:
                    print(f'  - {issue}')
                continue
            path = shape_dir / f'{trial_id}.csv'
            save_csv(traj, path)
            written.append(path)
            print(f'{path}  ({len(traj.data)}행, '
                  f"A={traj.meta['weaving']['amplitude_m'] * 1000:.2f}mm, "
                  f"f={traj.meta['weaving']['frequency_hz']:.2f}Hz)")

    print(f'\n생성 {len(written)}건 / 실패 {failed}건 '
          f'(shape: {", ".join(shapes)}, seed: {cfg.seed})')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
