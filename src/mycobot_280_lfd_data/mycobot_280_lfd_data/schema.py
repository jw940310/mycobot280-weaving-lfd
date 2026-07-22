"""LfD 궤적 CSV 스키마 v1 참조 구현 (명세: docs/trajectory_csv_schema.md).

포맷: `# ` 접두 YAML 메타데이터 블록 + 고정 컬럼 CSV.
쿼터니언은 w-first (pytransform3d/movement_primitives 관례) — ROS msg의
(x,y,z,w)와 다르므로 경계에서는 반드시 quat_*_wxyz 헬퍼를 쓸 것.
"""
import sys
from dataclasses import dataclass, field

import numpy as np
import yaml

SCHEMA_NAME = 'mycobot_lfd_trajectory'
SCHEMA_VERSION = 1
COLUMNS = ('t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')

SHAPES = ('line', 'zigzag', 'crescent', 'freeform')
SOURCES = ('synthetic', 'human_demo', 'robot_replay')

REQUIRED_META = ('schema', 'version', 'trial_id', 'shape', 'source',
                 'frames', 'workpiece', 'sample_rate_hz')

# 위치 크기 상식 한계 [m] — myCobot 280 작업공간 스케일
POSITION_NORM_LIMIT = 0.5
QUAT_NORM_TOL = 1e-3
DT_JITTER_RATIO = 0.5
MIN_ROWS = 10


@dataclass
class Trajectory:
    """단일 trial: 메타데이터 + (N×8) 데이터 행렬 (컬럼 순서 = COLUMNS)."""
    meta: dict
    data: np.ndarray = field(repr=False)

    @property
    def t(self):
        return self.data[:, 0]

    @property
    def positions(self):
        """(N×3) TCP 위치 [m], workpiece 프레임."""
        return self.data[:, 1:4]

    @property
    def quaternions_wxyz(self):
        """(N×4) TCP 자세 쿼터니언, w-first."""
        return self.data[:, 4:8]


def quat_wxyz_to_msg(q):
    """(qw,qx,qy,qz) → geometry_msgs/Quaternion 순서 (x,y,z,w) 튜플."""
    w, x, y, z = q
    return (x, y, z, w)


def quat_msg_to_wxyz(q):
    """geometry_msgs 순서 (x,y,z,w) → 스키마 순서 (qw,qx,qy,qz) 튜플."""
    x, y, z, w = q
    return (w, x, y, z)


def rpy_to_quat_wxyz(rpy):
    """workpiece.rpy 메타데이터 [rad] → (qw,qx,qy,qz).

    URDF/SDF와 동일한 고정축 RPY: R = Rz(yaw)·Ry(pitch)·Rx(roll).
    """
    cr, sr = np.cos(rpy[0] / 2), np.sin(rpy[0] / 2)
    cp, sp = np.cos(rpy[1] / 2), np.sin(rpy[1] / 2)
    cy, sy = np.cos(rpy[2] / 2), np.sin(rpy[2] / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def save_csv(traj, path):
    """Trajectory를 스키마 v1 CSV로 저장."""
    meta_yaml = yaml.safe_dump(traj.meta, allow_unicode=True,
                               default_flow_style=None, sort_keys=False)
    with open(path, 'w', encoding='utf-8') as f:
        for line in meta_yaml.rstrip('\n').split('\n'):
            f.write(f'# {line}\n')
        f.write(','.join(COLUMNS) + '\n')
        for row in traj.data:
            f.write('%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n' % tuple(row))


def load_csv(path):
    """스키마 v1 CSV → Trajectory. 형식이 깨졌으면 ValueError."""
    meta_lines, table_lines = [], []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.rstrip('\n')
            if line.startswith('#'):
                if table_lines:
                    raise ValueError('메타데이터 블록은 파일 선두에만 허용')
                meta_lines.append(line[1:].removeprefix(' '))
            elif line.strip():
                table_lines.append(line)
    if not meta_lines:
        raise ValueError('메타데이터 블록(# ...) 없음')
    if not table_lines:
        raise ValueError('데이터 행 없음')

    meta = yaml.safe_load('\n'.join(meta_lines))
    header = tuple(h.strip() for h in table_lines[0].split(','))
    if header != COLUMNS:
        raise ValueError(f'컬럼 헤더 불일치: {header} != {COLUMNS}')
    data = np.array([[float(v) for v in row.split(',')]
                     for row in table_lines[1:]])
    if data.ndim != 2 or data.shape[1] != len(COLUMNS):
        raise ValueError(f'데이터 형상 오류: {data.shape}')
    return Trajectory(meta=meta, data=data)


def with_lead_in(traj, hold_s):
    """궤적 앞에 첫 포즈를 hold_s초 동안 유지하는 정지 구간을 덧붙인다.

    용접 아크 스타트의 dwell에 해당하며, 시작 속도를 0으로 만든다. 위빙
    데모는 위상 중간(=횡속도 최대)에서 출발하는데, DMP는 시작 속도를 0으로
    강제하므로 그대로 학습시키면 초반에 큰 기동 과도응답이 생긴다. 정지
    구간을 앞에 두면 그 불일치가 사라진다 (합성 데모에서 초기 20% RMSE가
    3.5mm→0.2mm로 줄어드는 것을 확인).

    원본 샘플의 타이밍(지터 포함)은 그대로 보존하고 뒤로 밀며, 삽입한 정지
    구간만 균일 dt로 채운다. t는 다시 0부터. 새 Trajectory를 반환한다.
    """
    if hold_s <= 0.0:
        return traj
    dt = float(np.median(np.diff(traj.t)))
    n_hold = int(round(hold_s / dt))
    if n_hold <= 0:
        return traj

    lead_dur = n_hold * dt
    hold = np.tile(traj.data[0].copy(), (n_hold, 1))
    hold[:, 0] = np.arange(n_hold) * dt            # 0 .. (n_hold-1)*dt
    shifted = traj.data.copy()
    shifted[:, 0] = shifted[:, 0] + lead_dur       # 원래 타이밍 유지, 뒤로 밀기

    meta = dict(traj.meta)
    meta['lead_in_s'] = round(lead_dur, 4)
    return Trajectory(meta=meta, data=np.vstack([hold, shifted]))


def validate(traj):
    """유효성 규칙 검사. 위반 목록(list[str]) 반환 — 빈 리스트면 통과."""
    issues = []
    m = traj.meta

    for key in REQUIRED_META:
        if key not in m:
            issues.append(f'필수 메타데이터 누락: {key}')
    if m.get('schema') != SCHEMA_NAME:
        issues.append(f"schema != '{SCHEMA_NAME}': {m.get('schema')!r}")
    if m.get('version') != SCHEMA_VERSION:
        issues.append(f'version != {SCHEMA_VERSION}: {m.get("version")!r}')
    if 'shape' in m and m['shape'] not in SHAPES:
        issues.append(f'shape 허용값 아님: {m["shape"]!r} (허용: {SHAPES})')
    if 'source' in m and m['source'] not in SOURCES:
        issues.append(f'source 허용값 아님: {m["source"]!r} (허용: {SOURCES})')
    for parent_key, sub in (('frames', ('reference', 'tool')),
                            ('workpiece', ('parent', 'xyz', 'rpy'))):
        if isinstance(m.get(parent_key), dict):
            for k in sub:
                if k not in m[parent_key]:
                    issues.append(f'메타데이터 누락: {parent_key}.{k}')

    d = traj.data
    if len(d) < MIN_ROWS:
        issues.append(f'행 수 부족: {len(d)} < {MIN_ROWS}')
        return issues  # 이하 통계 검사는 무의미
    if not np.isfinite(d).all():
        issues.append('NaN/Inf 존재')
        return issues

    t = traj.t
    if abs(t[0]) > 1e-9:
        issues.append(f't[0] != 0: {t[0]}')
    dt = np.diff(t)
    if (dt <= 0).any():
        issues.append('t가 순증가가 아님')
    else:
        med = np.median(dt)
        jitter = np.abs(dt - med).max() / med
        if jitter >= DT_JITTER_RATIO:
            issues.append(f'dt 지터 과다: {jitter:.2f} >= {DT_JITTER_RATIO}')

    qnorm = np.linalg.norm(traj.quaternions_wxyz, axis=1)
    bad = np.abs(qnorm - 1.0) > QUAT_NORM_TOL
    if bad.any():
        issues.append(f'쿼터니언 노름 이탈 {bad.sum()}행 '
                      f'(최대 편차 {np.abs(qnorm - 1).max():.2e})')

    pnorm = np.linalg.norm(traj.positions, axis=1)
    if (pnorm >= POSITION_NORM_LIMIT).any():
        issues.append(f'위치 크기 한계 초과: max {pnorm.max():.3f} m '
                      f'>= {POSITION_NORM_LIMIT}')
    return issues


def main(argv=None):
    """CLI: validate_trajectory <file.csv> [...] — 전부 통과 시 exit 0."""
    args = (argv if argv is not None else sys.argv)[1:]
    if not args:
        print('usage: validate_trajectory <file.csv> [...]', file=sys.stderr)
        return 2
    failed = False
    for path in args:
        try:
            issues = validate(load_csv(path))
        except (OSError, ValueError, yaml.YAMLError) as e:
            print(f'{path}: LOAD ERROR — {e}')
            failed = True
            continue
        if issues:
            failed = True
            print(f'{path}: FAIL ({len(issues)})')
            for i in issues:
                print(f'  - {i}')
        else:
            print(f'{path}: OK')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
