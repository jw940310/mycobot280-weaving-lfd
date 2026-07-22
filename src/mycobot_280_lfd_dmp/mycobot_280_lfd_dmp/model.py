"""학습된 DMP 위빙 스킬의 영속화 (W4 07.23).

한 shape(line/zigzag/crescent)의 여러 시연에서 학습한 프로토타입 DMP를
파일 하나로 저장/복원한다. movement_primitives 객체를 그대로 pickle하지
않는 이유: 라이브러리 버전에 묶이지 않게, 그리고 사람이 열어볼 수 있게
하려고 — 저장 포맷은 순수 배열(.npz) + JSON 메타데이터다.

핵심 불변식: CartesianDMP의 forcing weights는 위상(정준계 s: 1→0) 공간에서
정의되므로 execution_time과 분리돼 있다. 그래서 서로 다른 시간 길이의
시연들에서 뽑은 가중치를 평균해 하나의 프로토타입으로 묶을 수 있고
(train.py), 재생 시 execution_time을 바꿔도 형태가 보존된다.
"""
import json
from dataclasses import dataclass, field

import numpy as np
from movement_primitives.dmp import CartesianDMP

MODEL_SCHEMA = 'mycobot_lfd_dmp_skill'
MODEL_VERSION = 1

# CartesianDMP forcing term은 위치 3 + 회전 3 = 6축에 대해 정의된다.
DMP_DIMS = 6
POSE_DIM = 7  # x, y, z, qw, qx, qy, qz (w-first)


@dataclass
class DMPSkill:
    """한 shape의 프로토타입 DMP 스킬. to_dmp()로 실행 가능한 DMP로 되살린다."""
    shape: str
    n_weights_per_dim: int
    execution_time: float
    dt: float
    alpha_y: float
    beta_y: float
    start_y: np.ndarray               # (7,) w-first
    goal_y: np.ndarray                # (7,) w-first
    weights: np.ndarray               # flat (DMP_DIMS * n_weights_per_dim,)
    meta: dict = field(default_factory=dict)  # 출처: n_demos, source_trials 등

    def to_dmp(self):
        """실행 가능한 CartesianDMP로 복원 (가중치 주입 + 시작/끝점 설정).

        imitate를 거치지 않으므로 학습 데이터 없이도 재생할 수 있다.
        가중치 주입 재생이 원본 학습 DMP와 비트 단위로 일치함을 확인했다.
        """
        dmp = CartesianDMP(execution_time=float(self.execution_time),
                           dt=float(self.dt),
                           n_weights_per_dim=int(self.n_weights_per_dim),
                           alpha_y=float(self.alpha_y),
                           beta_y=float(self.beta_y))
        dmp.set_weights(np.asarray(self.weights, float))
        dmp.configure(start_y=np.asarray(self.start_y, float),
                      goal_y=np.asarray(self.goal_y, float))
        return dmp

    def save(self, path):
        """.npz로 저장 (배열 + 스키마/출처는 JSON 문자열로 동봉)."""
        header = {'schema': MODEL_SCHEMA, 'version': MODEL_VERSION,
                  'shape': self.shape,
                  'n_weights_per_dim': int(self.n_weights_per_dim),
                  'execution_time': float(self.execution_time),
                  'dt': float(self.dt),
                  'alpha_y': float(self.alpha_y),
                  'beta_y': float(self.beta_y),
                  'meta': self.meta}
        np.savez(path, header=json.dumps(header, ensure_ascii=False),
                 start_y=np.asarray(self.start_y, float),
                 goal_y=np.asarray(self.goal_y, float),
                 weights=np.asarray(self.weights, float))

    @classmethod
    def load(cls, path):
        """save()로 저장한 .npz를 DMPSkill로 복원. 스키마 불일치면 ValueError."""
        with np.load(path, allow_pickle=False) as z:
            header = json.loads(str(z['header']))
            if header.get('schema') != MODEL_SCHEMA:
                raise ValueError(f"DMP 스킬 파일이 아님: schema={header.get('schema')!r}")
            if header.get('version') != MODEL_VERSION:
                raise ValueError(f"지원하지 않는 모델 버전: {header.get('version')!r}")
            weights = z['weights']
            expected = DMP_DIMS * header['n_weights_per_dim']
            if weights.shape != (expected,):
                raise ValueError(f'가중치 형상 오류: {weights.shape} != ({expected},)')
            return cls(shape=header['shape'],
                       n_weights_per_dim=header['n_weights_per_dim'],
                       execution_time=header['execution_time'],
                       dt=header['dt'],
                       alpha_y=header['alpha_y'],
                       beta_y=header['beta_y'],
                       start_y=z['start_y'],
                       goal_y=z['goal_y'],
                       weights=weights,
                       meta=header.get('meta', {}))
