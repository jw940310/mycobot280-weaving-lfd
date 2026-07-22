#!/usr/bin/env bash
# W4 DMP 파이썬 의존성 설치 (pip 없는 환경용).
#
# 이 머신에는 python3-pip이 없고 sudo도 세션 내에서 쓸 수 없어서,
# PyPI 배포판을 직접 받아 사용자 site-packages에 푼다.
#   - pytransform3d: wheel(py3-none-any) → 그대로 unzip
#   - movement_primitives: sdist만 배포 → 순수 파이썬 패키지 디렉터리만 복사
#     (선택적 Cython 가속 모듈 dmp_fast는 빌드하지 않음. 미존재 시 라이브러리가
#      python 스텝 함수로 자동 폴백하며, 위빙 궤적 규모(~400행)에서는 1초 내외.)
#
# 재실행 안전(idempotent). 성공 시 버전과 스모크 테스트 결과를 출력한다.
set -euo pipefail

PYTRANSFORM3D_VERSION=3.15.0
MOVEMENT_PRIMITIVES_VERSION=0.9.1

SITE_PACKAGES="$(python3 -c 'import site; print(site.getusersitepackages())')"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

echo "설치 대상: $SITE_PACKAGES"
mkdir -p "$SITE_PACKAGES"

fetch() {  # fetch <pypi-패키지명> <packagetype> <버전>
  local name=$1 ptype=$2 version=$3
  local url
  url=$(curl -fsSL "https://pypi.org/pypi/${name}/${version}/json" | python3 -c "
import json, sys
for f in json.load(sys.stdin)['urls']:
    if f['packagetype'] == '$ptype':
        print(f['url']); break
else:
    sys.exit('$name ${version}: $ptype 배포판 없음')
")
  # 진행 메시지는 stderr로 — stdout은 호출부가 파일명으로 캡처한다
  echo "  받는 중: $(basename "$url")" >&2
  curl -fsSL -o "$WORK_DIR/$(basename "$url")" "$url"
  basename "$url"
}

echo "[1/3] pytransform3d ${PYTRANSFORM3D_VERSION}"
WHEEL=$(fetch pytransform3d bdist_wheel "$PYTRANSFORM3D_VERSION")
python3 -c "
import zipfile
zipfile.ZipFile('$WORK_DIR/$WHEEL').extractall('$SITE_PACKAGES')
"

echo "[2/3] movement_primitives ${MOVEMENT_PRIMITIVES_VERSION}"
SDIST=$(fetch movement-primitives sdist "$MOVEMENT_PRIMITIVES_VERSION")
tar xzf "$WORK_DIR/$SDIST" -C "$WORK_DIR"
rm -rf "$SITE_PACKAGES/movement_primitives"
cp -r "$WORK_DIR/movement_primitives-${MOVEMENT_PRIMITIVES_VERSION}/movement_primitives" \
      "$SITE_PACKAGES/"

echo "[3/3] 검증"
python3 - <<'PY'
import numpy as np
import pytransform3d
import movement_primitives
from movement_primitives.dmp import CartesianDMP

print(f'  pytransform3d        {pytransform3d.__version__}')
print(f'  movement_primitives  {movement_primitives.__version__}')

T = np.linspace(0.0, 1.0, 101)
Y = np.column_stack([np.sin(2 * np.pi * T), np.cos(2 * np.pi * T), T,
                     np.tile([1.0, 0.0, 0.0, 0.0], (len(T), 1))])
dmp = CartesianDMP(execution_time=1.0, dt=0.01, n_weights_per_dim=10)
dmp.imitate(T, Y)
_, Y_repro = dmp.open_loop()
assert Y_repro.shape == (101, 7), Y_repro.shape
print('  CartesianDMP imitate/open_loop 스모크 테스트 통과')
PY

echo
echo "완료. 상세 검증: ros2 run mycobot_280_lfd_dmp check_dmp_env"
