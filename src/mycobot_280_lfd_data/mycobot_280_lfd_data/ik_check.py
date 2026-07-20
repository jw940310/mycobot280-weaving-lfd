"""궤적 waypoint의 도달 가능성 검사 (W3): MoveIt /compute_ik로 풀어본다.

CSV pose는 workpiece 기준이므로 메타데이터의 workpiece.xyz를 더해
로봇 좌표계(기본: g_base)로 옮긴 뒤 IK를 건다. rpy가 0이 아닌 배치는
아직 지원하지 않는다(회전 합성 필요) — 그런 CSV는 건너뛰고 경고한다.

move_group이 떠 있어야 한다:
  ros2 launch mycobot_280_lfd_moveit_config gazebo_demo.launch.py

  ros2 run mycobot_280_lfd_data check_ik --stride 8
"""
import argparse
import sys
from pathlib import Path

import rclpy
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node

from .schema import load_csv

# MoveIt error_code.val == 1 (SUCCESS). 그 외는 실패 사유별 음수 코드.
SUCCESS = 1


class IkChecker(Node):
    """/compute_ik 서비스를 감싸는 최소 클라이언트."""

    def __init__(self, group, ik_link, timeout):
        super().__init__('ik_check')
        self.group = group
        self.ik_link = ik_link
        self.timeout = timeout
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        if not self.cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(
                '/compute_ik 서비스를 찾을 수 없습니다 — move_group이 떠 있나요?')

    def solve(self, frame, xyz, quat_wxyz):
        """IK 1회. 성공이면 SUCCESS, 실패면 MoveIt 코드, 무응답이면 None."""
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = self.group
        ik.ik_link_name = self.ik_link
        ik.avoid_collisions = True
        ik.timeout.sec = 1

        ps = ik.pose_stamped
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
            float(v) for v in xyz)
        qw, qx, qy, qz = (float(v) for v in quat_wxyz)
        ps.pose.orientation.w = qw
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout)
        if not fut.done():
            return None
        return fut.result().error_code.val


def check_file(node, path, stride):
    """CSV 1개를 검사해 (성공, 시도, {실패코드: 횟수}) 반환."""
    traj = load_csv(path)
    wp = traj.meta['workpiece']
    if any(abs(v) > 1e-9 for v in wp['rpy']):
        print(f'  {path.name}: workpiece.rpy가 0이 아니라 건너뜀 {wp["rpy"]}')
        return 0, 0, {}

    ok = n = 0
    codes = {}
    for pos, quat in zip(traj.positions[::stride],
                         traj.quaternions_wxyz[::stride]):
        xyz = [wp['xyz'][i] + pos[i] for i in range(3)]
        code = node.solve(wp['parent'], xyz, quat)
        n += 1
        if code == SUCCESS:
            ok += 1
        else:
            codes[code] = codes.get(code, 0) + 1
    return ok, n, codes


def main(argv=None):
    p = argparse.ArgumentParser(
        description='궤적 waypoint 도달 가능성 검사 — MoveIt /compute_ik')
    p.add_argument('--root', type=Path,
                   default=Path.home() / 'ros2_ws/data/trajectories/synthetic',
                   help='검사할 CSV 루트 (<root>/<shape>/*.csv)')
    p.add_argument('--shape', default='all',
                   help='검사할 shape 또는 all')
    p.add_argument('--trials', type=int, default=3,
                   help='shape당 검사할 파일 수')
    p.add_argument('--stride', type=int, default=8,
                   help='waypoint 샘플링 간격 (1이면 전수 검사)')
    p.add_argument('--group', default='arm', help='SRDF planning group')
    p.add_argument('--ik-link', default='joint6_flange',
                   help='IK를 풀 링크 (CSV frames.tool과 일치해야 함)')
    p.add_argument('--call-timeout', type=float, default=5.0,
                   help='서비스 응답 대기 [s]')
    cfg = p.parse_args((argv if argv is not None else sys.argv)[1:])

    shapes = ([d.name for d in sorted(cfg.root.iterdir()) if d.is_dir()]
              if cfg.shape == 'all' else [cfg.shape])

    rclpy.init()
    node = IkChecker(cfg.group, cfg.ik_link, cfg.call_timeout)
    total_ok = total_n = 0
    try:
        print(f'waypoint {cfg.stride}개마다 1개씩 샘플링, '
              f'group={cfg.group}, ik_link={cfg.ik_link}\n')
        for shape in shapes:
            files = sorted((cfg.root / shape).glob('*.csv'))[:cfg.trials]
            ok = n = 0
            codes = {}
            for path in files:
                f_ok, f_n, f_codes = check_file(node, path, cfg.stride)
                ok += f_ok
                n += f_n
                for k, v in f_codes.items():
                    codes[k] = codes.get(k, 0) + v
            total_ok += ok
            total_n += n
            extra = f'  실패: {codes}' if codes else ''
            print(f'  {shape:10s} {ok:4d}/{n:4d} 성공{extra}')

        rate = 100.0 * total_ok / max(total_n, 1)
        print(f'\n합계 {total_ok}/{total_n} ({rate:.1f}%)')
        print('※ 실패코드 None은 IK 실패가 아니라 서비스 응답 지연 '
              '— --call-timeout을 늘려 재확인할 것')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if total_ok == total_n else 1


if __name__ == '__main__':
    sys.exit(main())
