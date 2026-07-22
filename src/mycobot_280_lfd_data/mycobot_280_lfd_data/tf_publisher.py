"""궤적 CSV를 tf2 트리에 발행 (W3): workpiece 좌표를 로봇 좌표계로 연결.

  <parent(=workpiece.parent, 예: g_base)>
      └─ static ── workpiece      (CSV 메타데이터 xyz/rpy)
             └─ dynamic ── <tcp_frame>   (CSV 각 행의 pose를 시간순 재생)

스키마의 pose는 workpiece 기준이므로 이 두 단계를 거치면 tf2가
로봇 좌표계(g_base)로의 변환을 대신 계산해준다 — W5 재생기와 W8 실측
데이터가 같은 구조를 그대로 쓴다(실측은 parent가 camera_link).

쿼터니언은 CSV가 w-first, ROS 메시지가 (x,y,z,w)로 순서가 다르다.
반드시 schema.quat_wxyz_to_msg를 거칠 것 (스키마 명세 버그 1순위 항목).

  ros2 run mycobot_280_lfd_data trajectory_tf_publisher --ros-args \
      -p csv:=<path> -p loop:=true

Gazebo와 함께 쓸 때는 반드시 -p use_sim_time:=true 를 줄 것.
gazebo_ros2_control이 /joint_states를 시뮬 시간으로 발행하고
robot_state_publisher가 그 스탬프를 그대로 전파하므로, 이 노드만 벽시계
시간을 쓰면 로봇 서브트리와 시간 기준이 어긋나 joint6_flange →
tcp_target 같은 체인 조회가 통째로 실패한다 (프레임은 존재하는데
lookup만 안 되는 형태라 원인 파악이 어려움).
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .schema import load_csv, quat_wxyz_to_msg, rpy_to_quat_wxyz, validate


def _fill_tf(tf, xyz, quat_wxyz):
    """TransformStamped에 위치·자세를 채운다 (쿼터니언 순서 변환 포함)."""
    tf.transform.translation.x = float(xyz[0])
    tf.transform.translation.y = float(xyz[1])
    tf.transform.translation.z = float(xyz[2])
    qx, qy, qz, qw = quat_wxyz_to_msg(quat_wxyz)
    tf.transform.rotation.x = float(qx)
    tf.transform.rotation.y = float(qy)
    tf.transform.rotation.z = float(qz)
    tf.transform.rotation.w = float(qw)
    return tf


class TrajectoryTfPublisher(Node):
    """CSV 궤적을 tf2로 재생하고 RViz용 Path를 함께 발행한다."""

    def __init__(self):
        super().__init__('trajectory_tf_publisher')
        self.declare_parameter('csv', '')
        self.declare_parameter('tcp_frame', 'tcp_target')
        self.declare_parameter('workpiece_frame', 'workpiece')
        self.declare_parameter('rate_scale', 1.0)
        self.declare_parameter('loop', True)
        # 여러 궤적을 겹쳐 볼 때(예: 데모 vs DMP 재현) parent→workpiece는
        # 하나만 발행해야 한다 — 같은 엣지를 두 노드가 잡으면 static tf
        # 권한이 충돌해 경고가 뜬다.
        self.declare_parameter('publish_static', True)

        csv_path = self.get_parameter('csv').value
        if not csv_path:
            raise RuntimeError('파라미터 csv(궤적 파일 경로)가 필요합니다')
        self.tcp_frame = self.get_parameter('tcp_frame').value
        self.workpiece_frame = self.get_parameter('workpiece_frame').value
        rate_scale = self.get_parameter('rate_scale').value
        self.loop = self.get_parameter('loop').value

        self.traj = load_csv(csv_path)
        issues = validate(self.traj)
        if issues:
            raise RuntimeError('궤적 유효성 검사 실패: ' + '; '.join(issues))
        meta = self.traj.meta

        # parent → workpiece: 메타데이터의 배치. 재생 중 불변이므로 static.
        if self.get_parameter('publish_static').value:
            self.static_bc = StaticTransformBroadcaster(self)
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = meta['workpiece']['parent']
            static_tf.child_frame_id = self.workpiece_frame
            self.static_bc.sendTransform(_fill_tf(
                static_tf, meta['workpiece']['xyz'],
                rpy_to_quat_wxyz(meta['workpiece']['rpy'])))

        self.bc = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '~/path', 1)
        self.path_msg = self._build_path()

        self.index = 0
        self.period = 1.0 / (meta['sample_rate_hz'] * rate_scale)
        period = self.period
        self.timer = self.create_timer(period, self.on_timer)
        # Path는 정적 데이터이므로 저주기로만 재발행 (RViz 늦은 구독 대비)
        self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f"'{meta['trial_id']}' ({meta['shape']}, {len(self.traj.data)}행) "
            f"재생: {meta['workpiece']['parent']} → {self.workpiece_frame} "
            f'→ {self.tcp_frame}, {1.0 / period:.1f} Hz, loop={self.loop}')

    def _build_path(self):
        """궤적 전체를 workpiece 프레임 Path로 (RViz 시각화용, 1회 생성)."""
        msg = Path()
        msg.header.frame_id = self.workpiece_frame
        for pos, quat in zip(self.traj.positions, self.traj.quaternions_wxyz):
            pose = PoseStamped()
            pose.header.frame_id = self.workpiece_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                float(v) for v in pos)
            qx, qy, qz, qw = quat_wxyz_to_msg(quat)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            msg.poses.append(pose)
        return msg

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def on_timer(self):
        n = len(self.traj.data)
        if self.loop:
            # 재생 위치를 카운터가 아니라 절대 시각에서 계산한다. 그래야
            # 같은 궤적을 여러 노드가 동시에 재생할 때(데모 vs DMP 재현)
            # 시작 시각이 달라도, 타이머가 밀려도 위상이 어긋나지 않는다.
            # 카운터 방식이면 두 마커가 서로 다른 시점을 가리켜 추종 오차를
            # 실제보다 크게 보이게 만든다.
            now = self.get_clock().now().nanoseconds * 1e-9
            self.index = int(now / self.period) % n
        elif self.index >= n:
            self.get_logger().info('재생 완료')
            self.timer.cancel()
            return

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.workpiece_frame
        tf.child_frame_id = self.tcp_frame
        self.bc.sendTransform(_fill_tf(tf, self.traj.positions[self.index],
                                       self.traj.quaternions_wxyz[self.index]))
        if not self.loop:  # loop일 때는 매 틱 시각에서 다시 계산한다
            self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
