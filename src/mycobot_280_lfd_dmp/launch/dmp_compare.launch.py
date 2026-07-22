"""데모 궤적과 DMP 재현 궤적을 RViz에서 겹쳐 보는 launch (W4 07.22).

DMP 재현 CSV가 없으면 먼저 만들 것:
  ros2 run mycobot_280_lfd_dmp dmp_reproduce \\
      ~/ros2_ws/data/trajectories/synthetic/line/line_0001.csv \\
      -o ~/ros2_ws/data/trajectories/dmp/line_0001_dmp.csv

  ros2 launch mycobot_280_lfd_dmp dmp_compare.launch.py \\
      demo_csv:=<데모.csv> dmp_csv:=<재현.csv>

파랑=데모, 주황=DMP 재현. 두 궤적을 각각 tcp_demo / tcp_dmp 프레임으로
동시에 재생하므로 축 마커가 벌어지는 정도로 추종 오차가 눈에 보인다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DATA_ROOT = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'data',
                         'trajectories')


def generate_launch_description():
    desc_share = get_package_share_directory('mycobot_280_lfd_description')
    dmp_share = get_package_share_directory('mycobot_280_lfd_dmp')
    xacro_file = os.path.join(desc_share, 'urdf', 'mycobot_280_m5.urdf.xacro')
    rviz_config = os.path.join(dmp_share, 'rviz', 'dmp_compare.rviz')

    demo_csv = LaunchConfiguration('demo_csv')
    dmp_csv = LaunchConfiguration('dmp_csv')
    rate_scale = LaunchConfiguration('rate_scale')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'demo_csv',
            default_value=os.path.join(DATA_ROOT, 'synthetic', 'line',
                                       'line_0001.csv'),
            description='데모(원본) 궤적 CSV'),
        DeclareLaunchArgument(
            'dmp_csv',
            default_value=os.path.join(DATA_ROOT, 'dmp', 'line_0001_dmp.csv'),
            description='DMP 재현 궤적 CSV'),
        DeclareLaunchArgument(
            'rate_scale', default_value='1.0',
            description='재생 속도 배율 (느리게 보려면 0.3 등)'),

        # 로봇 모델은 위치 참조용 — 조인트는 움직이지 않는다(W5에서 연결).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        # 데모 쪽만 parent→workpiece static을 발행한다 (권한 충돌 방지).
        Node(
            package='mycobot_280_lfd_data',
            executable='trajectory_tf_publisher',
            name='demo_trajectory',
            parameters=[{'csv': demo_csv,
                         'tcp_frame': 'tcp_demo',
                         'rate_scale': rate_scale,
                         'publish_static': True,
                         'loop': True}],
        ),
        Node(
            package='mycobot_280_lfd_data',
            executable='trajectory_tf_publisher',
            name='dmp_trajectory',
            parameters=[{'csv': dmp_csv,
                         'tcp_frame': 'tcp_dmp',
                         'rate_scale': rate_scale,
                         'publish_static': False,
                         'loop': True}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
