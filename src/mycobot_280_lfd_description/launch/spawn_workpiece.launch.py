"""가상 Workpiece(작업대+시편) 오브젝트를 실행 중인 Gazebo 월드에 스폰.

  ros2 launch mycobot_280_lfd_description spawn_workpiece.launch.py

- pose 기본값(x/y/z/roll/pitch/yaw, reference_frame)은 trajectory CSV
  스키마의 workpiece 메타데이터 기본값(docs/trajectory_csv_schema.md,
  synth.py --workpiece-xyz/--workpiece-rpy)과 반드시 일치시킬 것 —
  어긋나면 재생 궤적이 시편 표면과 정렬되지 않는다.
- `-reference_frame`으로 스폰하므로 대상 프레임(기본 g_base)이 이미
  Gazebo에 존재해야 한다. gazebo.launch.py가 로봇 스폰 완료 후
  (OnProcessExit) 이 launch를 include해 순서를 보장한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_280_lfd_description')
    model_path = os.path.join(pkg_share, 'models', 'lfd_workpiece', 'model.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'reference_frame', default_value='g_base',
            description='pose 기준 프레임(로봇 링크). trajectory CSV의 '
                        'workpiece.parent와 일치해야 함'),
        DeclareLaunchArgument('x', default_value='0.13', description='[m]'),
        DeclareLaunchArgument('y', default_value='-0.06', description='[m]'),
        DeclareLaunchArgument('z', default_value='0.05', description='[m]'),
        DeclareLaunchArgument('roll', default_value='0.0', description='[rad]'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='[rad]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='[rad]'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', model_path,
                '-entity', 'lfd_workpiece',
                '-reference_frame', LaunchConfiguration('reference_frame'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('roll'),
                '-P', LaunchConfiguration('pitch'),
                '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),
    ])
