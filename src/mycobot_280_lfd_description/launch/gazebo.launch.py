"""myCobot 280을 Gazebo Classic에 스폰하고 ros2_control 컨트롤러를 기동.

  ros2 launch mycobot_280_lfd_description gazebo.launch.py
  ros2 launch mycobot_280_lfd_description gazebo.launch.py gui:=false  # 헤드리스
"""
import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_280_lfd_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mycobot_280_m5_gazebo.urdf.xacro')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Gazebo Classic은 package:// 메시 URI를 GAZEBO_MODEL_PATH에서 찾는다.
    # mycobot_description의 share 부모 디렉터리를 등록해야 메시가 렌더링됨.
    gazebo_model_path = os.path.join(
        get_package_prefix('mycobot_description'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GAZEBO_MODEL_PATH']

    gui = LaunchConfiguration('gui')

    # inertia_scale=100: Gazebo Classic에서 저관성(1e-5 kg·m²) 체인의
    # 수치 불안정(진동/NaN) 회피. 질량·운동학은 실측값 유지.
    # effort_scale은 반드시 inertia_scale과 동일 배율 (가속 능력 보존).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' inertia_scale:=100.0 effort_scale:=100.0']),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='gzclient(GUI) 실행 여부'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={'gui': gui}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'mycobot_280'],
            output='screen',
        ),

        # controller_manager는 gazebo 플러그인 내부에서 뜨므로 spawner만 실행
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller']),
    ])
