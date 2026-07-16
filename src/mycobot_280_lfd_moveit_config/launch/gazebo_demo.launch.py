"""myCobot 280 MoveIt2 + Gazebo 디지털 트윈: move_group이 Gazebo 물리
시뮬레이션의 arm_controller로 궤적을 실행한다 (M1: 가상 Trajectory Planning).

  ros2 launch mycobot_280_lfd_moveit_config gazebo_demo.launch.py
  ros2 launch mycobot_280_lfd_moveit_config gazebo_demo.launch.py gui:=false rviz:=false

- 컨트롤러 스택(RSP·spawn·spawner)은 description의 gazebo.launch.py가 담당
- move_group의 robot_description은 demo.launch.py와 동일한 system xacro
  (SRDF virtual joint world→g_base와 정합; ros2_control 태그는 MoveIt이 무시)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def load_yaml(package, relpath):
    path = os.path.join(get_package_share_directory(package), relpath)
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_280_lfd_description')
    cfg_pkg = get_package_share_directory('mycobot_280_lfd_moveit_config')

    robot_description = {
        'robot_description': xacro.process_file(
            os.path.join(desc_pkg, 'urdf', 'mycobot_280_m5_system.urdf.xacro')
        ).toxml()
    }
    with open(os.path.join(cfg_pkg, 'srdf', 'mycobot_280.srdf'), encoding='utf-8') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics = {
        'robot_description_kinematics': load_yaml(
            'mycobot_280_lfd_moveit_config', 'config/kinematics.yaml')
    }
    joint_limits = {
        'robot_description_planning': load_yaml(
            'mycobot_280_lfd_moveit_config', 'config/joint_limits.yaml')
    }

    ompl_pipeline = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': ' '.join([
            'default_planner_request_adapters/AddTimeOptimalParameterization',
            'default_planner_request_adapters/FixWorkspaceBounds',
            'default_planner_request_adapters/FixStartStateBounds',
            'default_planner_request_adapters/FixStartStateCollision',
            'default_planner_request_adapters/FixStartStatePathConstraints',
        ]),
        'start_state_max_bounds_error': 0.1,
    }
    ompl_pipeline.update(
        load_yaml('mycobot_280_lfd_moveit_config', 'config/ompl_planning.yaml'))
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_pipeline,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml(
            'mycobot_280_lfd_moveit_config',
            'config/moveit_controllers.yaml')['moveit_simple_controller_manager'],
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='gzclient(GUI) 실행 여부'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz2(MotionPlanning) 실행 여부'),

        # --- Gazebo 물리 + ros2_control 스택 (RSP/spawn/spawner 포함) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'gui': gui}.items(),
        ),

        # --- move_group ---
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipelines,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor,
                # Gazebo는 /clock을 발행 — 궤적 타임스탬프 정합에 필수
                {'use_sim_time': True},
            ],
        ),

        # --- RViz (MotionPlanning) ---
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(cfg_pkg, 'launch', 'moveit.rviz')],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                {'use_sim_time': True},
            ],
            condition=IfCondition(rviz),
        ),
    ])
