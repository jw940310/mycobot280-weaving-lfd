"""myCobot 280 MoveIt2 데모: move_group + fake controller(ros2_control mock) + RViz2.

  ros2 launch mycobot_280_lfd_moveit_config demo.launch.py
  ros2 launch mycobot_280_lfd_moveit_config demo.launch.py rviz:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


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

    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='RViz2(MotionPlanning) 실행 여부'),

        # --- fake controller 스택 ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                robot_description,
                os.path.join(desc_pkg, 'config', 'controllers.yaml'),
            ],
            output='both',
        ),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller']),

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
            ],
            condition=IfCondition(rviz),
        ),
    ])
