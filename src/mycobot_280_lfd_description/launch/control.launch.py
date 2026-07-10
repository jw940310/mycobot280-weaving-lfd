"""myCobot 280 fake controller(ros2_control mock hardware) 기동.

  ros2 launch mycobot_280_lfd_description control.launch.py
  ros2 launch mycobot_280_lfd_description control.launch.py rviz:=false
  # 추후 실기 전환: hardware_plugin:=<pymycobot 어댑터 플러그인>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_280_lfd_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mycobot_280_m5_system.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    hardware_plugin = LaunchConfiguration('hardware_plugin')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' hardware_plugin:=', hardware_plugin]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_plugin', default_value='mock_components/GenericSystem',
            description='ros2_control 하드웨어 플러그인 (실기 전환 시 교체)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='RViz2 실행 여부'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                        controllers_yaml],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
