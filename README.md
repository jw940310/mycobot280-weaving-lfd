# myCobot 280 위빙 궤적 LfD 파이프라인

Ubuntu 22.04 · ROS2 Humble · MoveIt2 · Gazebo · movement_primitives(DMP) — 시뮬레이션 퍼스트.
주차별 마일스톤은 [docs/schedule.md](docs/schedule.md) 참고.

## 워크스페이스 복원

외부 패키지(`mycobot_ros2`, `xacro`)는 커밋에 포함하지 않고 `src/ros2.repos`에 핀 고정해 두었다.

```bash
cd ~/ros2_ws
vcs import src < src/ros2.repos
git -C src/mycobot_ros2 apply ../../patches/mycobot_description-setup_cfg.patch
colcon build --symlink-install
source install/setup.bash
```

`patches/mycobot_description-setup_cfg.patch`는 setuptools 최신 버전의
dash-키(`script-dir`, `install-scripts`) 폐기 경고를 underscore 표기로 고치는 로컬 수정이다.
