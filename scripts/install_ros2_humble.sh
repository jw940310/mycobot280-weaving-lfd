#!/usr/bin/env bash
# ROS2 Humble 설치 스크립트 (Ubuntu 22.04 / Jammy)
# 공식 문서 기준: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
set -euo pipefail

echo "=== [1/7] UTF-8 로케일 확인 ==="
locale

echo "=== [2/7] Ubuntu Universe 저장소 활성화 ==="
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "=== [3/7] ROS2 apt 저장소 등록 (ros2-apt-source) ==="
sudo apt update
sudo apt install -y curl
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

echo "=== [4/7] 패키지 목록 갱신 및 시스템 업그레이드 ==="
sudo apt update
sudo apt upgrade -y

echo "=== [5/7] ROS2 Humble Desktop 설치 ==="
sudo apt install -y ros-humble-desktop

echo "=== [6/7] 개발 도구(ros-dev-tools) 설치 ==="
sudo apt install -y ros-dev-tools

echo "=== [7/7] 설치 확인 ==="
source /opt/ros/humble/setup.bash
ros2 --version || true
printenv | grep -i ROS || true
dpkg -l | grep ros-humble-desktop || true

echo "=== 설치 완료 ==="
echo "다음 줄을 ~/.bashrc 에 추가하면 새 셸에서 자동으로 소싱됩니다:"
echo "  source /opt/ros/humble/setup.bash"
