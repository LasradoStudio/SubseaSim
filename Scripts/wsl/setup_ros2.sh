#!/usr/bin/env bash
# Install ROS 2 Jazzy desktop (includes RViz2) in a WSL Ubuntu 24.04 distro.
# Run as root:  wsl -d Ubuntu-24.04 -u root -- bash Scripts/wsl/setup_ros2.sh   (from the repo root)
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y locales curl software-properties-common
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# ROS apt source, installed the way the Jazzy docs describe (ros2-apt-source package).
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | sed -E 's/.*"([^"]+)".*/\1/')
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb"
    apt-get install -y /tmp/ros2-apt-source.deb
fi

apt-get update
apt-get upgrade -y
apt-get install -y ros-jazzy-desktop python3-numpy

grep -q '/opt/ros/jazzy/setup.bash' /root/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
echo "ROS 2 Jazzy installed: $(ls /opt/ros)"
