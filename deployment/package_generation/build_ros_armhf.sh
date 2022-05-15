#! /usr/bin/env bash

set -e

apt-get update
apt-get upgrade -y

apt-get install -y locales curl gnupg2
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo 'deb [arch=armhf signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main' | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt-get update

apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

apt-get install --no-install-recommends -y libcunit1-dev

apt-get install -y ca-certificates

mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos 

rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

colcon build --symlink-install

COLOR_GREEN='\033[0;32m'
COLOR_AUTO='\033[0m'

echo -e "\n${COLOR_GREEN}ROS !${COLOR_AUTO}"
