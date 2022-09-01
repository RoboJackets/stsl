#! /usr/bin/env bash

set -e

# amd64

docker build --no-cache -t stsl_build -f Dockerfile_amd64 ../..
docker create -ti --name stsl_build_temp stsl_build bash
docker cp stsl_build_temp:/usr/src/workspace/src/stsl/debians.tar.gz ./debians_amd64.tar.gz
docker rm -f stsl_build_temp

# arm64

# temporarily disabled until Gazebo & ROS Gazebo packages are released for arm64
# See https://github.com/osrf/gazebo/issues/3236

# docker build --no-cache -t stsl_build -f Dockerfile_arm64 ../..
# docker create -ti --name stsl_build_temp stsl_build bash
# docker cp stsl_build_temp:/usr/src/workspace/src/stsl/debians.tar.gz ./debians_arm64.tar.gz
# docker rm -f stsl_build_temp

COLOR_GREEN='\033[0;32m'
COLOR_AUTO='\033[0m'

echo -e "\n${COLOR_GREEN}Package generation complete!${COLOR_AUTO}"
