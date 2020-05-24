#! /bin/bash

set -e

PACKAGING_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$PACKAGING_DIR"

# TODO check that ../cross_compilation/sysroot/ros_ws/install_armhf exists and is a directory

mkdir -p package/ros2

cp -r ../cross_compilation/sysroot/ros_ws/install_armhf package/ros2/

mv package/ros2/install_armhf package/ros2/eloquent

tar -czf stsl_package.tar.gz package/

echo -e "\n\n\e[32m\e[1mPackaging Complete!\e[0m"
echo -e "To install on a robot:"
echo -e "Copy stsl_package.tar.gz and install_stsl.sh to the home directory of"
echo -e "the beaglebone, and run install_stsl.sh on the beaglebone.\n"
