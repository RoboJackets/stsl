#! /bin/bash

set -e

CROSS_COMPILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$CROSS_COMPILE_DIR"

mkdir -p sysroot/ros_ws/src

cp defaults.yaml sysroot/ros_ws/

vcs import sysroot/ros_ws/src < stsl.repos

ros_cross_compile -a armhf -d eloquent -o debian -r fastrtps --sysroot-path . --custom-setup-script ./custom_setup_script.sh
