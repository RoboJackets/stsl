#! /bin/bash

set -e

CROSS_COMPILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$CROSS_COMPILE_DIR"

mkdir -p cross_compile_ws/src

cp defaults.yaml cross_compile_ws/

vcs import cross_compile_ws/src < stsl.repos

cd cross_compile_ws

ros_cross_compile \
--arch armhf \
--rosdistro eloquent \
--os debian \
--custom-setup-script ../custom_setup_script.sh \
--colcon-defaults defaults.yaml \
$(pwd)

