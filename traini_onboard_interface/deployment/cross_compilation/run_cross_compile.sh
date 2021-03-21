#! /bin/bash

set -e

if ! [ -x "$(command -v ros_cross_compile)" ]
then
sudo apt-get install -y qemu-user-static
pip3 install ros_cross_compile==0.5.0
fi

CROSS_COMPILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$CROSS_COMPILE_DIR"

mkdir -p cross_compile_ws/src

cp defaults.yaml cross_compile_ws/

vcs import cross_compile_ws/src < stsl.repos

GIT_COMMIT_ID=$(git rev-parse HEAD)

pushd cross_compile_ws/src/stsl

git checkout $GIT_COMMIT_ID

popd

pushd cross_compile_ws

ros_cross_compile \
"$(pwd)" \
--arch armhf \
--rosdistro eloquent \
--os debian \
--custom-setup-script ../custom_setup_script.sh \
--colcon-defaults defaults.yaml \
--skip-rosdep-keys "libopensplice69 rti-connext-dds-5.3.1"

popd
