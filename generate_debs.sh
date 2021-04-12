#! /usr/bin/env bash

set -e

packages_to_skip="traini_onboard_interface"

function build_package(){
    local package_path=$1
    pushd $package_path > /dev/null
    bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
    fakeroot debian/rules binary
    popd > /dev/null
}

# Install needed tools
sudo apt install python3-bloom fakeroot dpkg-dev debhelper

# Iterate through packages in topological order
package_paths=$(colcon list -t -p --packages-skip $packages_to_skip)

for package_path in $package_paths; do
    echo "Building $package_path ..."
    build_package $package_path
done
