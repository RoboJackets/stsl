#! /usr/bin/env bash

set -e

packages_to_skip="traini_onboard_interface"

function install_rosdep_keys_file(){
    rosdep_keys_path="$(pwd)/rosdep.yaml"
    sudo echo "yaml file://$rosdep_keys_path" > /etc/ros/rosdep/sources.list.d/50-stsl-packages.list
}

function build_package(){
    local package_path=$1
    pushd $package_path > /dev/null
    bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro foxy
    fakeroot debian/rules binary
    popd > /dev/null
}

function install_package(){
    local package_path=$1
    local package_name=$(colcon list -n --paths $package_path)
    local deb_style_package_name=${package_name//_/-}
    local deb_path=$(ls ../ros-foxy-$deb_style_package_name*)
    echo "Installing $deb_path"
    sudo dpkg -i $deb_path
}

# Install needed tools
sudo apt install python3-bloom fakeroot dpkg-dev debhelper

install_rosdep_keys_file

# Iterate through packages in topological order
package_paths=$(colcon list -t -p --paths .. --packages-skip $packages_to_skip)

for package_path in $package_paths; do
    echo "\n\nBuilding $package_path ...\n"
    build_package $package_path
    install_package $package_path
done
