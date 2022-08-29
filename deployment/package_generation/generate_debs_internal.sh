#! /usr/bin/env bash

set -e

packages_to_skip="traini_onboard_interface"

generated_debs=()

function install_rosdep_keys_file(){
    rosdep_keys_path="$(pwd)/rosdep.yaml"
    sudo echo "yaml file://$rosdep_keys_path" > /etc/ros/rosdep/sources.list.d/50-stsl-packages.list
    rosdep update
}

function build_package(){
    local package_path=$1
    pushd $package_path > /dev/null
    bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble
    fakeroot debian/rules binary
    popd > /dev/null
}

function install_package(){
    local package_path=$1
    local package_name=$(colcon list -n --paths $package_path)
    local deb_style_package_name=${package_name//_/-}
    readarray -t deb_paths < <(ls $package_path/../ros-hunble-$deb_style_package_name*)
    echo -e "\n\nInstalling these files:"
    printf '%s\n' "${deb_paths[@]}"
    echo -e "\n"
    sudo dpkg -i ${deb_paths[@]}
    generated_debs+=(${deb_paths[@]})
    unset deb_paths
}

install_rosdep_keys_file

source /opt/ros/humble/setup.bash

package_paths=$(colcon list -t -p --base-paths ../../* --packages-skip $packages_to_skip)

for package_path in $package_paths; do
    echo -e "\n\nBuilding $package_path ...\n"
    build_package $package_path
    install_package $package_path
done

mkdir ../../package_files
for deb_file in ${generated_debs[@]}; do
    mv $deb_file ../../package_files/
done
pushd ../../package_files > /dev/null
tar -czvf ../debians.tar.gz *
popd > /dev/null
