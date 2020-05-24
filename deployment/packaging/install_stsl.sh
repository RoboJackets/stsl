#! /bin/bash

set -e

tar -xzf stsl_package.tar.gz

# TODO setup STSL on the BBB
# 1. setup robot number (prompt user and then add an export for ROS_DOMAIN_ID to bashrc?)
# 2. install deps (libgpiod2, libtinyxml2, etc.)
# 3. setup whatever it takes to make the robot_interface_node auto run on boot
