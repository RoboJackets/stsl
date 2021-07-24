# Software Training Support Library (STSL)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

The STSL is a library which facilitates the RoboJackets software training program. It has taken many forms over the years as the hardware platforms have changed.

Currently, the STSL is a collection ROS 2 packages that support an interface to both physical and simluated custom robots.

## Installation

### Installing desktop packages for student computers

1. Get the public key

   ```bash
   wget -qO https://stslaptstorage.z13.web.core.windows.net/pubkey.txt | sudo apt-key add -
   ```

1. Add the apt source repository

   ```bash
   sudo apt-add-repository "deb https://stslaptstorage.z13.web.core.windows.net/ focal main"
   ```

1. Install the desktop packages

   ```bash
   sudo apt install ros-foxy-stsl-desktop
   ```

### Installing robot packages for on-board interface

*Coming soon*
