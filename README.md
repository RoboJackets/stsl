# Software Training Support Library (STSL)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

The STSL is a library which facilitates the RoboJackets software training program. It has taken many forms over the years as the hardware platforms have changed.

Currently, the STSL is a library providing an API for the 2019 training robots based on the BeagleBone Blue.

## Dependencies

* [CMake](https://cmake.org/)
* [OpenCV](https://opencv.org)
* [librobotcontrol](https://github.com/StrawsonDesign/librobotcontrol)

## Installation

This library is intended to be used as part of the RoboJackets Software Training Program. The instrucor team will have already installed this library on the training robots.

For the instructor team, this library installs in the traditional way with CMake. 

```
mkdir build
cd build
cmake ..
cmake --build .
sudo make install
```

