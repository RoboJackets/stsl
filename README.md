# Software Training Support Library (STSL)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

The STSL is a cross-platform library and accompanying Arduino firmware which provides control over the Arduino-based robotics kits used in the RoboJackets Software Training program.

## Dependencies

* [CMake](https://cmake.org/)
* [Mingw-w64](https://mingw-w64.org) (Windows only)

## Installation

This library is intended to be used as part of the RoboJackets Software Training Program. The code for the hardware exercises used in that program will automatically fetch and install this library.

## Flashing Firmware

Before you can update the firmware on the robots, you'll need to install a few things:

1. Install the [Arduino IDE](https://www.arduino.cc/)
2. Follow the instructions for installing the Espresiff Systems [esp32 boards](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
3. Install the _Adafruit APDS9960 Library_ via the Arduino IDE library manager

With those dependencies installed, you can now flash the firmware.

1. Open STSL_Firmware/STSL_Firmware.ino with the Arduino IDE
2. Select the "Adafruit ESP32 Feather board
   * Tools->Board->Adafruit ESP32 Feather
   * Other board options can be left at default values
3. Plug the robot in to your computer via USB
4. Set the port as needed for the IDE to find the board
5. Press "Upload"
