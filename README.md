# Software Training Support Library (STSL)

The STSL is a cross-platform library and accompanying Arduino firmware sketch which provides control over the Arduino-based LEGO robotics kits used in the RoboJackets Software Training program.

## Dependencies

### Linux

* cmake
* Boost.ASIO
* libudev

### Windows

* MinGW
* cmake
* Boost.ASIO
* WinSock32

## Installation

### Linux

1. Install dependencies

    ```
    sudo apt install libboost-system-dev
    ```
    
2. Clone repository

    ```
    git clone 
    ```
    
3. Build library

    ```
    mkdir build && cd build
    cmake ..
    make
    ```

### Windows

1. Install dependencies

   Download Stephan T. Lavavej's MinGW distribution (without git):

    [https://nuwen.net/files/mingw/mingw-15.0-without-git.exe](https://nuwen.net/files/mingw/mingw-15.0-without-git.exe)

2. Install the GitHub desktop app

   **TODO**

3. Install cmake

4. Clone repository using the GitHub gui

   **TODO**

5. Build library

   Using the cmake gui tool or an IDE such as JetBrains' CLion