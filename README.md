# Software Training Support Library (STSL)

The STSL is a cross-platform library and accompanying Arduino firmware sketch which provides control over the Arduino-based LEGO robotics kits used in the RoboJackets Software Training program.

## Dependencies

### Linux

* cmake
* libudev

### Windows

* MinGW
* cmake
* WinSock32

### Max OS X

* cmake
* 

## Installation

### Linux

1. Clone repository

    ```
    git clone https://github.com/RoboJackets/stsl
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

    [https://nuwen.net/mingw.html](https://nuwen.net/mingw.html)

2. Install the GitHub desktop app

   Install from [https://desktop.github.com/](https://desktop.github.com/)

3. Install cmake

   Install from [https://cmake.org/download/](https://cmake.org/download/)

4. Clone repository using the GitHub gui

   **TODO**

5. Build library

   Using the cmake gui tool or an IDE such as JetBrains' CLion
   
   
### Mac OS X

1. Clone repository

   ```
   git clone **TODO**
   ```
   
2. Build library

   ```
   mkdir build && cd build
   cmake ..
   make
   ```
   
   __Note:__ If you encounter errors where the compiler is unable to find standard C++ headers, it may be caused by these headers being moved by Xcode. You may be able to resolve this issue by re-installing the Xcode command line tools using the following command.

   ```
   xcode-select --install
   ```
   On my machine (running OS X Mavericks, 10.9.5), I also had to change the compiler to use Apple's clang compiler instead of the GNU compiler in Xcode. This can be done by replacing the cmake command above with the following.
   ```
   cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
   ```
