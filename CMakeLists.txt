cmake_minimum_required(VERSION 3.1)
project(SoftwareTrainingSupportLibrary)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(LIB_SOURCE_FILES src/RJRobot.cpp)

if(WIN32)
    add_library(STSL SHARED ${LIB_SOURCE_FILES})
else()
    add_library(STSL STATIC ${LIB_SOURCE_FILES})
endif()

if(WIN32)
    target_sources(STSL PRIVATE src/WindowsUtils.cpp src/SerialPort_WIN32.cpp)
    target_link_libraries(STSL wsock32 ws2_32 )
endif()

if(UNIX AND NOT APPLE)
    target_sources(STSL PRIVATE src/LinuxUtils.cpp src/SerialPort_LINUX.cpp)
    target_link_libraries(STSL udev)
endif()

if(APPLE)
    target_sources(STSL PRIVATE src/OSXUtils.cpp src/SerialPort_OSX.cpp)
    target_link_libraries(STSL "-framework IOKit")
endif()

add_executable(Example example/main.cpp)
target_link_libraries(Example STSL)
