cmake_minimum_required(VERSION 3.7)
project(SoftwareTrainingSupportLibrary)

set(CMAKE_CXX_STANDARD 14)

#find_package(Boost COMPONENTS system REQUIRED)

#include_directories(SYSTEM  ${Boost_INCLUDE_DIR})
#link_directories(${Boost_LIBRARY_DIR})

include_directories(include)

set(LIB_SOURCE_FILES src/RJRobot.cpp)

add_library(STSL ${LIB_SOURCE_FILES})

if(WIN32)
    target_sources(STSL PRIVATE src/WindowsUtils.cpp src/SerialPort_WIN32.cpp)
    target_link_libraries(STSL wsock32 ws2_32 )
endif()

if(UNIX AND NOT APPLE)
    target_sources(STSL PRIVATE src/LinuxUtils.cpp src/SerialPort_LINUX.cpp)
    target_link_libraries(STSL udev)
endif()

#target_link_libraries(STSL ${Boost_LIBRARIES})

add_executable(Example example/main.cpp)
target_link_libraries(Example STSL)
