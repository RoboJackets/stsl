cmake_minimum_required(VERSION 3.5)
project(stsl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(LIBGPIOD_LIBRARY
  NAMES libgpiodcxx.so libgpiodcxx.so.1 libgpiodcxx.so.1.0.0
)
if(NOT LIBGPIOD_LIBRARY)
  message(FATAL_ERROR "libgpiodcxx not found!")
endif()

add_executable(robot_interface_node src/robot_interface_node.cpp src/motor.cpp src/line_sensor.cpp src/encoder.cpp src/ultrasonic_sensor.cpp)
ament_target_dependencies(robot_interface_node rclcpp std_msgs)
target_link_libraries(robot_interface_node ${LIBGPIOD_LIBRARY} pthread)

install(TARGETS
    robot_interface_node
    DESTINATION lib/${PROJECT_NAME})

install(FILES
    scripts/run_node.sh
    DESTINATION lib/${PROJECT_NAME})

ament_package()

