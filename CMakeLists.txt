cmake_minimum_required(VERSION 3.5)
project(stsl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libgpiodcxx REQUIRED IMPORTED_TARGET libgpiodcxx)

add_executable(robot_interface_node src/robot_interface_node.cpp src/motor.cpp src/line_sensor.cpp src/encoder.cpp src/ultrasonic_sensor.cpp)
ament_target_dependencies(robot_interface_node rclcpp std_msgs)
target_link_libraries(robot_interface_node PkgConfig::libgpiodcxx pthread)
target_compile_options(robot_interface_node PRIVATE cxx_std_17)
target_compile_options(robot_interface_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
    robot_interface_node
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    scripts/run_node.sh
    DESTINATION lib/${PROJECT_NAME})

ament_package()
