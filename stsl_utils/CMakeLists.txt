cmake_minimum_required(VERSION 3.5)
project(stsl_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CURSES_NEED_NCURSES true)
find_package(Curses REQUIRED)

add_library(stsl_utils
  src/keyboard_teleop.cpp
  src/floor_mat_visualizer.cpp
)
ament_target_dependencies(
  stsl_utils
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
)
target_include_directories(${PROJECT_NAME} PRIVATE ${CURSES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CURSES_LIBRARIES})
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "stsl_utils::KeyboardTeleop"
  EXECUTABLE keyboard_teleop
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "stsl_utils::FloorMatVisualizer"
  EXECUTABLE floor_mat_visualizer
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  visualization_meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
