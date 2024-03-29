cmake_minimum_required(VERSION 3.5)
project(stsl_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_library(${PROJECT_NAME} SHARED
  src/action_button.cpp
  src/action_button.hpp
  src/execute_mission_button.cpp
  src/park_at_peak_button.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "stsl_interfaces"
  "rviz_common"
  "pluginlib"
  "Qt5"
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
