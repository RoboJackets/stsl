cmake_minimum_required(VERSION 3.5)
project(stsl_nav_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  rclcpp
  nav2_core
  angles
)

add_library(${PROJECT_NAME} SHARED
  src/point_and_shoot_controller.cpp
  src/pd_controller.cpp
  src/straight_line_planner.cpp
  src/no_progress_checker.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core stsl_nav_plugins_libraries.xml)

ament_package()
