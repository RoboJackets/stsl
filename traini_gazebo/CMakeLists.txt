cmake_minimum_required(VERSION 3.5)
project(traini_gazebo)

find_package(ament_cmake REQUIRED)
find_package(stsl_aruco_tags REQUIRED)

generate_aruco_tag_models(
  IDS_START 0
  IDS_STOP 40
  RESOLUTION 100
  SIZE 0.1
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/models
)

install(
  PROGRAMS
  scripts/publish_map_boundary.py
  scripts/randomize_robot_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/models/
  DESTINATION share/${PROJECT_NAME}/models
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
