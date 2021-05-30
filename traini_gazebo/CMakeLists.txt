cmake_minimum_required(VERSION 3.5)
project(traini_gazebo)

find_package(ament_cmake REQUIRED)
find_package(aruco_tag_model_generation REQUIRED)

generate_aruco_tag_models(
  IDS_START 0
  IDS_STOP 5
  RESOLUTION 100
  SIZE 0.1
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/models
)

install(
  PROGRAMS scripts/kill_gazebo_server.sh
  DESTINATION bin
)

install(
  DIRECTORY
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
