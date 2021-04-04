cmake_minimum_required(VERSION 3.5)
project(traini_gazebo)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/kill_gazebo_server.sh
  DESTINATION bin
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
