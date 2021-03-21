cmake_minimum_required(VERSION 3.5)
project(ign_kinematics_plugin)

find_package(ament_cmake REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo3 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-msgs5 REQUIRED)

add_library(IgnKinematicsSystem SHARED src/IgnKinematicsSystem.cpp)
target_link_libraries(IgnKinematicsSystem
    ignition-plugin1::ignition-plugin1
    ignition-gazebo3::ignition-gazebo3
    ignition-transport8::ignition-transport8
    ignition-msgs5::ignition-msgs5
)
target_include_directories(IgnKinematicsSystem PRIVATE
    include/ign_kinematics_plugin
)

ament_export_targets(export_IgnKinematicsSystem HAS_LIBRARY_TARGET)

install(
  TARGETS IgnKinematicsSystem
  EXPORT export_IgnKinematicsSystem
  DESTINATION lib
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
