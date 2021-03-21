cmake_minimum_required(VERSION 3.11)
project(stsl_ign_plugins)

cmake_policy(SET CMP0072 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

find_package(ament_cmake REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo3 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-msgs5 REQUIRED)

add_library(Kinematics2dSystem SHARED src/Kinematics2dSystem.cpp)
target_link_libraries(Kinematics2dSystem
    ignition-plugin1::ignition-plugin1
    ignition-gazebo3::ignition-gazebo3
    ignition-transport8::ignition-transport8
    ignition-msgs5::ignition-msgs5
)
set_target_properties(Kinematics2dSystem PROPERTIES 
  OUTPUT_NAME ignition-2d-kinematics-system
  CXX_STANDARD 17
)

ament_export_targets(export_Kinematics2dSystem HAS_LIBRARY_TARGET)

install(
  TARGETS Kinematics2dSystem
  EXPORT export_Kinematics2dSystem
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
