cmake_minimum_required(VERSION 3.5)
project(traini_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/traini.urdf.xacro
  TARGET urdf_generation
  INSTALL DESTINATION urdf
)

install(
  DIRECTORY
  launch
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
