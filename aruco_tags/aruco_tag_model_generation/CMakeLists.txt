cmake_minimum_required(VERSION 3.5)
project(aruco_tag_model_generation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_executable(tag_model_generator src/tag_model_generator.cpp)
set_property(TARGET tag_model_generator PROPERTY CXX_STANDARD 17)
ament_target_dependencies(
  tag_model_generator
  "OpenCV"
)

install(TARGETS tag_model_generator
  EXPORT tag_model_generator_EXPORT
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(tag_model_generator_EXPORT)

ament_package(
  CONFIG_EXTRAS cmake/aruco_tag_model_generation-extras.cmake
)
