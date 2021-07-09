cmake_minimum_required(VERSION 3.5)
project(stsl_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tag.msg"
  "msg/TagArray.msg"
  "srv/SampleElevation.srv"
  "action/ExecuteMission.action"
  "action/ParkAtPeak.action"
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
