cmake_minimum_required(VERSION 3.5)
project(aruco_tag_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vision_opencv REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/detector_component.cpp)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "rclcpp_components"
  "vision_opencv"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "aruco_tag_detection::DetectorComponent"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
