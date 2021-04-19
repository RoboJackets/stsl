cmake_minimum_required(VERSION 3.5)
project(aruco_tag_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stsl_interfaces REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/detector_component.cpp)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "Eigen3"
  "stsl_interfaces"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "aruco_tag_detection::DetectorComponent"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
