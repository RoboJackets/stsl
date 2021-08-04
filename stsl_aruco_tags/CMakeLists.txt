cmake_minimum_required(VERSION 3.5)
project(stsl_aruco_tags)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(stsl_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/detector_component.cpp
  src/aruco_marker_visualizer.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "Eigen3"
  "stsl_interfaces"
  "visualization_msgs"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "stsl_aruco_tags::DetectorComponent"
  EXECUTABLE aruco_tag_detector
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "stsl_aruco_tags::ArucoMarkerVisualizer"
  EXECUTABLE aruco_marker_visualizer
)

add_executable(tag_model_generator src/tag_model_generator.cpp)
set_property(TARGET tag_model_generator PROPERTY CXX_STANDARD 17)
ament_target_dependencies(
  tag_model_generator
  "OpenCV"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tag_model_generator
  EXPORT tag_model_generator_EXPORT
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(tag_model_generator_EXPORT)

ament_package(
  CONFIG_EXTRAS cmake/stsl_aruco_tags-extras.cmake
)
