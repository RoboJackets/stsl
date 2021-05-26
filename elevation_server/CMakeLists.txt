cmake_minimum_required(VERSION 3.5)
project(elevation_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/elevation_server.cpp)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "rclcpp_components"
  "stsl_interfaces"
  "tf2_ros"
  "OpenCV"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "elevation_server::ElevationServer"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
