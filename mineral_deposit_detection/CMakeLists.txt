cmake_minimum_required(VERSION 3.5)
project(mineral_deposit_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/mineral_deposit_detector.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  stsl_interfaces
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "mineral_deposit_detection::MineralDepositDetector"
  EXECUTABLE mineral_deposit_detector
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
