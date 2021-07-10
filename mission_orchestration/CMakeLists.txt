cmake_minimum_required(VERSION 3.5)
project(mission_orchestration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(stsl_bt_nodes REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/mission_orchestrator.cpp
  src/yaml_helpers.cpp
)
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_behavior_tree
  tf2_ros
  stsl_interfaces
  stsl_bt_nodes
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}_components ${YAML_CPP_LIBRARIES})
rclcpp_components_register_node(
  ${PROJECT_NAME}_components
  PLUGIN "mission_orchestration::MissionOrchestrator"
  EXECUTABLE mission_orchestrator_node
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  behavior_trees
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
