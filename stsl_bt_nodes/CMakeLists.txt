cmake_minimum_required(VERSION 3.5)
project(stsl_bt_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(stsl_interfaces REQUIRED)

set(dependencies
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  stsl_interfaces
)

add_library(${PROJECT_NAME} SHARED
  src/for_each_mineral_sample_node.cpp
  src/get_sample_pose_node.cpp
  src/log_node.cpp
  src/node_registration.cpp
  src/reset_mineral_deposit_tracker_node.cpp
  src/set_light_state_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(${dependencies})

ament_package()
