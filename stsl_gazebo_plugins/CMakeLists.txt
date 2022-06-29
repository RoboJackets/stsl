cmake_minimum_required(VERSION 3.5)
project(stsl_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(RosLightControlPlugin SHARED src/RosLightControlPlugin.cpp)
ament_target_dependencies(RosLightControlPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
)

add_library(RosNoisyPlanarOdomPlugin SHARED src/RosNoisyPlanarOdomPlugin.cpp)
ament_target_dependencies(RosNoisyPlanarOdomPlugin
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
)

add_library(HideLightVisualsPlugin SHARED src/HideLightVisualsPlugin.cpp)
ament_target_dependencies(HideLightVisualsPlugin
  "gazebo_dev"
)

install(
  TARGETS RosLightControlPlugin RosNoisyPlanarOdomPlugin HideLightVisualsPlugin
  DESTINATION lib
)

install(DIRECTORY
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
