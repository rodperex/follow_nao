cmake_minimum_required(VERSION 3.8)
project(follow_nao)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  vision_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nao_lola_sensor_msgs
  behaviortree_cpp
)

include_directories(include)

# BT Nodes library
add_library(follow_nao_bt_nodes SHARED
  src/bt_nodes/is_target_detected_condition.cpp
  src/bt_nodes/spin_search_action.cpp
  src/bt_nodes/follow_action.cpp
)

ament_target_dependencies(follow_nao_bt_nodes ${dependencies})

# Main BT executor node
add_executable(bt_follow_node src/bt_follow_node.cpp)
target_link_libraries(bt_follow_node follow_nao_bt_nodes)
ament_target_dependencies(bt_follow_node ${dependencies})

# Install
install(TARGETS
  follow_nao_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bt_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(follow_nao_bt_nodes)
ament_export_dependencies(${dependencies})
ament_package()
