cmake_minimum_required(VERSION 3.5)
project(franka_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
########################################
# Find ament packages
# ROS relevant
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
# Franka relevant
find_package(Franka 0.7.0 REQUIRED)

########################################
# Build
include_directories(
  include
)

# Robot joint state publisher
add_library(robot_joint_state_publisher_component SHARED
  src/robot_joint_state_publisher_component.cpp
)
target_compile_definitions(robot_joint_state_publisher_component
  PRIVATE "FRANKA_VISUALIZATION_BUILDING_DLL"
)
ament_target_dependencies(robot_joint_state_publisher_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "Franka"
)
rclcpp_components_register_nodes(robot_joint_state_publisher_component 
  "franka_visualization::RobotJointStatePublisher"
)
# install(TARGETS
#   robot_joint_state_publisher_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# "manual composition" approach to robot_joint_state_publisher
add_executable(robot_joint_state_publisher
  src/robot_joint_state_publisher_node.cpp
)
target_link_libraries(
  robot_joint_state_publisher
  robot_joint_state_publisher_component
)
ament_target_dependencies(
  robot_joint_state_publisher
  "rclcpp"
  "libfranka"
)
install(TARGETS
  robot_joint_state_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  robot_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()