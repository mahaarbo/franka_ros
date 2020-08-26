cmake_minimum_required(VERSION 3.5)
project(franka_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
########################################
# Find packages
# ROS relevant
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
# Franka relevant
find_package(Franka REQUIRED)

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
)
target_link_libraries(
  robot_joint_state_publisher_component
  Franka::Franka
)
rclcpp_components_register_nodes(robot_joint_state_publisher_component 
  "franka_visualization::RobotJointStatePublisher"
)
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
  "Franka"
)
# Gripper joint state publisher
add_library(gripper_joint_state_publisher_component SHARED
  src/gripper_joint_state_publisher_component.cpp
)
target_compile_definitions(gripper_joint_state_publisher_component
  PRIVATE "FRANKA_VISUALIZATION_BUILDING_DLL"
)
ament_target_dependencies(gripper_joint_state_publisher_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)
target_link_libraries(
  gripper_joint_state_publisher_component
  Franka::Franka 
)
rclcpp_components_register_nodes(gripper_joint_state_publisher_component
  "franka_visualization::GripperJointStatePublisher"
)
add_executable(gripper_joint_state_publisher
  src/gripper_joint_state_publisher_node.cpp
)
target_link_libraries(
  gripper_joint_state_publisher
  gripper_joint_state_publisher_component
)
ament_target_dependencies(
  gripper_joint_state_publisher
  "rclcpp"
  "Franka"
)
########################################
# Install
# Launch files
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
# Config files
install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
# The rest
install(TARGETS
  robot_joint_state_publisher_component
  gripper_joint_state_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  robot_joint_state_publisher
  gripper_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()