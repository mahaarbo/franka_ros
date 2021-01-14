cmake_minimum_required(VERSION 3.5)
project(franka_gripper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
########################################
# Find packages
# ROS relevant
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Franka relevant
find_package(Franka REQUIRED)

########################################
# Build
# Actions and messages
set(action_files
    "action/Grasp.action"
    "action/Homing.action"
    "action/Move.action"
    "action/Stop.action"
)

set(message_files
    "msg/GraspEpsilon.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    ${message_files}
    DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_generators)

# Action server
include_directories(
    include
)
add_library(franka_gripper_server SHARED
    src/franka_gripper_server.cpp
)
target_compile_definitions(franka_gripper_server
    PRIVATE "FRANKA_GRIPPER_SERVER_BUILDING_DLL"
)
ament_target_dependencies(franka_gripper_server
    "rclcpp"
    "rclcpp_components"
    "rclcpp_action"
)
rosidl_target_interfaces(franka_gripper_server
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(
    franka_gripper_server
    Franka::Franka
)
rclcpp_components_register_nodes(franka_gripper_server
    "franka_gripper::FrankaGripperServer"
)
########################################
# Install
install(TARGETS
    franka_gripper_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_package()