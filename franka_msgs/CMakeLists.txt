cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ErrorRecovery.action"
  "msg/Errors.msg"
  "msg/FrankaState.msg"
  "srv/SetCartesianImpedance.srv"
  "srv/SetEEFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetKFrame.srv"
  "srv/SetLoad.srv"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()