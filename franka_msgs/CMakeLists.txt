cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# declare the action files to generate code for
set(action_files
  "action/ErrorRecovery.action"
)
# declare the message files to generate code for
set(msg_files
  "msg/Errors.msg"
  "msg/FrankaState.msg"
)

# declare the service files to generate code for
set(srv_files
  "srv/SetCartesianImpedance.srv"
  "srv/SetEEFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetKFrame.srv"
  "srv/SetLoad.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()