cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################################
# Find packages
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

#find_package(Franka REQUIRED)

########################################
# Build
add_library(franka_example_controllers SHARED
  src/joint_position_example_controller2.cpp
)
target_include_directories(franka_example_controllers PRIVATE include)
ament_target_dependencies(franka_example_controllers
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(franka_example_controllers PRIVATE "POSITION_CONTROLLERS_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(franka_example_controllers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface franka_example_controllers_plugin.xml)

########################################
# INSTALL
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS franka_example_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  control_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  franka_example_controllers
)
ament_package()
# find_package(catkin REQUIRED COMPONENTS
#   controller_interface
#   dynamic_reconfigure
#   eigen_conversions
#   franka_hw
#   geometry_msgs
#   hardware_interface
#   tf
#   tf_conversions
#   message_generation
#   pluginlib
#   realtime_tools
#   roscpp
#   rospy
# )

# find_package(Eigen3 REQUIRED)
# find_package(Franka 0.7.0 REQUIRED)

# add_message_files(FILES
#   JointTorqueComparison.msg
# )

# generate_messages()

# generate_dynamic_reconfigure_options(
#   cfg/compliance_param.cfg
#   cfg/desired_mass_param.cfg
#   cfg/dual_arm_compliance_param.cfg
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES franka_example_controllers
#   CATKIN_DEPENDS
#     controller_interface
#     dynamic_reconfigure
#     eigen_conversions
#     franka_hw
#     geometry_msgs
#     hardware_interface
#     tf
#     tf_conversions
#     message_runtime
#     roscpp
#   DEPENDS Franka
# )

# add_library(franka_example_controllers
#   src/elbow_example_controller.cpp
#   src/cartesian_pose_example_controller.cpp
#   src/cartesian_velocity_example_controller.cpp
#   src/joint_position_example_controller.cpp
#   src/joint_velocity_example_controller.cpp
#   src/model_example_controller.cpp
#   src/joint_impedance_example_controller.cpp
#   src/cartesian_impedance_example_controller.cpp
#   src/force_example_controller.cpp
#   src/dual_arm_cartesian_impedance_example_controller.cpp
# )

# add_dependencies(franka_example_controllers
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_generate_messages_cpp
#   ${PROJECT_NAME}_gencpp
#   ${PROJECT_NAME}_gencfg
# )

# target_link_libraries(franka_example_controllers PUBLIC
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_example_controllers SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_example_controllers PUBLIC
#   include
# )

# ## Installation
# install(TARGETS franka_example_controllers
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# install(FILES franka_example_controllers_plugin.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# catkin_install_python(
#   PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Tools
# include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
#   RESULT_VARIABLE CLANG_TOOLS
# )
# if(CLANG_TOOLS)
#   file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#   file(GLOB_RECURSE HEADERS
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
#   )
#   add_format_target(franka_example_controllers FILES ${SOURCES} ${HEADERS})
#   add_tidy_target(franka_example_controllers
#     FILES ${SOURCES}
#     DEPENDS franka_example_controllers
#   )
# endif()
