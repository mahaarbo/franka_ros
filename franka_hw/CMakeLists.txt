cmake_minimum_required(VERSION 3.5)
project(franka_hw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
########################################
# Find packages
# ROS relevant
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
#find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
#find_package(joint_limits_interface REQUIRED)
#find_package(controller_interface REQUIRED)
# Franka relevant
find_package(Franka REQUIRED)

########################################
# Build
add_library(franka_hw
  SHARED
  src/franka_hw2.cpp
  src/control_mode.cpp
  src/resource_helpers2.cpp
)

target_include_directories(
  franka_hw
  PRIVATE
  include
)

ament_target_dependencies(
  franka_hw
  hardware_interface
  pluginlib
  rclcpp
  Franka
)
target_link_libraries(
  franka_hw
  Franka::Franka
)
pluginlib_export_plugin_description_file(hardware_interface franka_hw_plugin.xml)

########################################
# INSTALL
install(
  TARGETS franka_hw
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  franka_hw
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()

########################################
### OLD
#find_package(catkin REQUIRED COMPONENTS
#  actionlib
  #controller_interface
  #combined_robot_hw
  #hardware_interface
  #joint_limits_interface
  #roscpp
  #pluginlib
#  urdf
  #franka_msgs
  #message_generation
#)

#find_package(Franka 0.7.0 REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES franka_hw franka_control_services
#  CATKIN_DEPENDS
#    actionlib_msgs
#    actionlib
#    controller_interface
#    combined_robot_hw
#    hardware_interface
#    joint_limits_interface
#    roscpp
#    pluginlib
#    urdf
#    franka_msgs
#  DEPENDS Franka
#)

#add_library(franka_hw
#  src/control_mode.cpp
#  src/franka_hw.cpp
#  src/franka_combinable_hw.cpp
#  src/franka_combined_hw.cpp
#  src/resource_helpers.cpp
#  src/trigger_rate.cpp
#)

#add_dependencies(franka_hw
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(franka_hw
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
#   franka_control_services
# )

# target_include_directories(franka_hw SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_hw PUBLIC
#   include
# )

# ## Installation
# install(TARGETS franka_hw
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ## franka_control_services
# add_library(franka_control_services
#   src/services.cpp
# )

# add_dependencies(franka_control_services
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_control_services
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_control_services SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )
# target_include_directories(franka_control_services PUBLIC
#   include
# )

# install(TARGETS franka_control_services
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()

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
#   add_format_target(franka_hw FILES ${SOURCES} ${HEADERS})
#   add_tidy_target(franka_hw
#     FILES ${SOURCES}
#     DEPENDS franka_hw
#   )
# endif()
