
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
  Command,
  FindExecutable,
  LaunchConfiguration, 
  PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
  launch_args = []
  launch_args.append(DeclareLaunchArgument(
    "robot_ip", 
    default_value="172.16.0.2",
    description="<fci-ip> in the libfranka documentation."
  ))
  robot_ip = LaunchConfiguration("robot_ip")
  # Determining whether to load gripper
  launch_args.append(DeclareLaunchArgument(
    "load_gripper",
    default_value="False",
    description="Load the Panda with the gripper [True|False]."
  ))
  gripper_condition = IfCondition(
    LaunchConfiguration("load_gripper")
  )
  no_gripper_condition = UnlessCondition(
    LaunchConfiguration("load_gripper")
  )

  # Robot description
  xacro_path_with_gripper = PathJoinSubstitution([
    FindPackageShare("franka_description"),
    "robots", "panda_arm_hand.urdf.xacro"
  ])
  xacro_path_without_gripper = PathJoinSubstitution([
    FindPackageShare("franka_description"),
    "robots", "panda_arm.urdf.xacro"
  ])
  urdf_with_gripper = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    xacro_path_with_gripper,
    " ",
    "robot_ip:=",
    robot_ip
  ])
  urdf_without_gripper = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    xacro_path_without_gripper,
    " ",
    "robot_ip:=",
    robot_ip
  ])

  # Controller configurations
  robot_controllers = PathJoinSubstitution([
    FindPackageShare("franka_hw"),
    "config", "test_controllers.yaml"
  ])
  # RViz configuration
  rviz_config = PathJoinSubstitution([
    FindPackageShare("franka_visualization"),
    "launch", "franka_visualization.rviz"
  ])
  # Nodes
  nodes = []
  # Robot state publisher
  nodes.append(Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[{"robot_description": urdf_with_gripper}],
    condition=gripper_condition
  ))
  nodes.append(Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[{"robot_description": urdf_without_gripper}],
    condition=no_gripper_condition
  ))
  # Gripper action server 
  # TODO + combining joint_states from the two.
  nodes.append(Node(
    package="franka_gripper",
    executable="franka_gripper_node",
    output="both",
    parameters=[{"robot_ip":robot_ip}],
    condition=gripper_condition
  ))
  # Controller manager
  nodes.append(Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="both",
    parameters=[
      {"robot_description": urdf_without_gripper},
      robot_controllers],
    condition=no_gripper_condition
  ))
  nodes.append(Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="both",
    parameters=[
      {"robot_description": urdf_with_gripper},
      robot_controllers],
    condition=gripper_condition
  ))
  # RViz
  nodes.append(Node(
    package="rviz2",
    executable="rviz2",
    output="both",
    arguments=["-d", rviz_config]
  ))

  return LaunchDescription(launch_args + nodes)