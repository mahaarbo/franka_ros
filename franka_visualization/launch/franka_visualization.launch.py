# Author: Mathias Hauan Arbo
# Department of Engineering Cybernetics, NTNU, 2020
# Based on the work of Franka Emika
import os
import launch
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with franka_visualization component"""
    # Arguments
    load_gripper_arg = launch.actions.DeclareLaunchArgument(
        name="load_gripper",
        default_value="True",
        description="Load with the Franka Emika Panda gripper?"
    )
    gripper_condition = launch.conditions.IfCondition(
        LaunchConfiguration("load_gripper")
    )
    no_gripper_condition = launch.conditions.UnlessCondition(
        LaunchConfiguration("load_gripper")
    )
    robot_ip_arg = launch.actions.DeclareLaunchArgument(
        name="robot_ip",
        default_value="172.16.0.2",
        description="The <fci-ip> in the Franka documentation."
    )
    publish_rate_arg = launch.actions.DeclareLaunchArgument(
        name="publish_rate",
        default_value="30.0",
        description="Publish rate of the robot state publisher."
    )

    # Location of the relevant URDFs
    urdf_with_gripper_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots", "panda_arm_hand.urdf"
    )
    urdf_without_gripper_path = os.path.join(
        get_package_share_directory("franka_description"),
       "robots", "panda_arm.urdf"
    )
    # RVIZ config
    rviz_config = os.path.join(
        get_package_share_directory("franka_visualization"),
        "launch", "franka_visualization.rviz"
    )
    # Visualization Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config]
    )
    # Robot description and TF publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[urdf_without_gripper_path],
        condition=no_gripper_condition
    )
    # Robot description and TF publisher with gripper
    robot_state_publisher_gripper_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[urdf_with_gripper_path],
        condition=gripper_condition
    )
    # Robot joint state node
    robot_joint_state_publisher_config = os.path.join(
        get_package_share_directory("franka_visualization"),
        "config",
        "robot_settings.yaml"
    )
    robot_joint_state_publisher_node = Node(
        package="franka_visualization",
        executable="robot_joint_state_publisher",
        output="screen",
        name="robot_joint_state_publisher",
        namespace="robot_joint_state_publisher",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "publish_rate": LaunchConfiguration("publish_rate")
            },
            robot_joint_state_publisher_config,
        ]
    )
    # Gripper joint state node
    gripper_joint_state_publisher_config = os.path.join(
        get_package_share_directory("franka_visualization"),
        "config",
        "gripper_settings.yaml"
    )
    # gripper_joint_state_publisher_node = Node(
    #     package="franka_visualization",
    #     node_executable="gripper_joint_state_publisher",
    #     output="screen",
    #     node_name="gripper_joint_state_publisher",
    #     node_namespace="gripper_joint_state_publisher",
    #     parameters=[
    #         {
    #             "robot_ip": LaunchConfiguration("robot_ip"),
    #             "publish_rate": LaunchConfiguration("publish_rate")
    #         },
    #         gripper_joint_state_publisher_config
    #     ],
    #     condition=gripper_condition
    # )
    gripper_joint_action_server = Node(
        package="franka_gripper",
        executable="franka_gripper_node",
        output="screen",
        name="franka_gripper_node",
        namespace="franka_gripper_node",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "publish_rate": LaunchConfiguration("publish_rate")
            }
        ]

    )
    # Join the joint states of the gripper and the robot
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        name="joint_state_publisher",
        parameters=[
            {
                "rate": LaunchConfiguration("publish_rate"),
                "source_list": [
                    "robot_joint_state_publisher/joint_states",
                    #"gripper_joint_state_publisher/joint_states"
                    "franka_gripper_node/joint_states"
                ]
            }
        ]
    )
    return launch.LaunchDescription([
        load_gripper_arg,
        robot_ip_arg,
        publish_rate_arg,
        rviz_node,
        robot_state_publisher_node,
        robot_state_publisher_gripper_node,
        robot_joint_state_publisher_node,
        gripper_joint_action_server,
        #gripper_joint_state_publisher_node,
        joint_state_publisher_node
    ])