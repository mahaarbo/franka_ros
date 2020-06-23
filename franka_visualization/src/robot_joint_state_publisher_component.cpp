// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
#include "franka_visualization/robot_joint_state_publisher_component.hpp"

namespace franka_visualization
{
using namespace std::chrono_literals;

RobotJointStatePublisher::RobotJointStatePublisher(const rclcpp::NodeOptions & options)
: Node("robot_joint_state_publisher")
{
    this->declare_parameter("robot_ip");
    this->declare_parameter("joint_names");
    this->declare_parameter("publish_rate");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    robot_ip = parameters_client->get_parameter<std::string>("robot_ip", "172.16.0.2");
    joint_names = parameters_client->get_parameter<std::vector<std::string>>("joint_names",
        {
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7"
        }
    );
    publish_rate = parameters_client->get_parameter<double>("publish_rate", 30.);
    // Initialize joint_state message
    current_joint_state.name = joint_names;
    current_joint_state.position = {0., 0., 0., 0., 0., 0., 0.};
    current_joint_state.velocity = {0., 0., 0., 0., 0., 0., 0.};
    current_joint_state.effort = {0., 0., 0., 0., 0., 0., 0.};
    RCLCPP_INFO(this->get_logger(), "Establishing connection to Franka Robot.");
    robot = std::make_shared<franka::Robot>(robot_ip);
    publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    timer = create_wall_timer(
        std::chrono::duration<double>(1./publish_rate),
        std::bind(&RobotJointStatePublisher::on_timer, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Everything setup.");
}

void RobotJointStatePublisher::on_timer()
{
    current_joint_state.header.stamp = rclcpp::Clock().now();
    current_robot_state = robot->readOnce();
    for (size_t i = 0; i < joint_names.size(); i++)
    {
        current_joint_state.position[i] = current_robot_state.q[i];
        current_joint_state.velocity[i] = current_robot_state.dq[i];
        current_joint_state.effort[i] = current_robot_state.tau_J[i];
    }
    publisher->publish(current_joint_state);
}

} //namespace franka_visualization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(franka_visualization::RobotJointStatePublisher)