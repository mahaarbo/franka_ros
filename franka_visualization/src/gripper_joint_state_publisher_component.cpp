// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
#include "franka_visualization/gripper_joint_state_publisher_component.hpp"

namespace franka_visualization
{
using namespace std::chrono_literals;

GripperJointStatePublisher::GripperJointStatePublisher(const rclcpp::NodeOptions & options)
: Node("gripper_joint_state_publisher")
{
    this->declare_parameter<std::string>("robot_ip", "172.16.0.2");
    this->declare_parameter<std::vector<std::string>>("joint_names", 
        {
            "panda_finger_joint1",
            "panda_finger_joint2"
        }
    );
    this->declare_parameter<double>("publish_rate", 30.);
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    robot_ip = parameters_client->get_parameter<std::string>("robot_ip");
    joint_names = parameters_client->get_parameter<std::vector<std::string>>("joint_names");
    publish_rate = parameters_client->get_parameter<double>("publish_rate");
    
    // Initialize joint state message
    current_joint_state.name = joint_names;
    current_joint_state.position = {0., 0.};
    current_joint_state.velocity = {0., 0.};
    current_joint_state.effort = {0., 0.};
    RCLCPP_INFO(this->get_logger(), "Establishing connection to Franka Controller.");
    gripper = std::make_shared<franka::Gripper>(robot_ip);
    publisher = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    timer = create_wall_timer(
        std::chrono::duration<double>(1./publish_rate),
        std::bind(&GripperJointStatePublisher::on_timer, this)
    );

}

void GripperJointStatePublisher::on_timer()
{
    current_joint_state.header.stamp = rclcpp::Clock().now();
    current_gripper_state = gripper->readOnce();
    for (size_t i = 0; i < joint_names.size(); i++)
    {
        current_joint_state.position[i] = current_gripper_state.width * 0.5;
    }
    publisher->publish(current_joint_state);
}

} //namespace franka_visualization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(franka_visualization::GripperJointStatePublisher)