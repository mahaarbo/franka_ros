// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
#ifndef FRANKA_VISUALIZATION__GRIPPER_STATE_PUBLISHER_COMPONENT_
#define FRANKA_VISUALIZATION__GRIPPER_STATE_PUBLISHER_COMPONENT_
#include <string>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "franka/gripper.h"

namespace franka_visualization
{

class GripperJointStatePublisher: public rclcpp::Node
{
public:
    explicit GripperJointStatePublisher(const rclcpp::NodeOptions & options);
private:
    void on_timer();
    std::string robot_ip;
    double publish_rate;
    std::vector<std::string> joint_names;
    std::shared_ptr<franka::Gripper> gripper;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    franka::GripperState current_gripper_state;
    sensor_msgs::msg::JointState current_joint_state;
};

} //namespace franka_visualization
#endif //FRANKA_VISUALIZATION__GRIPPER_STATE_PUBLISHER_COMPONENT_