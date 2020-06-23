// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
// and
// https://github.com/ros2/demos/blob/dashing/composition/src/manual_composition.cpp
#include <memory>
#include "franka_visualization/robot_joint_state_publisher_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "franka/exception.h"

int main (int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources
    rclcpp::init(argc, argv);

    // Create an executor
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    // Start the robot joint state publisher component
    try
        {
        auto robot_joint_state_publisher = std::make_shared<franka_visualization::RobotJointStatePublisher>(options);
        exec.add_node(robot_joint_state_publisher);
        exec.spin();
    }
    catch(const franka::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("robot_joint_state_publisher"), std::string("Exception: ") + e.what());
        rclcpp::shutdown();
    }
    rclcpp::shutdown();
    return 0;
}