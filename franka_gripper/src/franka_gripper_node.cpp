// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
// and
// https://github.com/ros2/demos/blob/dashing/composition/src/manual_composition.cpp
#include <memory>
#include "franka_gripper/franka_gripper_server.hpp"
#include <rclcpp/rclcpp.hpp>
#include <franka/exception.h>


int main (int argc, char * argv[])
{
    // Force flush of the stdout buffer
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources
    rclcpp::init(argc, argv);

    // Create an executor
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    
    // Start the gripper action server component
    try
    {
        auto gripper_action_server = std::make_shared<franka_gripper::FrankaGripperServer>(options);
        exec.add_node(gripper_action_server);
        exec.spin();
    }
    catch (const franka::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("franka_gripper_server"), std::string("Exception: ") + e.what());
        rclcpp::shutdown();

    }
    
    rclcpp::shutdown();
    return 0;
}