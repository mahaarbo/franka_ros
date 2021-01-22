// Author: Mathias Hauan Arbo
// Department of Engineering Cybernetics, NTNU, 2020
// Based on the work of Franka Emika
#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "franka_gripper/msg/grasp_epsilon.hpp"
#include "franka_gripper/action/grasp.hpp"
#include "franka_gripper/action/homing.hpp"
#include "franka_gripper/action/move.hpp"
#include "franka_gripper/action/stop.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "franka/gripper.h"
#include "franka/exception.h"

namespace franka_gripper
{
/**
 * @brief A component containing action servers for the Franka Emika gripper.
 * 
 * @author Mathias Hauan Arbo
 */
class FrankaGripperServer : public rclcpp::Node
{
public:
    using GraspEpsilon = franka_gripper::msg::GraspEpsilon;
    using Grasp = franka_gripper::action::Grasp;
    using GraspGoalHandle = rclcpp_action::ServerGoalHandle<Grasp>;
    using Homing = franka_gripper::action::Homing;
    using HomingGoalHandle = rclcpp_action::ServerGoalHandle<Homing>;
    using Move = franka_gripper::action::Move;
    using MoveGoalHandle = rclcpp_action::ServerGoalHandle<Move>;
    using Stop = franka_gripper::action::Stop;
    using StopGoalHandle = rclcpp_action::ServerGoalHandle<Stop>;
    FrankaGripperServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
private:
    // Parameters
    std::string robot_ip_;
    double default_speed_;
    GraspEpsilon default_grasp_epsilon_;
    double publish_rate_;
    std::vector<std::string> joint_names_;

    // Franka Gripper Connection
    std::shared_ptr<franka::Gripper> gripper_;

    // Actions
    rclcpp_action::Server<Grasp>::SharedPtr grasp_action_server_;
    rclcpp_action::GoalResponse handle_grasp_goal_(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Grasp::Goal> goal
    );
    rclcpp_action::CancelResponse handle_grasp_cancel_(
        const std::shared_ptr<GraspGoalHandle> goal_handle
    );
    void handle_grasp_accepted_(const std::shared_ptr<GraspGoalHandle> goal_handle);
    void grasp_execute_(const std::shared_ptr<GraspGoalHandle> goal_handle);
    
    rclcpp_action::Server<Homing>::SharedPtr homing_action_server_;
    rclcpp_action::GoalResponse handle_homing_goal_(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Homing::Goal> goal
    );
    rclcpp_action::CancelResponse handle_homing_cancel_(
        const std::shared_ptr<HomingGoalHandle> goal_handle
    );
    void handle_homing_accepted_(const std::shared_ptr<HomingGoalHandle> goal_handle);
    void homing_execute_(const std::shared_ptr<HomingGoalHandle> goal_handle);

    rclcpp_action::Server<Move>::SharedPtr move_action_server_;
    rclcpp_action::GoalResponse handle_move_goal_(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Move::Goal> goal
    );
    rclcpp_action::CancelResponse handle_move_cancel_(
        const std::shared_ptr<MoveGoalHandle> goal_handle
    );
    void handle_move_accepted_(const std::shared_ptr<MoveGoalHandle> goal_handle);
    void move_execute_(const std::shared_ptr<MoveGoalHandle> goal_handle);

    rclcpp_action::Server<Stop>::SharedPtr stop_action_server_;
    rclcpp_action::GoalResponse handle_stop_goal_(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Stop::Goal> goal
    );
    rclcpp_action::CancelResponse handle_stop_cancel_(
        const std::shared_ptr<StopGoalHandle> goal_handle
    );
    void handle_stop_accepted_(const std::shared_ptr<StopGoalHandle> goal_handle);
    void stop_execute_(const std::shared_ptr<StopGoalHandle> goal_handle);

    // Gripper joint state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    sensor_msgs::msg::JointState current_joint_state_;
    franka::GripperState current_gripper_state_;
    std::mutex gripper_state_mutex_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_timer_();
    rclcpp::TimerBase::SharedPtr gripper_state_timer_;
    void on_gripper_state_timer_();

    // Since gripper state is read in separate thread, we'll use 2 callback groups
    rclcpp::CallbackGroup::SharedPtr cb_group1_;
    rclcpp::CallbackGroup::SharedPtr cb_group2_;
}; // class FrankaGripperServer
} // franka_gripper