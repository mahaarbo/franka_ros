/* Author: Mathias Hauan Arbo
*  Department of Engineering Cybernetics, NTNU, 2020
*  Based on the work of Franka Emika
*/
#include <franka_gripper/franka_gripper_server.hpp>

#include <functional>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;


namespace franka_gripper
{
FrankaGripperServer::FrankaGripperServer(const rclcpp::NodeOptions& options)
: Node("franka_gripper_server")
{
    RCLCPP_INFO(this->get_logger(), "Starting up franka_gripper_server");
    // Declare Parameters
    this->declare_parameter<std::string>("robot_ip", "172.16.0.2");
    this->declare_parameter<double>("default_speed", 0.1);
    std::map<std::string, double> default_grasp_epsilon_map = {
        {"inner", 0.005},
        {"outer", 0.005}
    };
    this->declare_parameters("default_grasp_epsilon", default_grasp_epsilon_map);
    this->declare_parameter<double>("publish_rate", 30.);
    this->declare_parameter<std::vector<std::string>>("joint_names",
        {
            "panda_finger_joint1",
            "panda_finger_joint2"
        }
    );
    // Get parameters
    this->get_parameter("robot_ip", robot_ip_);
    this->get_parameter<double>("default_speed", default_speed_);
    std::map<std::string, double> epsilon_map;
    this->get_parameters("default_grasp_epsilon", epsilon_map);
    default_grasp_epsilon_.inner = epsilon_map["inner"];
    default_grasp_epsilon_.outer = epsilon_map["outer"];
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("joint_names", joint_names_);

    // Start connection to franka gripper
    try{
        gripper_ = std::make_shared<franka::Gripper>(robot_ip_);
    } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), ex.what());
        return;
    }

    // Start action servers
    this->grasp_action_server_ = rclcpp_action::create_server<Grasp>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "grasp",
        std::bind(&FrankaGripperServer::handle_grasp_goal_, this, _1, _2),
        std::bind(&FrankaGripperServer::handle_grasp_cancel_, this, _1),
        std::bind(&FrankaGripperServer::handle_grasp_accepted_, this, _1)
    );
    this->homing_action_server_ = rclcpp_action::create_server<Homing>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "homing",
        std::bind(&FrankaGripperServer::handle_homing_goal_, this, _1, _2),
        std::bind(&FrankaGripperServer::handle_homing_cancel_, this, _1),
        std::bind(&FrankaGripperServer::handle_homing_accepted_, this, _1)
    );
    this->move_action_server_ = rclcpp_action::create_server<Move>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "move",
        std::bind(&FrankaGripperServer::handle_move_goal_, this, _1, _2),
        std::bind(&FrankaGripperServer::handle_move_cancel_, this, _1),
        std::bind(&FrankaGripperServer::handle_move_accepted_, this, _1)
    );
    this->stop_action_server_ = rclcpp_action::create_server<Stop>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "stop",
        std::bind(&FrankaGripperServer::handle_stop_goal_, this, _1, _2),
        std::bind(&FrankaGripperServer::handle_stop_cancel_, this, _1),
        std::bind(&FrankaGripperServer::handle_stop_accepted_, this, _1)
    );
}

// Grasp action
rclcpp_action::GoalResponse FrankaGripperServer::handle_grasp_goal_(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Grasp::Goal> goal
)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FrankaGripperServer::handle_grasp_cancel_(
    const std::shared_ptr<GraspGoalHandle> goal_handle
)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void FrankaGripperServer::handle_grasp_accepted_(const std::shared_ptr<GraspGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Handling grasp accepted");
    std::thread([this, goal_handle](){this->grasp_execute_(goal_handle);}).detach();
}
void FrankaGripperServer::grasp_execute_(const std::shared_ptr<GraspGoalHandle> goal_handle)
{
    auto result = std::make_shared<Grasp::Result>();
    auto goal = goal_handle->get_goal();
    try{
        result->success = gripper_->grasp(
            goal->width, goal->speed, goal->force, 
            goal->epsilon.inner, goal->epsilon.outer);
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
        }
    } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), ex.what());
        result->success = false;
        result->error = ex.what();
        if (rclcpp::ok()) {
            goal_handle->abort(result);
        }
    }
}

// Homing action
rclcpp_action::GoalResponse FrankaGripperServer::handle_homing_goal_(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Homing::Goal> goal
)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FrankaGripperServer::handle_homing_cancel_(
    const std::shared_ptr<HomingGoalHandle> goal_handle
)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void FrankaGripperServer::handle_homing_accepted_(const std::shared_ptr<HomingGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Handling home accepted");
    std::thread([this, goal_handle](){this->homing_execute_(goal_handle);}).detach();
}
void FrankaGripperServer::homing_execute_(const std::shared_ptr<HomingGoalHandle> goal_handle)
{
    auto result = std::make_shared<Homing::Result>();
    try{
        result->success = gripper_->homing();
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
        }
    } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), ex.what());
        result->success = false;
        result->error = ex.what();
        if (rclcpp::ok()) {
            goal_handle->abort(result);
        }
    }
}

// Move action
rclcpp_action::GoalResponse FrankaGripperServer::handle_move_goal_(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal
)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FrankaGripperServer::handle_move_cancel_(
    const std::shared_ptr<MoveGoalHandle> goal_handle
)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void FrankaGripperServer::handle_move_accepted_(const std::shared_ptr<MoveGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Handling move accepted");
    std::thread([this, goal_handle](){this->move_execute_(goal_handle);}).detach();
}
void FrankaGripperServer::move_execute_(const std::shared_ptr<MoveGoalHandle> goal_handle)
{
    auto result = std::make_shared<Move::Result>();
    auto goal = goal_handle->get_goal();
    try{
        result->success = gripper_->move(goal->width, goal->speed);
        if (rclcpp::ok()) {
            goal_handle->succeed(result);
        }
    } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), ex.what());
        result->success = false;
        result->error = ex.what();
        if (rclcpp::ok()) {
            goal_handle->abort(result);
        }
    }
}

// Stop action
rclcpp_action::GoalResponse FrankaGripperServer::handle_stop_goal_(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Stop::Goal> goal
)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse FrankaGripperServer::handle_stop_cancel_(
    const std::shared_ptr<StopGoalHandle> goal_handle
)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
void FrankaGripperServer::handle_stop_accepted_(const std::shared_ptr<StopGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Handling stop accepted");
    std::thread([this, goal_handle](){this->stop_execute_(goal_handle);}).detach();
}
void FrankaGripperServer::stop_execute_(const std::shared_ptr<StopGoalHandle> goal_handle)
{
    auto result = std::make_shared<Stop::Result>();
    try{
        result->success = gripper_->stop();
    } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(this->get_logger(), ex.what());
        result->success = false;
        result->error = ex.what();
        if (rclcpp::ok()) {
            goal_handle->abort(result);
        }
    }
}
} // franka_gripper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::FrankaGripperServer)