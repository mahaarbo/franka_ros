// Copyright 2021 Department of Engineering Cybernetics, NTNU.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/joint_position_example_controller2.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace franka_example_controllers
{

JointPositionExampleController::JointPositionExampleController()
: controller_interface::ControllerInterface(),
  joint_names_({}),
  elapsed_time_(std::chrono::milliseconds(0)) {}

controller_interface::return_type
JointPositionExampleController::init(const std::string & controller_name)
{
  // Initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration JointPositionExampleController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration JointPositionExampleController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::return_type
JointPositionExampleController::update()
{
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return controller_interface::return_type::OK;
  }
  elapsed_time_ = node_->now() - initial_time_;
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.seconds())) * 0.2;
  for (size_t i = 0; i < 7; i++) {
    if (i == 4) {
      command_interfaces_[i].set_value(initial_pose_[i] - delta_angle);
    } else {
      command_interfaces_[i].set_value(initial_pose_[i] + delta_angle);
    }
  }
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionExampleController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionExampleController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Starting JointPositionExampleController");
  
  initial_time_ = node_->now();
  for (size_t i = 0; i < 7; i++) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
JointPositionExampleController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  franka_example_controllers::JointPositionExampleController, controller_interface::ControllerInterface)
