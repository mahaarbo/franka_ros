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

#ifndef JOINT_POSITION_EXAMPLE_CONTROLLER__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_
#define JOINT_POSITION_EXAMPLE_CONTROLLER__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_

#include <array>
#include <vector>
#include <string>

#include "franka_example_controllers/visibility_control.h"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

namespace franka_example_controllers {

class JointPositionExampleController : public controller_interface::ControllerInterface
{
public:
  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  JointPositionExampleController();

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  controller_interface::return_type
  init(const std::string & controller_name) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  controller_interface::return_type
  update() override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FRANKA_EXAMPLE_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;
  std::array<double, 7> initial_pose_{};
  rclcpp::Duration elapsed_time_;
  rclcpp::Time initial_time_;
};
}  // namespace franka_example_controllers

#endif  // JOINT_POSITION_EXAMPLE_CONTROLLER__JOINT_POSITION_EXAMPLE_CONTROLLER_HPP_