#ifndef FRANKA_HW__FRANKA_HW_HPP_
#define FRANKA_HW__FRANKA_HW_HPP_

#include <array>
#include <atomic>
#include <exception>
#include <functional>
#include <list>
#include <string>
#include <thread>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/exception.h>

#include <franka_hw/control_mode.h>
#include <franka_hw/resource_helpers.hpp>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::return_type;

namespace franka_hw {

/**
 * This class wraps the functionality of libfranka for controlling Panda robots into the ros2_control
 * framework
 */
class FrankaHW : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FrankaHW);
  
  FrankaHW();
  /**
   * Reads the parametrization of the hardware class from the System info
   * (e.g. arm_id, robot_ip, etc.).
   * 
   */
  virtual bool initParameters();
  
  /**
   * Runs the currently active controller in a realtime loop. If no controller is active, the
   * function immediately exits.
   * 
   * @throw franka::ControlException if an error related to torque control occurred.
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost, e.g. after a timeout.
   * @throw franka::RealtimeException if realtime priority cannot be set for the current thread.
   */
  void control() const;

  /**
   * Configures the FrankaHW class to be fully operational. This involves parsing required
   * configurations from the ROS2 parameters, initializing data structures, and verifying the 
   * interfaces for the ros2_control framework.
   * 
   * @param[in] system_info Hardware info obtained from the URDF.
   * 
   * @return return_type::OK if succesful, return_type::ERROR otherwise.
   */
  return_type configure(const hardware_interface::HardwareInfo & system_info) override;
  
  /**
   * Exports state interfaces for the ros2_control framework.
   * 
   * @return The state interfaces
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * Exports command interfaces for the ros2_control framework.
   * 
   * @return The command interfaces
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  /**
   * Prepares switching between controllers (not real-time capable).
   * 
   * @param[in] start_interfaces all command interface keys requested to start.
   * @param[in] stop_interfaces all command interface keys requested to stop.
   * 
   * @return return_type::OK if the switch is possible, return_type::ERROR otherwise.
   */
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  
  /**
   * Performs controller switching (real-time capable).
   */
  return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  /**
   * Start the hardware interface (handled by resource manager).
   * Connects to the robot.
   */
  return_type start() override;

  /**
   * Stop the hardware interface (handled by resource manager).
   * Disconnects from the robot.
   */
  return_type stop() override;

  /**
   * Reads data from the franka robot.
   */
  return_type read() override;

  /**
   * Writes data to the franka robot.
   */
  return_type write() override;

protected:
  /**
   * The control loop that actually connects to the robot and sends the commands/states.
   * 
   */
  void controlLoop();

  /**
   * Checks whether an array of doubles contains NaN values.
   * 
   * @param[in] command array The array to check.
   * 
   * @return True if the array contains NaN values, false otherwise.
   */
  template <size_t size>
  static bool arrayHasNaN(const std::array<double, size> & array) {
    return std::any_of(array.begin(), array.end(), [](const double& e) { return std::isnan(e); });
  }

  /**
   * 
   */
  template <typename T>
  T controlCallback(const T& command,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    if (commandHasNaN(command)) {
      std::string error_message = "FrankaHW::controlCallback: Got NaN command!";
      RCLCPP_FATAL(
        rclcpp::get_logger("FrankaHW"),
        error_message);
      throw std::invalid_argument(error_message);
    }
    // TODO: check joint limits?
    {
      std::lock_guard<std::mutex> state_lock(libfranka_state_mutex_);
      robot_state_libfranka_ = robot_state;
    }
    std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
    T current_cmd = command;
    if (has_error_ || !controller_active_) {
      return franka::MotionFinished(current_cmd);
    }
    return current_cmd;
  }

  /**
   * Configures the run function which is used as libfranka control callback based on the requested
   * control mode.
   * 
   * @param[in] requested_control_mode The control mode to configure (e.g. torque/position/velocity
   * etc.)
   * @param[in] limit_rate Flag to enable/disable rate limiting to smoothen the commands.
   * @param[in] cutoff_frequency The cutoff frequency applied for command smoothing.
   * @param[in] internal_controller The internal controller to use when using position or velocity
   * modes.
   * 
   * @return True if successful, false otherwise.
   */ 
  virtual bool setRunFunction(const ControlMode& requested_control_mode,
                              bool limit_rate,
                              double cutoff_frequency,
                              franka::ControllerMode internal_controller);
  /**
   * Uses the robot_ip_ to connect to the robot via libfranka and loads the libfranka model.
   */
  virtual void initRobot();

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::Torques& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointPositions& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointVelocities& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianPose& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianVelocities& command);

  /**
   * Parses a set of collision thresholds from the parameter server. The methods returns
   * the default values if no parameter was found or the size of the array did not match
   * the defaults dimension.
   *
   * @param[in] name The name of the parameter to look for.
   * @param[in] defaults A set of default values that also specify the size the parameter must have
   * to be valid.
   * @return A set parsed parameters if valid parameters where found, the default values otherwise.
   */
  static std::vector<double> getCollisionThresholds(const hardware_interface::HardwareInfo& info,
                                                    const std::string& name,
                                                    const std::vector<double>& defaults);

  struct CollisionConfig {
    std::array<double, 7> lower_torque_thresholds_acceleration;
    std::array<double, 7> upper_torque_thresholds_acceleration;
    std::array<double, 7> lower_torque_thresholds_nominal;
    std::array<double, 7> upper_torque_thresholds_nominal;
    std::array<double, 6> lower_force_thresholds_acceleration;
    std::array<double, 6> upper_force_thresholds_acceleration;
    std::array<double, 6> lower_force_thresholds_nominal;
    std::array<double, 6> upper_force_thresholds_nominal;
  };

  CollisionConfig collision_config_;
 
  std::mutex libfranka_state_mutex_;
  std::mutex ros_state_mutex_;
  franka::RobotState robot_state_libfranka_{};
  franka::RobotState robot_state_ros_{};

  std::mutex libfranka_cmd_mutex_;
  franka::JointPositions position_joint_command_libfranka_;
  franka::JointVelocities velocity_joint_command_libfranka_;
  franka::Torques effort_joint_command_libfranka_;

  std::mutex ros_cmd_mutex_;
  franka::JointPositions position_joint_command_ros_;
  franka::JointVelocities velocity_joint_command_ros_;
  franka::Torques effort_joint_command_ros_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;

  std::string arm_id_;
  std::string robot_ip_;
  double joint_limit_warning_threshold_{0.1};
  franka::RealtimeConfig realtime_config_;
  bool rate_limiting_;
  double cutoff_frequency_;
  franka::ControllerMode internal_controller_;

  std::unique_ptr<std::thread> control_loop_thread_;
  std::atomic_bool has_error_{false};
  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;

  std::function<void(franka::Robot&)> run_function_;
};
}  // namespace franka_hw

#endif  // FRANKA_HW__FRANKA_HW_HPP_ 