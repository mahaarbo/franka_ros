#include "franka_hw/franka_hw2.hpp"

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <unordered_map>
#include <string>

namespace franka_hw
{

bool has_parameter(
  const hardware_interface::HardwareInfo & system_info,
  const std::string key)
{
  return system_info.hardware_parameters.find(key) != system_info.hardware_parameters.end();
}

FrankaHW::FrankaHW()
    : position_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      position_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

bool FrankaHW::initParameters()
{
  // Hardware parameters
  if (!has_parameter(info_, "rate_limiting")) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Invalid or no rate_limiting parameter provided");
    return false;
  }
  rate_limiting_ = info_.hardware_parameters["rate_limiting"] == "true";

  if (!has_parameter(info_, "cutoff_frequency")) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Invalid or no cutoff_frequency parameter provided");
    return false;
  }
  cutoff_frequency_ = stod(info_.hardware_parameters["cutoff_frequency"]);
  
  if (!has_parameter(info_, "internal_controller")) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "No internal_controller parameter provided");
    return false;
  }
  std::string internal_controller = info_.hardware_parameters["internal_controller"];
  if (internal_controller == "joint_impedance") {
    internal_controller_ = franka::ControllerMode::kJointImpedance;
  } else if (internal_controller == "cartesian_impedance") {
    internal_controller_ = franka::ControllerMode::kCartesianImpedance;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("FrankaHW"),
                "Invalid internal_controller parameter provided, falling back to joint impedance");
    internal_controller_ = franka::ControllerMode::kJointImpedance;
  }

  if (!has_parameter(info_, "arm_id")) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Invalid or no arm_id parameter provided");
    return false;
  }
  arm_id_ = info_.hardware_parameters["arm_id"];

  if (!has_parameter(info_, "robot_ip")) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Invalid or no robot_ip parameter provided");
    return false;
  }
  robot_ip_ = info_.hardware_parameters["robot_ip"];

  if (!has_parameter(info_, "joint_limit_warning_threshold")) {
    RCLCPP_WARN(rclcpp::get_logger("FrankaHW"),
                "No parameter joint_value_limit_warning_threshold is found, using default"
                "value %f",
                joint_limit_warning_threshold_);
  } else {
    joint_limit_warning_threshold_ = stod(info_.hardware_parameters["joint_limit_warning_threshold"]);
  }

  std::string realtime_config_param;
  if (!has_parameter(info_, "realtime_config")) {
    realtime_config_param = "enforce";
  } else {
    realtime_config_param = info_.hardware_parameters["realtime_config"];
  }
  if (realtime_config_param == "enforce") {
    realtime_config_ = franka::RealtimeConfig::kEnforce;
  } else if (realtime_config_param == "ignore") {
    realtime_config_ = franka::RealtimeConfig::kIgnore;
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"), 
                 "Invalid realtime_config parameter provided. Got '%s', expected 'enforce' or 'ignore'.",
                 realtime_config_param.c_str());
    return false;
  }
  // Get full collision behavior config from info
  std::vector<double> thresholds =
    getCollisionThresholds(info_,
                           "lower_torque_thresholds_acceleration",
                           {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds(info_,
                                      "upper_torque_thresholds_acceleration",
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds(info_,
                                      "lower_torque_thesholds_nominal",
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_nominal.begin());
  thresholds = getCollisionThresholds(info_,
                                      "upper_torque_thresholds_nominal",
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_torque_thresholds_nominal.begin());
  thresholds.resize(6);
  thresholds = getCollisionThresholds(info_,
                                      "lower_force_thresholds_acceleration",
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds(info_,
                                      "upper_force_thresholds_acceleration",
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds(info_,
                                      "lower_force_thresholds_nominal",
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_nominal.begin());
  thresholds = getCollisionThresholds(info_,
                                      "upper_force_thresholds_nominal",
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_nominal.begin());
  return true;
}

void FrankaHW::initRobot() {
  robot_ = std::make_unique<franka::Robot>(robot_ip_, realtime_config_);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  robot_->setCollisionBehavior(collision_config_.lower_torque_thresholds_acceleration,
                               collision_config_.upper_torque_thresholds_acceleration,
                               collision_config_.lower_torque_thresholds_nominal,
                               collision_config_.upper_torque_thresholds_nominal,
                               collision_config_.lower_force_thresholds_acceleration,
                               collision_config_.upper_force_thresholds_acceleration,
                               collision_config_.lower_force_thresholds_nominal,
                               collision_config_.upper_force_thresholds_nominal);
  update(robot_->readOnce());
  control_loop_thread_ = std::make_unique<std::thread>(&FrankaHW::controlLoop, this);
}

void FrankaHW::controlLoop() {
  while (rclcpp::ok) {
    while (!controller_active_ || has_error_) {
      // TODO: !controllerActive debug print, has_error_ debug print, should clock be instantiated in init?

      {
        std::lock_guard<std::mutex> ros_state_lock(ros_state_mutex_);
        std::lock_guard<std::mutex> libfranka_state_lock(libfranka_state_mutex_);
        robot_state_libfranka_ = robot_->readOnce();
        robot_state_ros_ = robot_->readOnce();
      }

      if (!rclcpp::ok) {
        return;
      }
    }
    // Reset commands
    {
      std::lock_guard <std::mutex> command_lock(libfranka_cmd_mutex_);
      velocity_joint_command_libfranka_ = franka::JointVelocities({0., 0., 0., 0., 0., 0., 0.});
      effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    }

    try {
      control();
    } catch (const franka::ControlException& e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FrankaHW"),
        "%s: %s", arm_id_.c_str(), e.what());
      has_error_ = true;
      // Todo: publish error state?
    }
  }
}

void FrankaHW::control() const {
  if (!controller_active_) {
    return;
  }
  run_function_(*robot_);
}

void FrankaHW::update(const franka::RobotState& robot_state) {
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  robot_state_ros_ = robot_state;
}

return_type FrankaHW::configure(
    const hardware_interface::HardwareInfo & system_info)
{
  if (configure_default(system_info) != return_type::OK) {
    return return_type::ERROR;
  }

  // Initialize 
  position_joint_command_ros_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  position_joint_command_libfranka_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  velocity_joint_command_ros_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  velocity_joint_command_libfranka_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  effort_joint_command_ros_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  effort_joint_command_libfranka_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Verify system_info
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%d' command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first command interface. '%s' expected",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first command interface. '%s' expected",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first command interface. '%s' expected",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%d' state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first state interface. '%s' expected",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first state interface. '%s' expected",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                   "Joint '%s' has '%s' found as first state interface. '%s' expected",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }
  }

  if (!initParameters()) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Failed to parse all required parameters.");
    return return_type::ERROR;
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> FrankaHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &robot_state_ros_.q[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &robot_state_ros_.dq[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &robot_state_ros_.tau_J[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FrankaHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_joint_command_ros_.q[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_joint_command_ros_.dq[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_joint_command_ros_.tau_J[i]));
  }

  return command_interfaces;
}

return_type FrankaHW::start()
{
  RCLCPP_INFO(rclcpp::get_logger("FrankaHW"), "Starting FrankaHW..");

  if (!initParameters()) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Failed to parse all required parameters.");
    return return_type::ERROR;
  }

  try {
    initRobot();
  } catch (const std::runtime_error& error) {
    RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"),
                 "Failed to initialize libfranka robot. %s",
                 error.what());
    return return_type::ERROR;
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip_, realtime_config_);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  return return_type::OK;
}

bool FrankaHW::setRunFunction(
  const ControlMode& requested_control_mode,
  bool limit_rate,
  double cutoff_frequency,
  franka::ControllerMode internal_controller) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  switch (requested_control_mode) {
    case ControlMode::None:
      break;
    case ControlMode::JointTorque:
      run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot) {
        robot.control(
          std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                    std::cref(effort_joint_command_libfranka_), _1, _2),
          limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [this, limit_rate, cutoff_frequency, internal_controller](franka::Robot& robot) {
        robot.control(
          std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                    std::cref(velocity_joint_command_libfranka_), _1, _2),
          internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [this, limit_rate, cutoff_frequency, internal_controller](franka::Robot& robot) {
        robot.control(
          std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                    std::cref(position_joint_command_libfranka_), _1, _2),
          internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot) {
        robot.control(
          std::bind(&FrankaHW::controlCallback<franka::Torques>, this, 
                    std::cref(effort_joint_command_libfranka_), _1, _2),
          std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                    std::cref(position_joint_command_libfranka_), _1, _2),
          limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot) {
        robot.control(
          std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                    std::cref(effort_joint_command_libfranka_), _1, _2),
          std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                    std::cref(velocity_joint_command_libfranka_), _1, _2),
          limit_rate, cutoff_frequency);
      };
      break;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("FrankaHW"),
        "No valid control mode selected; cannot switch controllers.");
      return false;
  }
  return true;
}

bool FrankaHW::commandHasNaN(const franka::Torques& command) {
  return arrayHasNaN(command.tau_J);
}

bool FrankaHW::commandHasNaN(const franka::JointPositions& command) {
  return arrayHasNaN(command.q);
}

bool FrankaHW::commandHasNaN(const franka::JointVelocities& command) {
  return arrayHasNaN(command.dq);
}

bool FrankaHW::commandHasNaN(const franka::CartesianPose& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_T_EE);
}

bool FrankaHW::commandHasNaN(const franka::CartesianVelocities& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_dP_EE);
}

std::vector<double> FrankaHW::getCollisionThresholds(
  const hardware_interface::HardwareInfo& info,
  const std::string& name,
  const std::vector<double>& defaults)
{
  std::vector<double> thresholds;
  std::string key;
  thresholds.resize(defaults.size());
  for (size_t i = 0; i < defaults.size(); i++) {
    key = name + std::to_string(i);
    if (!has_parameter(info, key)) {
      RCLCPP_INFO(rclcpp::get_logger("FrankaHW"),
                  "No parameter %s found, using default value: %s",
                  key.c_str(), std::to_string(defaults[i]).c_str());
      thresholds[i] = defaults[i];
    } else {
      thresholds[i] = std::stod(info.hardware_parameters.at(key));
    }
  }
  return thresholds;
}

return_type FrankaHW::read() 
{
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_state_mutex_);
  robot_state_ros_ = robot_state_libfranka_;
  return return_type::OK;
}

return_type FrankaHW::write()
{
  std::lock_guard<std::mutex> ros_lock(ros_cmd_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_cmd_mutex_);
  effort_joint_command_libfranka_ = effort_joint_command_ros_;
  position_joint_command_libfranka_ = position_joint_command_ros_;
  velocity_joint_command_libfranka_ = velocity_joint_command_ros_;
  return return_type::OK;
}

return_type FrankaHW::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces) {
  ArmClaimedMap start_arm_claim_map;
  getArmClaimedMap(start_interfaces, info_, start_arm_claim_map);
  
  // Check for conflict
  if (hasConflictingMultiClaim(start_arm_claim_map, arm_id_) ||
      hasConflictingJointAndCartesianClaim(start_arm_claim_map, arm_id_)) {
    return return_type::ERROR;
  }
  ControlMode start_control_mode = getControlMode(arm_id_, start_arm_claim_map);

  ArmClaimedMap stop_arm_claim_map;
  getArmClaimedMap(stop_interfaces, info_, stop_arm_claim_map);

  ControlMode stop_control_mode = getControlMode(arm_id_, stop_arm_claim_map);

  ControlMode requested_control_mode = current_control_mode_;
  requested_control_mode &= ~stop_control_mode;
  requested_control_mode |= start_control_mode;

  if (!setRunFunction(requested_control_mode, rate_limiting_, cutoff_frequency_, internal_controller_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FrankaHW"),
      "Error switching due to setRunFunction.");
    return return_type::ERROR;
  }
  if (current_control_mode_ != requested_control_mode) {
    RCLCPP_INFO(
      rclcpp::get_logger("FrankaHW"),
      "Prepared switching controllers to %i with parameters "
      "limit_rate=%i, cutoff_frequency=%f, internal_controller=%i",
      requested_control_mode, rate_limiting_, cutoff_frequency_, internal_controller_);
    current_control_mode_ = requested_control_mode;
    controller_active_ = false;
  }

  return return_type::OK;

}

return_type FrankaHW::perform_command_mode_switch(
  const std::vector<std::string>& /*start_interfaces*/,
  const std::vector<std::string>& /*stop_interfaces*/) {
  if (current_control_mode_ != ControlMode::None) {
    //reset();
    controller_active_ = true;
  }
  return return_type::OK;
}

}  // namespace franka_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  franka_hw::FrankaHW,
  hardware_interface::SystemInterface)
