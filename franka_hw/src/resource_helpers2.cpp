#include <franka_hw/resource_helpers.hpp>

#include "rclcpp/rclcpp.hpp"

namespace franka_hw {


void getArmClaimedMap(
  const std::vector<std::string> interface_keys,
  hardware_interface::HardwareInfo& info,
  ArmClaimedMap& arm_claim_map) {
  
  std::string current_arm_id = info.hardware_parameters["arm_id"];
  ResourceClaims new_claim;
  for (std::string key : interface_keys) {
    for (std::size_t i = 0; i < info.joints.size(); i++) {
      if (key == info.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        new_claim.joint_torque_claims++;
      } else if (key == info.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        new_claim.joint_velocity_claims++;
      } else if (key == info.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        new_claim.joint_position_claims++;
      }
    }
  }
  arm_claim_map[current_arm_id].joint_position_claims += new_claim.joint_position_claims;
  arm_claim_map[current_arm_id].joint_velocity_claims += new_claim.joint_velocity_claims;
  arm_claim_map[current_arm_id].joint_torque_claims += new_claim.joint_torque_claims;
  arm_claim_map[current_arm_id].cartesian_velocity_claims += new_claim.cartesian_velocity_claims;
  arm_claim_map[current_arm_id].cartesian_pose_claims += new_claim.cartesian_pose_claims;
}

ControlMode getControlMode(const std::string& arm_id, ArmClaimedMap& arm_claim_map) {
  ControlMode control_mode = ControlMode::None;
  if (arm_claim_map[arm_id].joint_position_claims > 0 &&
      arm_claim_map[arm_id].joint_velocity_claims == 0 &&
      arm_claim_map[arm_id].joint_torque_claims == 0 &&
      arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
      arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointPosition;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims > 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims > 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::CartesianPose;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims > 0) {
    control_mode = ControlMode::CartesianVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims > 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::JointPosition;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims > 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::JointVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims > 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::CartesianPose;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims > 0) {
    control_mode = ControlMode::JointTorque | ControlMode::CartesianVelocity;
  }
  return control_mode;
}

bool hasConflictingMultiClaim(const ArmClaimedMap& arm_claim_map,
                              const std::string& arm_id) {
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    uint8_t claims = 0;
    if (arm_claim_map.at(arm_id).joint_position_claims > 0) {
      claims++;
    }
    if (arm_claim_map.at(arm_id).joint_velocity_claims > 0) {
      claims++;
    }
    if (arm_claim_map.at(arm_id).cartesian_pose_claims > 0) {
      claims++;
    }
    if (arm_claim_map.at(arm_id).cartesian_velocity_claims > 0) {
      claims++;
    }
    if (claims > 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FrankaHW"),
        "Resource conflict: Illegal combination of command resource claims."
      );
      return true;
    }
  }
  return false;
}

bool hasConflictingJointAndCartesianClaim(const ArmClaimedMap& arm_claim_map,
                                          const std::string& arm_id) {
  // check for conflicts between joint and cartesian level for each arm.
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if ((arm_claim_map.at(arm_id).cartesian_velocity_claims +
                 arm_claim_map.at(arm_id).cartesian_pose_claims >
             0 &&
         arm_claim_map.at(arm_id).joint_position_claims +
                 arm_claim_map.at(arm_id).joint_velocity_claims >
             0)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FrankaHW"),
        "Resource conflict: Invalid combination of claims on joint AND cartesian level on arm %s.",
        arm_id);
      return true;
    }
  }
  return false;
}

bool partiallyClaimsArmJoints(const ArmClaimedMap& arm_claim_map, const std::string& arm_id) {
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if ((arm_claim_map.at(arm_id).joint_position_claims > 0 &&
         arm_claim_map.at(arm_id).joint_position_claims != 7) ||
        (arm_claim_map.at(arm_id).joint_velocity_claims > 0 &&
         arm_claim_map.at(arm_id).joint_velocity_claims != 7) ||
        (arm_claim_map.at(arm_id).joint_torque_claims > 0 &&
         arm_claim_map.at(arm_id).joint_torque_claims != 7)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("FrankaHW"),
        "Resource conflict: Partially claiming joints of arm %s is not supported. Make sure to claim all 7 joints of the robot.",
        arm_id);
      return true;
    }
  }
  return false;
}

bool hasTrajectoryClaim(const ArmClaimedMap& arm_claim_map, const std::string& arm_id) {
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if (arm_claim_map.at(arm_id).joint_position_claims +
            arm_claim_map.at(arm_id).joint_velocity_claims +
            arm_claim_map.at(arm_id).cartesian_velocity_claims +
            arm_claim_map.at(arm_id).cartesian_pose_claims >
        0) {
      return true;
    }
  }
  return false;
}
}  // namespace franka_hw