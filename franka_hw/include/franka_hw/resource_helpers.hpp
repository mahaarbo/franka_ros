#pragma once

#include <list>
#include <map>
#include <string>
#include <vector>

#include <franka_hw/control_mode.h>
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace franka_hw {

/**
 * Struct representing the type of claim requested of each control type.
 */ 
struct ResourceClaims {
  uint8_t joint_position_claims = 0;
  uint8_t joint_velocity_claims = 0;
  uint8_t joint_torque_claims = 0;
  uint8_t cartesian_velocity_claims = 0;
  uint8_t cartesian_pose_claims = 0;
};

using ArmClaimedMap = std::map<std::string, ResourceClaims>;

/**
 * Populate an arm_claim_map from the requested interfaces, and hardware_info.
 * Uses info.hardware_parameters["arm_id"] to figure out which arm it is populating.
 * 
 * @param[in] interface_keys The interface keys requested
 * @param[in] info The HardwareInfo as processed from the URDF
 * @param[out] arm_claimed_map The resource claims associated with the arm_id
 */
void getArmClaimedMap(
  const std::vector<std::string> interface_keys,
  hardware_interface::HardwareInfo & info,
  ArmClaimedMap& arm_claim_map);

ControlMode getControlMode(const std::string& arm_id, ArmClaimedMap& arm_claim_map);

bool hasConflictingMultiClaim(const ArmClaimedMap& arm_claim_map,
                              const std::string& arm_id);

bool hasConflictingJointAndCartesianClaim(const ArmClaimedMap& arm_claim_map,
                                          const std::string& arm_id);

bool partiallyClaimsArmJoints(const ArmClaimedMap& arm_claim_map, const std::string& arm_id);

bool hasTrajectoryClaim(const ArmClaimedMap& arm_claim_map, const std::string& arm_id);

}  // namespace franka_hw