#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_instance.h"
#include "arm_logger.h"
#include "robo_manager.h"

namespace {

void PrintJointState(const char* label, const ArmMotion::JointState_t& s) {
  const char* kJointNames[6] = {"shoulder", "upper_arm", "elbow",
                                "wrist1", "wrist2", "wrist3"};

  std::cout << "\n=== " << label << " ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << std::left << std::setw(12) << "joint" << std::setw(12)
            << "pos(rad)" << std::setw(12) << "vel(rad/s)" << std::setw(12)
            << "effort(Nm)"
            << "\n";
  std::cout << std::string(48, '-') << "\n";
  for (int i = 0; i < 6; ++i) {
    std::cout << std::left << std::setw(12) << kJointNames[i] << std::setw(12)
              << s.position.q[i] << std::setw(12) << s.velocity.q[i]
              << std::setw(12) << s.effort.q[i] << "\n";
  }
}

bool PrintCurrentState(ArmInstance& arm, const char* title) {
  ArmMotion::JointState_t state{};
  if (arm.GetCurrentJoint(state.position) != 0) {
    ARM_LOG_ERROR("GetCurrentJoint failed: {}", title);
    return false;
  }
  PrintJointState(title, state);
  return true;
}

}  // namespace

int main() {
  RoboManager mgr("geniarm_sim_config.yaml");
  auto arm = mgr.GetArmInstance();

  if (!PrintCurrentState(*arm, "Initial state")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_target1(0.0, 1, -1, 0.0, 0.00, 0.00);
  ARM_LOG_INFO("MoveJ to drawing start pose");
  if (arm->MoveJ(joint_target1, 2, 3, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJ to drawing start pose failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(*arm, "After MoveJ to drawing start pose")) {
    return 1;
  }

  ArmMotion::Pose6Dof_t current_pose{};
  if (arm->GetCurrentPose(current_pose) != 0) {
    ARM_LOG_ERROR("GetCurrentPose failed before MoveL");
    return 1;
  }

  ArmMotion::Pose6Dof_t lateral_pose = current_pose;
  lateral_pose.y -= 0.12;

  ARM_LOG_INFO("MoveL side translation (-Y 0.12m)");
  if (arm->MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.z -= 0.12;
  ARM_LOG_INFO("MoveL side translation (-Z 0.12m)");
  if (arm->MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.y += 0.12;
  ARM_LOG_INFO("MoveL side translation (+Y 0.12m)");
  if (arm->MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.z += 0.12;
  ARM_LOG_INFO("MoveL side translation (+Z 0.12m)");
  if (arm->MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(*arm, "After lateral MoveL")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_origin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ARM_LOG_INFO("MoveJ back to joint origin");
  if (arm->MoveJ(joint_origin, 2, 3, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJ back to joint origin failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(*arm, "After MoveJ to origin")) {
    return 1;
  }

  ARM_LOG_INFO("MoveJ && MoveL demo completed successfully");
  return 0;
}
