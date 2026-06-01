#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_instance.h"
#include "arm_logger.h"
#include "kinematic.h"
#include "sim_ctrl.h"

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
              << s.position.theta[i] << std::setw(12) << s.velocity.theta[i]
              << std::setw(12) << s.effort.theta[i] << "\n";
  }
}

bool PrintCurrentState(SimCtrl& ctrl, const char* title) {
  ArmMotion::JointState_t state{};
  if (!ctrl.GetJointState(state)) {
    ARM_LOG_ERROR("GetJointState failed: {}", title);
    return false;
  }
  PrintJointState(title, state);
  return true;
}

}  // namespace

int main() {
  SimCtrl ctrl;
  if (!ctrl.Init()) {
    ARM_LOG_ERROR("SimCtrl::Init() failed");
    return 1;
  }

  // ArmMotion::MDHParams mdh_params{
  //     .alpha = {0.0, -M_PI_2, 0.0, -M_PI_2, M_PI_2, -M_PI_2},
  //     .a = {0.0, 0.04, 0.20, 0.059, 0.0, 0.0},
  //     .d = {0.123, 0.0, 0.0, 0.2, 0.0, 0.104},
  //     .theta = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0}};
  ArmMotion::MDHParams mdh_params = {
    .alpha = {0.0, -M_PI_2, 0.0, 0.0, -M_PI_2, M_PI_2},  // alpha
    .a = {0.0, 0.02, 0.26, 0.24668, 0.075, 0.0},             // a
    .d = {0.1207, 0.0, 0.0, 0.0, 0.0, 0.0265},       // d
    .theta = {0.0, -M_PI, (M_PI - 0.233176), 0.233176, M_PI_2, 0.0}          // theta (初始偏移量)
  };
  ArmInstance robot(ctrl, mdh_params, KinematicSolver::IKMode::ParallelWrist);
  KinematicSolver fk_solver(mdh_params);

  if (!PrintCurrentState(ctrl, "Initial state")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_target1(0.0, 1, -1, 0.0, 0.00, 0.00);
  ARM_LOG_INFO("MoveJ to drawing start pose");
  if (robot.MoveJ(joint_target1, 2, 3, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJ to drawing start pose failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After MoveJ to drawing start pose")) {
    return 1;
  }

  ArmMotion::JointState_t state_after_movej{};
  if (!ctrl.GetJointState(state_after_movej)) {
    ARM_LOG_ERROR("GetJointState failed before MoveL");
    return 1;
  }

  ArmMotion::Pose6Dof_t current_pose{};
  if (!fk_solver.FK(state_after_movej.position, current_pose)) {
    ARM_LOG_ERROR("FK failed before MoveL");
    return 1;
  }

  ArmMotion::Pose6Dof_t lateral_pose = current_pose;
  lateral_pose.y -= 0.12;

  ARM_LOG_INFO("MoveL side translation (-Y 0.12m)");
  if (robot.MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.z -= 0.12;
  ARM_LOG_INFO("MoveL side translation (-Z 0.12m)");
  if (robot.MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.y += 0.12;
  ARM_LOG_INFO("MoveL side translation (+Y 0.12m)");
  if (robot.MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }
  lateral_pose.z += 0.12;
  ARM_LOG_INFO("MoveL side translation (+Z 0.12m)");
  if (robot.MoveL(lateral_pose, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveL side translation failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After lateral MoveL")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_origin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ARM_LOG_INFO("MoveJ back to joint origin");
  if (robot.MoveJ(joint_origin, 2, 3, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJ back to joint origin failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After MoveJ to origin")) {
    return 1;
  }

  ARM_LOG_INFO("MoveJ && MoveL demo completed successfully");
  return 0;
}
