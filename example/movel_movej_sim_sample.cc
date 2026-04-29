#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "kinematic.h"
#include "mujoco_sim_ctrl.h"
#include "s6h4d_instance.h"

namespace {

void PrintJointState(const char* label, const ArmMotion::JointState_t& s) {
  const char* kJointNames[6] = {"shoulder", "upper_arm", "elbow",
                                "wrist1",   "wrist2",    "wrist3"};

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

bool PrintCurrentState(MujocoSimCtrl& ctrl, const char* title) {
  ArmMotion::JointState_t state{};
  if (!ctrl.GetJointState(state)) {
    std::cerr << "[ERROR] GetJointState failed: " << title << "\n";
    return false;
  }
  PrintJointState(title, state);
  return true;
}

}  // namespace

int main() {
  MujocoSimCtrl ctrl;
  if (!ctrl.Init()) {
    std::cerr << "[ERROR] MujocoSimCtrl::Init() failed\n";
    return 1;
  }

  ArmMotion::MDHConfig_t mdh_config{0.123, 0.04, 0.20, 0.059, 0.2, 0.104};
  S6h4dInstance robot(ctrl, mdh_config);
  KinematicSolver fk_solver(mdh_config);

  if (!PrintCurrentState(ctrl, "Initial state")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_target1(0.0, -0.840, 0.840, 0.0, 0.00, 0.00);
  std::cout << "\n[INFO] MoveJ to drawing start pose\n";
  if (robot.MoveJ(joint_target1, 0.8, 1.2, 2.0) != 0) {
    std::cerr << "[ERROR] MoveJ to drawing start pose failed\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After MoveJ to drawing start pose")) {
    return 1;
  }

  ArmMotion::JointState_t state_after_movej{};
  if (!ctrl.GetJointState(state_after_movej)) {
    std::cerr << "[ERROR] GetJointState failed before MoveL\n";
    return 1;
  }

  ArmMotion::Pose6Dof_t current_pose{};
  if (!fk_solver.FK(state_after_movej.position, current_pose)) {
    std::cerr << "[ERROR] FK failed before MoveL\n";
    return 1;
  }

  ArmMotion::Pose6Dof_t lateral_pose = current_pose;
  lateral_pose.y -= 0.12;

  std::cout << "\n[INFO] MoveL side translation (-Y 0.12m)\n";
  if (robot.MoveL(lateral_pose, 0.50, 0.40, 0.0) != 0) {
    std::cerr << "[ERROR] MoveL side translation failed\n";
    return 1;
  }
  lateral_pose.z -= 0.12;
  std::cout << "\n[INFO] MoveL side translation (-Z 0.12m)\n";
  if (robot.MoveL(lateral_pose, 0.50, 0.40, 0.0) != 0) {
    std::cerr << "[ERROR] MoveL side translation failed\n";
    return 1;
  }
  lateral_pose.y += 0.12;
  std::cout << "\n[INFO] MoveL side translation (+Y 0.12m)\n";
  if (robot.MoveL(lateral_pose, 0.50, 0.40, 0.0) != 0) {
    std::cerr << "[ERROR] MoveL side translation failed\n";
    return 1;
  }
  lateral_pose.z += 0.12;
  std::cout << "\n[INFO] MoveL side translation (+Z 0.12m)\n";
  if (robot.MoveL(lateral_pose, 0.50, 0.40, 0.0) != 0) {
    std::cerr << "[ERROR] MoveL side translation failed\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After lateral MoveL")) {
    return 1;
  }

  ArmMotion::Joint6Dof_t joint_origin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  std::cout << "\n[INFO] MoveJ back to joint origin\n";
  if (robot.MoveJ(joint_origin, 0.8, 1.2, 2.5) != 0) {
    std::cerr << "[ERROR] MoveJ back to joint origin failed\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After MoveJ to origin")) {
    return 1;
  }

  std::cout << "\n[INFO] MoveJ && MoveL demo completed successfully\n";
  return 0;
}
