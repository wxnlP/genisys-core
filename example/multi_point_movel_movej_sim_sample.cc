#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

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

  // 1) 多点关节轨迹（抛物线融合）
  std::vector<ArmMotion::Joint6Dof_t> joint_points;
  joint_points.emplace_back(0.00, 0.75, -0.80, 0.00, 0.10, 0.00);
  joint_points.emplace_back(0.20, 0.95, -0.95, 0.10, 0.10, -0.10);
  joint_points.emplace_back(-0.15, 0.85, -0.90, -0.05, 0.15, 0.05);
  joint_points.emplace_back(0.00, 0.70, -0.75, 0.00, 0.10, 0.00);

  ARM_LOG_INFO("MoveJMulti demo: 4 waypoints, blend radius=0.08 rad, v=0.5 rad/s");
  if (robot.MoveJMulti(joint_points, 0.08, 2.0, 3.0, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJMulti failed");
    return 1;
  }

  if (!PrintCurrentState(ctrl, "After MoveJMulti")) {
    return 1;
  }

  // 2) 多点笛卡尔轨迹（圆弧融合）
  ArmMotion::JointState_t current_state{};
  if (!ctrl.GetJointState(current_state)) {
    ARM_LOG_ERROR("GetJointState failed before MoveLMulti");
    return 1;
  }

  ArmMotion::Pose6Dof_t start_pose{};
  if (!fk_solver.FK(current_state.position, start_pose)) {
    ARM_LOG_ERROR("FK failed before MoveLMulti");
    return 1;
  }

  // 以当前姿态为基准做一个矩形轨迹（姿态保持不变）
  std::vector<ArmMotion::Pose6Dof_t> cart_points;
  ArmMotion::Pose6Dof_t p1 = start_pose;
  p1.y -= 0.10;
  cart_points.push_back(p1);

  ArmMotion::Pose6Dof_t p2 = p1;
  p2.z -= 0.10;
  cart_points.push_back(p2);

  ArmMotion::Pose6Dof_t p3 = p2;
  p3.y += 0.10;
  cart_points.push_back(p3);

  ArmMotion::Pose6Dof_t p4 = p3;
  p4.z += 0.10;
  cart_points.push_back(p4);

  ARM_LOG_INFO("MoveLMulti demo: rectangle path, blend radius=0.01 m, v=0.1 m/s");
  if (robot.MoveLMulti(cart_points, 0.01, 0.1, 0.2, 0.0) != 0) {
    ARM_LOG_ERROR("MoveLMulti failed");
    return 1;
  }

  
  if (!PrintCurrentState(ctrl, "After MoveLMulti")) {
    return 1;
  }

  // 3) 回零
  ArmMotion::Joint6Dof_t joint_origin(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ARM_LOG_INFO("MoveJ back to joint origin");
  if (robot.MoveJ(joint_origin, 2.0, 3.0, 0.0) != 0) {
    ARM_LOG_ERROR("MoveJ back to joint origin failed");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  if (!PrintCurrentState(ctrl, "After MoveJ to origin")) {
    return 1;
  }

  ARM_LOG_INFO("MoveJMulti && MoveLMulti demo completed successfully");
  return 0;
}
