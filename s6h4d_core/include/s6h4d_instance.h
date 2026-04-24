#pragma once

#include <memory>

#include "arm_ctrl_base.h"
#include "cartesian_linear_path.h"
#include "cartesian_trajectory.h"
#include "cubic_time_scaler.h"
#include "jacobian.h"
#include "joint_path.h"
#include "joint_trajectory.h"
#include "kinematic.h"
#include "lspb_time_scaler.h"

class S6h4dInstance {
 public:
  // 切换真机与仿真时，构造函数传入的arm_ctrl对象不同
  S6h4dInstance(ArmCtrlBase& arm_ctrl, ArmMotion::MDHConfig_t& mdh_config)
      : arm_ctrl_(arm_ctrl) {
    kinematic_solver_ = std::make_unique<KinematicSolver>(mdh_config);
    jacobian_ = std::make_unique<Jacobian6Dof>(mdh_config);
    arm_ctrl_.Init();
  }
  ~S6h4dInstance() = default;

  void Enable();
  void Disable();

  int MoveL(ArmMotion::Pose6Dof_t target_pose, double v, double a, double t);
  int MoveJ(ArmMotion::Joint6Dof_t target_joint, double v, double a, double t);

 private:
  bool SelectBestIKSolve(const ArmMotion::IKSolves_t& ik_solves,
                         const ArmMotion::Joint6Dof_t& reference_joint,
                         ArmMotion::Joint6Dof_t& selected_joint) const;

  std::unique_ptr<KinematicSolver> kinematic_solver_;
  std::unique_ptr<Jacobian6Dof> jacobian_;
  std::unique_ptr<JointTrajectory> joint_trajectory_;
  std::unique_ptr<CartesianTrajectory> cartesian_trajectory_;

  ArmCtrlBase& arm_ctrl_;
  const double dt_ = 0.01;
};
