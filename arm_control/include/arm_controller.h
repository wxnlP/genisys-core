#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
#include "jacobian.h"
#include "kinematic.h"
#include "time_scaler_base.h"

class ArmController {
 public:
  enum class TrajectoryMode {
    LSPB,     // 梯形加减速
    CUBIC,    // 三次多项式
    QUINTIC,  // 五次多项式
  };

  ArmController(const ArmMotion::MDHParams& mdh_params,
                KinematicSolver::IKMode ik_mode = KinematicSolver::IKMode::SphericalWrist) {
    kinematic_solver_ = std::make_unique<KinematicSolver>(mdh_params);
    kinematic_solver_->SetIKMode(ik_mode);
    jacobian_ = std::make_unique<Jacobian6Dof>(mdh_params);
  }
  ~ArmController() = default;

  void SetMotorParams(ArmMotion::Joint6Dof stiffness, ArmMotion::Joint6Dof damping);
  void SetJointLimits(ArmMotion::Joint6Dof joint_min, ArmMotion::Joint6Dof joint_max);
  void SetTrajectoryMode(TrajectoryMode mode);

  int GetCurrentPose(ArmMotion::Pose6Dof& current_pose);
  int GetCurrentJoint(ArmMotion::Joint6Dof& current_joint);

  void SetToolPose(const ArmMotion::Pose6Dof& tool_pose);
  void ClearToolPose();

  // 底层驱动更新
  void SetJointState(const ArmMotion::JointState& joint_state);
  bool GetJointCommand(ArmMotion::JointCommand& joint_command);

  void WaitForCompletion();

  int MoveL(ArmMotion::Pose6Dof target_pose, double v, double a);
  int MoveJ(ArmMotion::Joint6Dof target_joint, double v, double a);
  int MoveLMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a);
  int MoveJMulti(const std::vector<ArmMotion::Joint6Dof>& points, double r, double v, double a);

 private:
  bool SelectBestIKSolve(const ArmMotion::IKSolves& ik_solves,
                         const ArmMotion::Joint6Dof& reference_joint,
                         ArmMotion::Joint6Dof& selected_joint) const;

  std::unique_ptr<TimeScalerBase> CreateTimeScaler(double normalized_v,
                                                   double normalized_a,
                                                   TimeScalerBase::PlanConfig& config);

 private:
  std::unique_ptr<KinematicSolver> kinematic_solver_;
  std::unique_ptr<Jacobian6Dof> jacobian_;

  ArmMotion::Joint6Dof motor_stiffness_{};
  ArmMotion::Joint6Dof motor_damping_{};
  ArmMotion::Joint6Dof joint_min_{};
  ArmMotion::Joint6Dof joint_max_{};
  ArmMotion::JointState current_state_{};

  TrajectoryMode traj_planning_mode{TrajectoryMode::QUINTIC};

  double sample_frequency_ = 100.0;                    // 采样频率（Hz）
  std::mutex state_mutex_;                             // 保护 current_state_
  std::mutex command_mutex_;                           // 保护 command_queue_
  std::queue<ArmMotion::JointCommand> command_queue_;  // 待执行的关节命令队列
  std::condition_variable command_cv_;
};
