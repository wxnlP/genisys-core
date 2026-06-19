#pragma once

#include <memory>
#include <vector>

#include "arm_ctrl_base.h"
#include "cartesian_linear_path.h"
#include "cartesian_multi_point_path.h"
#include "cartesian_trajectory.h"
#include "cubic_time_scaler.h"
#include "jacobian.h"
#include "joint_multi_point_path.h"
#include "joint_path.h"
#include "joint_trajectory.h"
#include "kinematic.h"
#include "lspb_time_scaler.h"
#include "quintic_time_scaler.h"

class ArmInstance {
 public:
  enum class TrajectoryMode {
    LSPB,     // 梯形加减速
    CUBIC,    // 三次多项式
    QUINTIC,  // 五次多项式
  };
  // 切换真机与仿真时，构造函数传入的arm_ctrl对象不同
  ArmInstance(ArmCtrlBase& arm_ctrl, const ArmMotion::MDHParams& mdh_params, KinematicSolver::IKMode ik_mode = KinematicSolver::IKMode::SphericalWrist)
      : arm_ctrl_(arm_ctrl) {
    kinematic_solver_ = std::make_unique<KinematicSolver>(mdh_params);
    kinematic_solver_->SetIKMode(ik_mode);
    jacobian_ = std::make_unique<Jacobian6Dof>(mdh_params);
  }
  ~ArmInstance() = default;

  void SetMotorParams(ArmMotion::Joint6Dof_t stiffness, ArmMotion::Joint6Dof_t damping);
  void SetJointLimits(ArmMotion::Joint6Dof_t joint_min, ArmMotion::Joint6Dof_t joint_max);
  void SetTrajectoryMode(TrajectoryMode mode);

  void Init();
  void Enable();
  void Disable();

  // 单段笛卡尔直线运动（MoveL）
  // 参数单位：
  //   target_pose : 目标位姿（位置单位 m）
  //   v           : TCP 线速度上限（m/s）
  //   a           : TCP 线加速度上限（m/s^2）
  //   t           : 期望总时长（s），<=0 表示由 v/a 自动规划
  int MoveL(ArmMotion::Pose6Dof_t target_pose, double v, double a, double t);

  // 单段关节运动（MoveJ）
  // 参数单位：
  //   target_joint: 目标关节角（rad）
  //   v           : 关节空间路径速度上限（rad/s）
  //   a           : 关节空间路径加速度上限（rad/s^2）
  //   t           : 期望总时长（s），<=0 表示由 v/a 自动规划
  int MoveJ(ArmMotion::Joint6Dof_t target_joint, double v, double a, double t);

  // 多点笛卡尔运动（MoveLMulti，圆弧融合）
  // 参数单位：
  //   points : 目标路径点序列（位置单位 m）
  //   r      : 融合半径（m）
  //   v      : TCP 线速度上限（m/s）
  //   a      : TCP 线加速度上限（m/s^2）
  //   t      : 期望总时长（s），<=0 表示由 v/a 自动规划
  int MoveLMulti(const std::vector<ArmMotion::Pose6Dof_t>& points, double r,
                 double v, double a, double t);

  // 多点关节运动（MoveJMulti，抛物线融合）
  // 参数单位：
  //   points : 目标关节路径点序列（rad）
  //   r      : 融合半径（rad）
  //   v      : 关节空间路径速度上限（rad/s）
  //   a      : 关节空间路径加速度上限（rad/s^2）
  //   t      : 期望总时长（s），<=0 表示由 v/a 自动规划
  int MoveJMulti(const std::vector<ArmMotion::Joint6Dof_t>& points, double r,
                 double v, double a, double t);
  int GetCurrentPose(ArmMotion::Pose6Dof_t& current_pose);
  int GetCurrentJoint(ArmMotion::Joint6Dof_t& current_joint);

  void SetToolPose(const ArmMotion::Pose6Dof_t& tool_pose);
  void ClearToolPose();

 private:
  bool SelectBestIKSolve(const ArmMotion::IKSolves_t& ik_solves,
                         const ArmMotion::Joint6Dof_t& reference_joint,
                         ArmMotion::Joint6Dof_t& selected_joint) const;

  // 类型桥接：ArmMotion::* ↔ ArmCtrlBase::*
  bool GetActuatorJointState(ArmMotion::JointState_t& state);
  bool SetActuatorJointState(const ArmMotion::JointCommand_t& cmd);

  std::unique_ptr<KinematicSolver> kinematic_solver_;
  std::unique_ptr<Jacobian6Dof> jacobian_;
  std::unique_ptr<JointTrajectory> joint_trajectory_;
  std::unique_ptr<CartesianTrajectory> cartesian_trajectory_;
  std::unique_ptr<TimeScalerBase> time_scaler_;

  ArmCtrlBase& arm_ctrl_;
  const double dt_ = 0.01;
  ArmMotion::Joint6Dof_t motor_stiffness_{};
  ArmMotion::Joint6Dof_t motor_damping_{};

  // 关节限位（弧度）
  ArmMotion::Joint6Dof_t joint_min_{};
  ArmMotion::Joint6Dof_t joint_max_{};

  // 轨迹规划模式
  TrajectoryMode traj_planning_mode{TrajectoryMode::QUINTIC};
};
