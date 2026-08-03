#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <optional>
#include "cartesian_trajectory.h"
#include "jacobian.h"
#include "joint_trajectory.h"
#include "kinematic.h"
#include "dynamics.h"
#include "spsc_ring_buffer.h"
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
    mdh_params_ = mdh_params;
    kinematic_solver_ = std::make_unique<KinematicSolver>(mdh_params);
    kinematic_solver_->SetIKMode(ik_mode);
    jacobian_ = std::make_unique<Jacobian6Dof>(mdh_params);
  }
  ~ArmController() = default;

  void SetMotorParams(ArmMotion::Joint6Dof stiffness, ArmMotion::Joint6Dof damping);
  void SetJointLimits(ArmMotion::Joint6Dof joint_min, ArmMotion::Joint6Dof joint_max);
  void SetTrajectoryMode(TrajectoryMode mode);
  void StartSampleThread();
  void StopSampleThread();
  void SetSampleFrequency(double hz) { sample_frequency_ = hz; }
  double GetSampleFrequency() const { return sample_frequency_; }
  void SetDynamicsModel(const std::vector<Dynamics::Link>& links, const std::vector<double>& g);

  int GetCurrentPose(ArmMotion::Pose6Dof& current_pose);
  int GetCurrentJoint(ArmMotion::Joint6Dof& current_joint);
  int JointToPose(const ArmMotion::Joint6Dof& joint, ArmMotion::Pose6Dof& pose);

  void SetToolPose(const ArmMotion::Pose6Dof& tool_pose);
  void ClearToolPose();

  // 底层驱动更新
  void SetJointState(const ArmMotion::JointState& joint_state);
  bool GetJointCommand(ArmMotion::JointCommand& joint_command);

  int WaitForCompletion();
  int WaitForCompletion(ArmMotion::Joint6Dof& target, uint32_t timeout_ms);
  int WaitForCompletion(ArmMotion::Pose6Dof& target, uint32_t timeout_ms);

  int MoveL(ArmMotion::Pose6Dof target_pose, double v, double a);
  int MoveJ(ArmMotion::Joint6Dof target_joint, double v, double a);
  int MoveJ(ArmMotion::Pose6Dof target_pose, double v, double a);
  int MoveLMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a);
  int MoveJMulti(const std::vector<ArmMotion::Joint6Dof>& points, double r, double v, double a);
  int MoveJMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a);
  int ServoJ(const ArmMotion::Joint6Dof& target_joint, double dt);
  int ServoJ(const ArmMotion::Pose6Dof& target_pose, double dt);
  int ServoL(const ArmMotion::Pose6Dof& target_pose, double dt);
  void ServoStop();

  void SetRobotName(std::string name) { robot_name_ = name; }

 private:
  bool SelectBestIKSolve(const ArmMotion::IKSolves& ik_solves,
                         const ArmMotion::Joint6Dof& reference_joint,
                         ArmMotion::Joint6Dof& selected_joint) const;

  std::unique_ptr<TimeScalerBase> CreateTimeScaler(double normalized_v,
                                                   double normalized_a,
                                                   TimeScalerBase::PlanConfig& config);
  using SampleTask = std::function<bool(double t, ArmMotion::JointCommand& cmd)>;
  struct SampleContext {
    SampleTask task;
    double duration;
  };

 private:
  std::unique_ptr<KinematicSolver> kinematic_solver_;
  std::unique_ptr<Jacobian6Dof> jacobian_;
  std::unique_ptr<Dynamics> dynamics_;

  ArmMotion::MDHParams mdh_params_;

  ArmMotion::Joint6Dof motor_stiffness_{};
  ArmMotion::Joint6Dof motor_damping_{};
  ArmMotion::Joint6Dof joint_min_{};
  ArmMotion::Joint6Dof joint_max_{};
  ArmMotion::JointState current_state_{};

  TrajectoryMode traj_planning_mode{TrajectoryMode::QUINTIC};

  double sample_frequency_{};                                    // 采样频率（Hz）
  bool is_cartesian_trajectory_{false};                          // 当前轨迹类型标志
  std::mutex state_mutex_;                                       // 保护 current_state_
  std::mutex sample_mutex_;                                      // 保护 sample_queue_
  std::condition_variable sample_cv_;                            // 采样线程条件变量
  std::jthread sampling_thread_;                                 // 采样线程
  enum class MotionState : uint8_t { IDLE, ACTIVE };
  std::atomic<MotionState> motion_state_{MotionState::IDLE};     // 采样线程运动状态
  SPSCRingBuffer<ArmMotion::JointCommand> command_queue_{8192};  // 待执行的关节命令队列
  std::deque<SampleContext> sample_queue_;                       // 待采样的轨迹队列
  std::optional<ArmMotion::Joint6Dof> prev_servo_joint_;
  std::optional<ArmMotion::Pose6Dof> prev_servo_pose_;

  std::string robot_name_;
};
