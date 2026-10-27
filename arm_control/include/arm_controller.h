#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>
#include "cartesian_trajectory.h"
#include "dynamics.h"
#include "jacobian.h"
#include "joint_trajectory.h"
#include "kinematic.h"
#include "spsc_ring_buffer.h"
#include "time_scaler_base.h"
#include "tool_frame.h"

struct EndEffectorPoses {
  ArmMotion::Pose6Dof flange;
  ArmMotion::Pose6Dof tcp;
};

class ArmController {
 public:
  enum class TrajectoryMode {
    LSPB,     // 梯形加减速
    CUBIC,    // 三次多项式
    QUINTIC,  // 五次多项式
  };

  enum class TaskState {
    Running,
    SampleError,
    TolerError,
    Finished,
    Unknown = -1
  };

  using time_ns = std::chrono::sys_time<std::chrono::nanoseconds>;

  struct TaskContext {
    uint64_t id{};                        // 任务标识ID
    double duration_ms{};                 // 预计执行时间
    time_ns end_timestamp{};              // 完成时间（系统时间）
    TaskState state{TaskState::Unknown};  // 任务当前状态
    int code{404};                        // 错误码
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
  void SetTaskContextSize(int size);

  int GetTCPPose(ArmMotion::Pose6Dof& tcp);
  int GetFlangePose(ArmMotion::Pose6Dof& flange);
  int GetEndEffectorPoses(EndEffectorPoses& poses);
  int GetCurrentJoint(ArmMotion::Joint6Dof& current_joint);
  int JointToTCPPose(const ArmMotion::Joint6Dof& joint, ArmMotion::Pose6Dof& tcp);

  void SetTool(const ArmMotion::Pose6Dof& tool_pose);
  void ClearTool();

  int SolveIK(const ArmMotion::Pose6Dof& target, bool is_flange, bool do_fk, ArmMotion::Joint6Dof& joint);

  // 底层驱动更新
  void SetJointState(const ArmMotion::JointState& joint_state);
  bool GetJointCommand(ArmMotion::JointCommand& joint_command);

  int WaitForCompletion();
  bool JointTargetJudge(double tolerance);
  void MotionStateHandle(time_ns now);
  bool GetHistoryTaskContext(std::vector<TaskContext>& task_contexts);
  bool GetCurrentTaskContext(TaskContext& task_context);

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

 private:
  // 只接受关节限位内的解；调用方无需再单独过滤限位。
  bool SelectBestIKSolve(const ArmMotion::IKSolves& ik_solves, const ArmMotion::Joint6Dof& reference_joint,
                         ArmMotion::Joint6Dof& selected_joint, bool log) const;

  ArmMotion::Joint6Dof ResolveStartJoint(const ArmMotion::Joint6Dof& measured) const;

  std::unique_ptr<TimeScalerBase> CreateTimeScaler(double normalized_v,
                                                   double normalized_a,
                                                   TimeScalerBase::PlanConfig& config);
  using SampleTask = std::function<bool(double t, ArmMotion::JointCommand& cmd)>;
  struct SampleContext {
    SampleTask task;
    double duration;
    bool is_servo{false};
  };

  int BeginMoveTask(SampleTask task, double duration_s);
  int BeginServoTask(SampleTask task, double duration_s);

 private:
  enum class MotionState {
    Idle,
    Running,
    Sampled,
    Error,
    Servo
  };

  std::unique_ptr<KinematicSolver> kinematic_solver_;
  std::unique_ptr<Jacobian6Dof> jacobian_;
  std::unique_ptr<Dynamics> dynamics_;
  ToolFrame tool_frame_;

  ArmMotion::MDHParams mdh_params_;

  ArmMotion::Joint6Dof motor_stiffness_{};
  ArmMotion::Joint6Dof motor_damping_{};
  ArmMotion::Joint6Dof joint_min_{};
  ArmMotion::Joint6Dof joint_max_{};

  TrajectoryMode traj_planning_mode{TrajectoryMode::QUINTIC};

  double sample_frequency_{};            // 采样频率（Hz）
  bool is_cartesian_trajectory_{false};  // 当前轨迹类型标志
  std::mutex state_mutex_;               // 保护 current_state_
  std::mutex sample_mutex_;              // 保护 sample_queue_
  std::condition_variable sample_cv_;    // 采样线程条件变量
  std::jthread sampling_thread_;         // 采样线程

  std::atomic<MotionState> motion_state_{MotionState::Idle};  // 当前运动状态
  uint64_t task_id_{0};
  std::vector<TaskContext> task_contexts_;
  std::mutex task_context_mutex_;
  TaskContext current_task_context_{};

  SPSCRingBuffer<ArmMotion::JointCommand> command_queue_{8192};  // 待执行的关节命令队列
  std::deque<SampleContext> sample_queue_;                       // 待采样的轨迹队列

  std::optional<ArmMotion::JointState> current_state_;
  std::optional<ArmMotion::Joint6Dof> prev_servo_joint_;
  std::optional<ArmMotion::Pose6Dof> prev_servo_tcp_;
  std::optional<ArmMotion::Joint6Dof> last_cmd_joint_;
  mutable std::mutex last_cmd_mutex_;
};
