#include "arm_controller.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>
#include "arm_logger.h"
#include "cartesian_linear_path.h"
#include "cartesian_multi_point_path.h"
#include "cubic_time_scaler.h"
#include "joint_multi_point_path.h"
#include "joint_path.h"
#include "lspb_time_scaler.h"
#include "quintic_time_scaler.h"
#include "time_scaler_base.h"

namespace {

// 限位检查浮点容差 (rad)
constexpr double kLimitEps = 0.01;
// 到位判定容差：实际关节角与最后一条下发指令的最大偏差 (rad)
constexpr double kJointArriveTolRad = 0.02;
constexpr double kBumplessTolRad = 0.05;

constexpr size_t kMaxTaskContexts = 10;

// TaskContext 里的错误码
constexpr int kCodeRequestError = -1;  // 请求错误
constexpr int kCodeSampleError = -2;   // 采样错误
constexpr int kCodeTolerError = -3;    // 容差错误

double JointPathLength(const ArmMotion::Joint6Dof& q0, const ArmMotion::Joint6Dof& q1) {
  double sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double d = q1.q[i] - q0.q[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

double CartesianPosDistance(const ArmMotion::Pose6Dof& p0, const ArmMotion::Pose6Dof& p1) {
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double dz = p1.z - p0.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

std::unique_ptr<TimeScalerBase> ArmController::CreateTimeScaler(
    double normalized_v, double normalized_a,
    TimeScalerBase::PlanConfig& config) {
  switch (traj_planning_mode) {
    case TrajectoryMode::CUBIC: {
      double t_v = 1.5 / normalized_v;
      double t_a = std::sqrt(6.0 / normalized_a);
      config.T = std::max(t_v, t_a);
      return std::make_unique<CubicTimeScaler>();
    }
    case TrajectoryMode::QUINTIC: {
      double t_v = 1.875 / normalized_v;
      double t_a = std::sqrt(5.7735 / normalized_a);
      config.T = std::max(t_v, t_a);
      return std::make_unique<QuinticTimeScaler>();
    }
    case TrajectoryMode::LSPB: {
      config.a_max = normalized_a;
      config.v_max = normalized_v;
      return std::make_unique<LSPBTimeScaler>();
    }
    default:
      return nullptr;
  }
}

bool ArmController::SelectBestIKSolve(
    const ArmMotion::IKSolves& ik_solves,
    const ArmMotion::Joint6Dof& reference_joint,
    ArmMotion::Joint6Dof& selected_joint) const {
  constexpr double kPi = M_PI;
  constexpr double kTwoPi = 2.0 * kPi;

  double best_score = std::numeric_limits<double>::infinity();
  bool found_valid_solve = false;

  // 记录最接近限位、但因限位被丢弃的关节
  int bad_branch = -1;
  int bad_joint = -1;
  double bad_value = 0.0;
  double bad_lo = 0.0;
  double bad_hi = 0.0;
  double bad_gap = std::numeric_limits<double>::infinity();

  for (int i = 0; i < 8; ++i) {
    const char j1_flag = ik_solves.solveFlag[i][0];
    const char j23_flag = ik_solves.solveFlag[i][1];
    const char wrist_flag = ik_solves.solveFlag[i][2];

    // 0 表示不可达；-1 表示奇异位形仍可用但可作为次优。
    if (j1_flag == 0 || j23_flag == 0 || wrist_flag == 0) {
      continue;
    }

    // 把每个关节格式化到限位内的 2π 表示；多个表示时取离参考最近的。
    ArmMotion::Joint6Dof candidate;
    bool within_limits = true;
    for (int j = 0; j < 6; ++j) {
      const double raw = ik_solves.solves[i].q[j];
      const double lo = joint_min_.q[j] - kLimitEps;
      const double hi = joint_max_.q[j] + kLimitEps;
      const int k_min = static_cast<int>(std::ceil((lo - raw) / kTwoPi));
      const int k_max = static_cast<int>(std::floor((hi - raw) / kTwoPi));

      bool found = false;
      double best = 0.0;
      double best_dist = std::numeric_limits<double>::infinity();
      for (int k = k_min; k <= k_max; ++k) {
        const double q = raw + static_cast<double>(k) * kTwoPi;
        const double dist = std::abs(q - reference_joint[j]);
        if (dist < best_dist) {
          best_dist = dist;
          best = q;
          found = true;
        }
      }
      if (!found) {
        // 该关节没有任何 2π 表示落在限位内；记录离限位最近的表示。
        const double q_above = raw + static_cast<double>(k_min) * kTwoPi;
        const double q_below = raw + static_cast<double>(k_max) * kTwoPi;
        const double gap_above = q_above - joint_max_.q[j];
        const double gap_below = joint_min_.q[j] - q_below;
        const bool use_above = gap_above <= gap_below;
        const double gap = use_above ? gap_above : gap_below;
        if (gap < bad_gap) {
          bad_gap = gap;
          bad_branch = i;
          bad_joint = j;
          bad_value = use_above ? q_above : q_below;
          bad_lo = joint_min_.q[j];
          bad_hi = joint_max_.q[j];
        }
        within_limits = false;
        break;
      }
      candidate[j] = best;
    }

    // 关节限位是硬约束：没有任何 2π 表示落在限位内的分支直接丢弃。
    if (!within_limits) {
      continue;
    }

    double score = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double diff = candidate.q[j] - reference_joint.q[j];
      score += diff * diff;

      // 对接近限位的解增加惩罚，优先选择远离限位的解
      double val = candidate.q[j];
      double margin_lower = val - joint_min_.q[j];
      double margin_upper = joint_max_.q[j] - val;
      double min_margin = std::min(margin_lower, margin_upper);
      if (min_margin < 0.2) {                // 距离限位小于 0.2 rad (~11°)
        score += 10.0 * (0.2 - min_margin);  // 越接近限位惩罚越大
      }
    }

    // 对奇异位形解加入轻微惩罚，优先选择常规可逆解。
    if (j1_flag < 0) {
      score += 1e-4;
    }
    if (j23_flag < 0) {
      score += 1e-4;
    }
    if (wrist_flag < 0) {
      score += 1e-4;
    }

    if (score < best_score) {
      best_score = score;
      selected_joint = candidate;
      found_valid_solve = true;
    }
  }

  if (!found_valid_solve && bad_joint >= 0) {
    ARM_LOG_WARN(
        "[IK] no branch fits joint limits: branch {} joint {} value={:.4f} rad, "
        "limit=[{:.4f}, {:.4f}] rad, gap={:.4f} rad",
        bad_branch, bad_joint, bad_value, bad_lo, bad_hi, bad_gap);
  }

  return found_valid_solve;
}

void ArmController::SetMotorParams(ArmMotion::Joint6Dof stiffness, ArmMotion::Joint6Dof damping) {
  motor_stiffness_ = stiffness;
  motor_damping_ = damping;
}

void ArmController::SetJointLimits(ArmMotion::Joint6Dof joint_min, ArmMotion::Joint6Dof joint_max) {
  joint_min_ = joint_min;
  joint_max_ = joint_max;
}

void ArmController::SetTrajectoryMode(TrajectoryMode mode) {
  traj_planning_mode = mode;
}

void ArmController::SetDynamicsModel(const std::vector<Dynamics::Link>& links, const std::vector<double>& g) {
  dynamics_ = std::make_unique<Dynamics>(mdh_params_, links, g);
}

void ArmController::StopSampleThread() {
  if (sampling_thread_.joinable()) {
    sampling_thread_.request_stop();
    sample_cv_.notify_one();
    sampling_thread_.join();
    motion_state_.store(MotionState::Idle);
    command_queue_.Reset();
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      sample_queue_.clear();
    }
  }
}

void ArmController::StartSampleThread() {
  sampling_thread_ = std::jthread([this](std::stop_token stoken) {
    constexpr size_t kLowWaterMark = 2048;   // 队列低水位阈值 (~25%)
    constexpr size_t kHighWaterMark = 6144;  // 队列高水位阈值 (~75%)
    enum class Phase { BOOTSTRAP,
                       STEADY,
                       BACKOFF };
    ArmMotion::JointCommand cmd;
    while (!stoken.stop_requested()) {
      SampleContext context;
      {
        std::unique_lock<std::mutex> lock(sample_mutex_);
        if (sample_queue_.empty()) {
          // 无轨迹：等待新任务
          sample_cv_.wait(lock, [this, &stoken]() {
            return !sample_queue_.empty() || stoken.stop_requested();
          });
          if (stoken.stop_requested()) {
            break;
          }
        }
        context = std::move(sample_queue_.front());
        sample_queue_.pop_front();
      }

      Phase phase = Phase::BOOTSTRAP;
      double t = 0.0;
      double dt = 1.0 / sample_frequency_;
      auto t_last_push = std::chrono::steady_clock::now();

      bool ok = true;
      while (t <= context.duration + 1e-7 && !stoken.stop_requested()) {
        // 跳过 t=0 采样点
        if (t > 1e-9) {
          // 采样
          if (!context.task(t, cmd)) {
            ok = false;
            break;
          }

          // 入队（满则重试，永不丢点）
          while (!command_queue_.Push(cmd) && !stoken.stop_requested()) {
            phase = Phase::BACKOFF;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
        }
        t += dt;

        // 获取轨迹点队列长度
        size_t size = command_queue_.Size();

        switch (phase) {
          case Phase::BOOTSTRAP: {
            if (size >= kLowWaterMark) {
              phase = Phase::STEADY;
              t_last_push = std::chrono::steady_clock::now();
            }
            break;
          }
          case Phase::STEADY: {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(now - t_last_push);
            auto interval_ms = static_cast<uint64_t>(dt * 1'000'000);
            if (elapsed_ms.count() < interval_ms) {
              std::this_thread::sleep_for(std::chrono::microseconds(interval_ms - elapsed_ms.count()));
            }
            t_last_push = std::chrono::steady_clock::now();
            if (size >= kHighWaterMark) {
              phase = Phase::BACKOFF;
            }
            break;
          }
          case Phase::BACKOFF: {
            if (size <= kLowWaterMark) {
              phase = Phase::STEADY;
              t_last_push = std::chrono::steady_clock::now();
            } else {
              // 睡 5ms > 控制循环 2ms 周期，确保消费比生产快
              std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            break;
          }
        }
      }
      if (!context.is_servo) {
        motion_state_.store(ok ? MotionState::Sampled : MotionState::Error);
      } else if (!ok) {
        ARM_LOG_WARN("[Servo] Sampling failed");
      }
    }
  });
}

int ArmController::GetTCPPose(ArmMotion::Pose6Dof& tcp) {
  ArmMotion::Pose6Dof flange;
  const int ret = GetFlangePose(flange);
  if (ret != 0) {
    return ret;
  }
  tcp = tool_frame_.Compose(flange);
  return 0;
}

int ArmController::GetFlangePose(ArmMotion::Pose6Dof& flange) {
  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    return -1;
  }
  if (!kinematic_solver_->FK(current_joint, flange)) {
    return -1;
  }
  return 0;
}

int ArmController::GetEndEffectorPoses(EndEffectorPoses& poses) {
  const int ret = GetFlangePose(poses.flange);
  if (ret != 0) {
    return ret;
  }
  poses.tcp = tool_frame_.Compose(poses.flange);
  return 0;
}

int ArmController::GetCurrentJoint(ArmMotion::Joint6Dof& current_joint) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_state_.has_value()) {
    return -1;
  }
  current_joint = current_state_.value().position;
  return 0;
}

int ArmController::JointToTCPPose(const ArmMotion::Joint6Dof& joint, ArmMotion::Pose6Dof& tcp) {
  ArmMotion::Pose6Dof flange;
  if (!kinematic_solver_->FK(joint, flange)) {
    return -1;
  }
  tcp = tool_frame_.Compose(flange);
  return 0;
}

void ArmController::SetTool(const ArmMotion::Pose6Dof& tool_pose) {
  tool_frame_.Set(tool_pose);
}

void ArmController::ClearTool() {
  tool_frame_.Clear();
}

void ArmController::SetJointState(const ArmMotion::JointState& joint_state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_state_ = joint_state;
}

bool ArmController::GetJointCommand(ArmMotion::JointCommand& joint_command) {
  if (command_queue_.Pop(joint_command)) {
    {
      std::lock_guard<std::mutex> lock(last_cmd_mutex_);
      last_cmd_joint_ = joint_command.position;
    }
    return true;
  }
  joint_command.damping = motor_damping_;
  joint_command.stiffness = motor_stiffness_;
  return false;
}

ArmMotion::Joint6Dof ArmController::ResolveStartJoint(const ArmMotion::Joint6Dof& measured) const {
  std::optional<ArmMotion::Joint6Dof> last;
  {
    std::lock_guard<std::mutex> lock(last_cmd_mutex_);
    last = last_cmd_joint_;
  }
  if (!last.has_value()) {
    return measured;
  }

  double max_delta = 0.0;
  int max_delta_idx = 0;
  for (int i = 0; i < 6; ++i) {
    if (last->q[i] < joint_min_.q[i] - kLimitEps ||
        last->q[i] > joint_max_.q[i] + kLimitEps) {
      ARM_LOG_WARN("[bumpless] last command joint[{}]={:.4f} out of limit, fallback to measured",
                   i, last->q[i]);
      return measured;
    }
    const double d = std::abs(last->q[i] - measured.q[i]);
    if (d > max_delta) {
      max_delta = d;
      max_delta_idx = i;
    }
  }

  // 设定点与实测值差得太多
  if (max_delta > kBumplessTolRad) {
    ARM_LOG_WARN("[bumpless] fallback to measured: joint{} delta={:.4f} rad > {:.3f} rad",
                 max_delta_idx + 1, max_delta, kBumplessTolRad);
    return measured;
  }

  ARM_LOG_DEBUG("[bumpless] start from last command, max delta={:.4f} rad (joint{})",
                max_delta, max_delta_idx + 1);
  return last.value();
}

int ArmController::WaitForCompletion() {
  while (motion_state_.load() != MotionState::Idle || !command_queue_.IsEmpty()) {
    std::this_thread::yield();
  }
  return 0;
}

bool ArmController::JointTargetJudge(double tolerance) {
  ArmMotion::Joint6Dof actual, last;
  if (GetCurrentJoint(actual) != 0) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(last_cmd_mutex_);
    if (!last_cmd_joint_.has_value()) {
      return false;
    }
    last = last_cmd_joint_.value();
  }

  for (int i = 0; i < 6; ++i) {
    if (std::abs(actual[i] - last[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

void ArmController::SetTaskContextSize(int size) {
  const int clamped = std::min(std::max(size, 1), static_cast<int>(kMaxTaskContexts));
  if (clamped != size) {
    ARM_LOG_WARN("[TaskContext] size {} out of range, clamped to {}", size, clamped);
  }
  std::lock_guard<std::mutex> lock(task_context_mutex_);
  task_contexts_.assign(static_cast<size_t>(clamped), TaskContext{});
}

int ArmController::BeginMoveTask(SampleTask task, double duration_s) {
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (motion_state_.load() != MotionState::Idle) {
      return -1;
    }
    sample_queue_.emplace_back(std::move(task), duration_s);
    {
      std::lock_guard<std::mutex> ctx_lock(task_context_mutex_);
      current_task_context_ = TaskContext{++task_id_, duration_s * 1000.0, {}, TaskState::Running, 0};
    }
    motion_state_.store(MotionState::Running);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::BeginServoTask(SampleTask task, double duration_s) {
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    const MotionState s = motion_state_.load();
    if (s != MotionState::Idle && s != MotionState::Servo) {
      return -1;
    }
    sample_queue_.emplace_back(std::move(task), duration_s, true);
    motion_state_.store(MotionState::Servo);
  }
  sample_cv_.notify_one();
  return 0;
}

void ArmController::MotionStateHandle(time_ns now) {
  const MotionState now_state = motion_state_.load();
  if (now_state != MotionState::Sampled && now_state != MotionState::Error) {
    return;
  }
  if (!command_queue_.IsEmpty()) {
    return;
  }

  TaskContext ctx;
  {
    std::lock_guard<std::mutex> lock(task_context_mutex_);
    ctx = current_task_context_;
  }

  ctx.end_timestamp = now;
  if (now_state == MotionState::Error) {
    ctx.state = TaskState::SampleError;
    ctx.code = kCodeSampleError;
  } else if (JointTargetJudge(kJointArriveTolRad)) {
    ctx.state = TaskState::Finished;
    ctx.code = 0;
  } else {
    ctx.state = TaskState::TolerError;
    ctx.code = kCodeTolerError;
  }

  {
    std::lock_guard<std::mutex> lock(task_context_mutex_);
    task_contexts_[(ctx.id - 1) % task_contexts_.size()] = ctx;  // 环形覆盖最旧的一条
    current_task_context_ = TaskContext{};
  }

  motion_state_.store(MotionState::Idle);
}

bool ArmController::GetHistoryTaskContext(std::vector<TaskContext>& task_contexts) {
  std::lock_guard<std::mutex> lock(task_context_mutex_);
  task_contexts.clear();
  task_contexts.reserve(task_contexts_.size());
  for (const auto& ctx : task_contexts_) {
    if (ctx.id != 0) {
      task_contexts.push_back(ctx);
    }
  }
  return !task_contexts.empty();
}

bool ArmController::GetCurrentTaskContext(TaskContext& task_context) {
  std::lock_guard<std::mutex> lock(task_context_mutex_);
  task_context = current_task_context_;
  return current_task_context_.id != 0;
}

int ArmController::MoveL(ArmMotion::Pose6Dof target_pose, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveL] failed: motion state is not idle");
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveL] failed: current joint state is not available");
    return -1;
  }

  current_joint = ResolveStartJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  // 通过正运动学获取当前笛卡尔位置
  ArmMotion::Pose6Dof flange;
  if (!kinematic_solver_->FK(current_joint, flange)) {
    ARM_LOG_ERROR("[MoveL] failed: FK at start");
    return -1;
  }
  const ArmMotion::Pose6Dof current_tcp = tool_frame_.Compose(flange);

  const double path_length = CartesianPosDistance(current_tcp, target_pose);
  if (path_length <= 1e-7) {
    ARM_LOG_WARN("[MoveL] skip: path length too small");
    return -1;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a / path_length;

  TimeScalerBase::PlanConfig config;
  auto time_scaler = CreateTimeScaler(normalized_v, normalized_a, config);
  if (!time_scaler) {
    ARM_LOG_ERROR("[MoveL] failed: time scaler setting");
    return -1;
  }
  if (!time_scaler->Plan(config)) {
    ARM_LOG_ERROR("[MoveL] failed: time scaler plan");
    return -1;
  }

  // 创建笛卡尔路径（直线路径）
  auto path = std::make_shared<CartesianLinearPath>(current_tcp, target_pose);
  // 创建笛卡尔轨迹
  auto trajectory = std::make_shared<CartesianTrajectory>(path, std::move(time_scaler));
  double duration = trajectory->GetDuration();

  const ToolFrame tool = tool_frame_;
  bool first_sample = true;
  SampleTask task = [trajectory, current_joint, this, tool, first_sample](double t, ArmMotion::JointCommand& cmd) mutable {
    ArmMotion::CartesianTrajectoryPoint cart_point;
    if (!trajectory->Sample(t, cart_point)) {
      ARM_LOG_ERROR("[MoveL] failed: Sample at t={}", t);
      return false;
    }

    // 通过逆运动学求解目标关节角
    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(tool.Remove(cart_point.pose), current_joint, ik_solves)) {
      ARM_LOG_ERROR("[MoveL] failed: IK at t={}, pose=({}, {}, {})",
                    t, cart_point.pose.x, cart_point.pose.y, cart_point.pose.z);
      return false;
    }

    // 选择最优逆解：过滤不可达解并优先选与当前姿态最接近的解。
    ArmMotion::Joint6Dof target_joint;
    if (!SelectBestIKSolve(ik_solves, current_joint, target_joint)) {
      ARM_LOG_ERROR("[MoveL] failed: no valid IK solve at t={}", t);
      return false;
    }

    // 检查 IK 结果是否在关节限位内
    for (int i = 0; i < 6; ++i) {
      if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
        ARM_LOG_ERROR("[MoveL] trajectory point joint[{}]={:.4f} out of limit at t={}",
                      i, target_joint[i], t);
        return false;
      }
    }

    // 通过雅可比矩阵将笛卡尔速度/加速度转换为关节速度/加速度
    jacobian_->Update(target_joint, first_sample ? 0.0 : 1.0 / sample_frequency_,
                      tool.LocalPoint());
    first_sample = false;

    ArmMotion::Joint6Dof joint_vel;
    if (!jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6)) {
      return false;
    }

    ArmMotion::Joint6Dof joint_acc;
    if (!jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6)) {
      return false;
    }

    cmd.position = target_joint;
    cmd.velocity = joint_vel;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    // s6h4d特殊处理
    if (robot_name_ == "s6h4d") {
      cmd.effort = joint_acc;
    } else if (dynamics_) {
      // 重力补偿
      dynamics_->GravityVector(target_joint, cmd.effort);
    } else {
      cmd.effort = ArmMotion::Joint6Dof{};
    }
    // 本地更新参考关节角
    current_joint = target_joint;
    return true;
  };

  return BeginMoveTask(std::move(task), duration);
}

int ArmController::MoveJ(ArmMotion::Joint6Dof target_joint, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveJ] failed: motion state is not idle");
    return -1;
  }

  for (int i = 0; i < 6; i++) {
    if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
      ARM_LOG_ERROR("[MoveJ] target joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
                    i, target_joint[i], joint_min_[i], joint_max_[i]);
      return -1;
    }
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveJ] failed: current joint state is not available");
    return -1;
  }

  current_joint = ResolveStartJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  const double path_length = JointPathLength(current_joint, target_joint);
  if (path_length <= 1e-7) {
    ARM_LOG_WARN("[MoveJ] skip: path length too small");
    return 0;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a / path_length;

  TimeScalerBase::PlanConfig config;
  auto time_scaler = CreateTimeScaler(normalized_v, normalized_a, config);
  if (!time_scaler) {
    ARM_LOG_ERROR("[MoveJ] failed: time scaler setting");
    return -1;
  }
  if (!time_scaler->Plan(config)) {
    ARM_LOG_ERROR("[MoveJ] failed: time scaler plan");
    return -1;
  }

  auto path = std::make_shared<JointPath>(current_joint, target_joint);
  auto trajectory = std::make_shared<JointTrajectory>(path, std::move(time_scaler));
  double duration = trajectory->GetDuration();

  SampleTask task = [trajectory, this](double t, ArmMotion::JointCommand& cmd) {
    ArmMotion::JointTrajectoryPoint point;
    if (!trajectory->Sample(t, point)) {
      ARM_LOG_ERROR("[MoveJ] failed: Sample at t={}", t);
      return false;
    }

    for (int i = 0; i < 6; ++i) {
      if (point.position.q[i] < joint_min_.q[i] - kLimitEps || point.position.q[i] > joint_max_.q[i] + kLimitEps) {
        ARM_LOG_ERROR("[MoveJ] trajectory point joint[{}]={:.4f} out of limit at t={}",
                      i, point.position.q[i], t);
        return false;
      }
    }

    cmd.position = point.position;
    cmd.velocity = point.velocity;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    if (robot_name_ == "s6h4d") {
      cmd.effort = point.acceleration;
    } else if (dynamics_) {
      // 重力补偿
      dynamics_->GravityVector(point.position, cmd.effort);
    } else {
      cmd.effort = ArmMotion::Joint6Dof{};
    }
    return true;
  };

  return BeginMoveTask(std::move(task), duration);
}

int ArmController::MoveJ(ArmMotion::Pose6Dof target_pose, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveJ] failed: motion state is not idle");
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveJ] failed: current joint state is not available");
    return -1;
  }

  // 逆解得到目标关节角，优先选择接近当前关节角的解
  ArmMotion::IKSolves ik_solves;
  if (!kinematic_solver_->IK(tool_frame_.Remove(target_pose), current_joint, ik_solves)) {
    ARM_LOG_ERROR("[MoveJ] failed: IK at target pose=({}, {}, {})",
                  target_pose.x, target_pose.y, target_pose.z);
    return -1;
  }
  ArmMotion::Joint6Dof target_joint;
  if (!SelectBestIKSolve(ik_solves, current_joint, target_joint)) {
    ARM_LOG_ERROR("[MoveJ] failed: no valid IK solve within joint limits at target pose=({}, {}, {})",
                  target_pose.x, target_pose.y, target_pose.z);
    return -1;
  }

  return MoveJ(target_joint, v, a);
}

int ArmController::MoveLMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveLMulti] failed: motion state is not idle");
    return -1;
  }

  if (points.size() < 1) {
    ARM_LOG_ERROR("[MoveLMulti] failed: points size {} < 1", points.size());
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveLMulti] failed: current joint state is not available");
    return -1;
  }

  current_joint = ResolveStartJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  ArmMotion::Pose6Dof flange;
  if (!kinematic_solver_->FK(current_joint, flange)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: FK at start");
    return -1;
  }
  const ArmMotion::Pose6Dof current_tcp = tool_frame_.Compose(flange);

  std::vector<ArmMotion::Pose6Dof> path_points;
  path_points.reserve(points.size() + 1);
  path_points.push_back(current_tcp);
  path_points.insert(path_points.end(), points.begin(), points.end());

  auto path = std::make_shared<CartesianMultiPointPath>(std::move(path_points), r);
  const double path_length = path->GetTotalLength();
  if (path_length <= 1e-7) {
    ARM_LOG_ERROR("[MoveLMulti] failed: invalid path total length");
    return -1;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a / path_length;

  TimeScalerBase::PlanConfig config;
  auto time_scaler = CreateTimeScaler(normalized_v, normalized_a, config);
  if (!time_scaler) {
    ARM_LOG_ERROR("[MoveLMulti] failed: time scaler setting");
    return -1;
  }
  if (!time_scaler->Plan(config)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: time scaler plan");
    return -1;
  }

  auto trajectory = std::make_shared<CartesianTrajectory>(path, std::move(time_scaler));
  double duration = trajectory->GetDuration();

  const ToolFrame tool = tool_frame_;
  bool first_sample = true;
  SampleTask task = [trajectory, current_joint, this, tool, first_sample](double t, ArmMotion::JointCommand& cmd) mutable {
    ArmMotion::CartesianTrajectoryPoint cart_point;
    if (!trajectory->Sample(t, cart_point)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: Sample at t={}", t);
      return false;
    }

    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(tool.Remove(cart_point.pose), current_joint, ik_solves)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: IK at t={}, pose=({}, {}, {})",
                    t, cart_point.pose.x, cart_point.pose.y, cart_point.pose.z);
      return false;
    }

    ArmMotion::Joint6Dof target_joint;
    if (!SelectBestIKSolve(ik_solves, current_joint, target_joint)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: no valid IK solve at t={}", t);
      return false;
    }

    for (int i = 0; i < 6; ++i) {
      if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
        ARM_LOG_ERROR("[MoveLMulti] trajectory point joint[{}]={:.4f} out of limit at t={}",
                      i, target_joint[i], t);
        return false;
      }
    }

    jacobian_->Update(target_joint, first_sample ? 0.0 : 1.0 / sample_frequency_,
                      tool.LocalPoint());
    first_sample = false;
    ArmMotion::Joint6Dof joint_vel;
    if (!jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6)) {
      return false;
    }
    ArmMotion::Joint6Dof joint_acc;
    if (!jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6)) {
      return false;
    }

    cmd.position = target_joint;
    cmd.velocity = joint_vel;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    if (robot_name_ == "s6h4d") {
      cmd.effort = joint_acc;
    } else if (dynamics_) {
      dynamics_->GravityVector(target_joint, cmd.effort);
    } else {
      cmd.effort = ArmMotion::Joint6Dof{};
    }
    // 本地更新参考关节角
    current_joint = target_joint;
    return true;
  };

  return BeginMoveTask(std::move(task), duration);
}

int ArmController::MoveJMulti(const std::vector<ArmMotion::Joint6Dof>& points, double r, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveJMulti] failed: motion state is not idle");
    return -1;
  }

  if (points.size() < 2) {
    ARM_LOG_ERROR("[MoveJMulti] failed: points size {} < 2", points.size());
    return -1;
  }

  // 检查输入点关节限位
  for (size_t p = 0; p < points.size(); ++p) {
    for (int i = 0; i < 6; ++i) {
      if (points[p][i] < joint_min_[i] - kLimitEps || points[p][i] > joint_max_[i] + kLimitEps) {
        ARM_LOG_ERROR(
            "[MoveJMulti] input point {} joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
            p, i, points[p][i], joint_min_[i], joint_max_[i]);
        return -1;
      }
    }
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveJMulti] failed: current joint state is not available");
    return -1;
  }

  current_joint = ResolveStartJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  std::vector<ArmMotion::Joint6Dof> path_points;
  path_points.reserve(points.size() + 1);
  path_points.push_back(current_joint);
  path_points.insert(path_points.end(), points.begin(), points.end());

  auto path = std::make_shared<MultiPointJointPath>(std::move(path_points), r);
  const double path_length = path->GetTotalLength();
  if (path_length <= 1e-7) {
    ARM_LOG_ERROR("[MoveJMulti] failed: invalid path total length");
    return -1;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a / path_length;

  TimeScalerBase::PlanConfig config;
  auto time_scaler = CreateTimeScaler(normalized_v, normalized_a, config);
  if (!time_scaler) {
    ARM_LOG_ERROR("[MoveJMulti] failed: time scaler setting");
    return -1;
  }
  if (!time_scaler->Plan(config)) {
    ARM_LOG_ERROR("[MoveJMulti] failed: time scaler plan");
    return -1;
  }

  auto trajectory = std::make_shared<JointTrajectory>(path, std::move(time_scaler));
  double duration = trajectory->GetDuration();

  SampleTask task = [trajectory, this](double t, ArmMotion::JointCommand& cmd) {
    ArmMotion::JointTrajectoryPoint point;
    if (!trajectory->Sample(t, point)) {
      ARM_LOG_ERROR("[MoveJMulti] failed: Sample at t={}", t);
      return false;
    }

    for (int i = 0; i < 6; ++i) {
      if (point.position.q[i] < joint_min_[i] - kLimitEps || point.position.q[i] > joint_max_[i] + kLimitEps) {
        ARM_LOG_ERROR("[MoveJMulti] failed: sampled joint[{}]={:.4f} out of limit at t={}",
                      i, point.position[i], t);
        return false;
      }
    }

    cmd.position = point.position;
    cmd.velocity = point.velocity;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    if (robot_name_ == "s6h4d") {
      cmd.effort = point.acceleration;
    } else if (dynamics_) {
      dynamics_->GravityVector(point.position, cmd.effort);
    } else {
      cmd.effort = ArmMotion::Joint6Dof{};
    }
    return true;
  };

  return BeginMoveTask(std::move(task), duration);
}

int ArmController::MoveJMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a) {
  if (motion_state_.load() != MotionState::Idle) {
    ARM_LOG_ERROR("[MoveJMulti] failed: motion state is not idle");
    return -1;
  }

  if (points.size() < 2) {
    ARM_LOG_ERROR("[MoveJMulti] failed: points size {} < 2", points.size());
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[MoveJMulti] failed: current joint state is not available");
    return -1;
  }

  // 逐个位姿逆解，每个点以上一个点的关节解作为参考，保证路径连续性
  std::vector<ArmMotion::Joint6Dof> joint_points;
  joint_points.reserve(points.size());
  ArmMotion::Joint6Dof reference_joint = current_joint;
  for (size_t p = 0; p < points.size(); ++p) {
    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(tool_frame_.Remove(points[p]), reference_joint, ik_solves)) {
      ARM_LOG_ERROR("[MoveJMulti] failed: IK at point {}", p);
      return -1;
    }
    ArmMotion::Joint6Dof target_joint;
    if (!SelectBestIKSolve(ik_solves, reference_joint, target_joint)) {
      ARM_LOG_ERROR("[MoveJMulti] failed: no valid IK solve at point {}", p);
      return -1;
    }
    joint_points.push_back(target_joint);
    reference_joint = target_joint;
  }

  return MoveJMulti(joint_points, r, v, a);
}

int ArmController::ServoJ(const ArmMotion::Joint6Dof& target_joint, double dt) {
  const MotionState s = motion_state_.load();
  if (s != MotionState::Idle && s != MotionState::Servo) {
    ARM_LOG_ERROR("[ServoJ] failed: motion state is not idle or servo");
    return -1;
  }
  if (dt <= 0.0) {
    ARM_LOG_ERROR("[ServoJ] failed: duration must be positive");
    return -1;
  }

  for (int i = 0; i < 6; ++i) {
    if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
      ARM_LOG_ERROR("[ServoJ] target joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
                    i, target_joint[i], joint_min_[i], joint_max_[i]);
      return -1;
    }
  }

  // 单独处理首次servo
  ArmMotion::Joint6Dof q_prev;
  if (prev_servo_joint_.has_value()) {
    q_prev = prev_servo_joint_.value();
  } else {
    ArmMotion::Joint6Dof current_joint;
    if (GetCurrentJoint(current_joint) != 0) {
      ARM_LOG_ERROR("[ServoJ] failed: current joint state is not available");
      return -1;
    }
    q_prev = ResolveStartJoint(current_joint);
  }

  ArmMotion::Joint6Dof q_now = target_joint;

  SampleTask task = [this, q_prev, q_now, dt](double t, ArmMotion::JointCommand& cmd) {
    for (int i = 0; i < 6; ++i) {
      // 线性插值
      cmd.position[i] = q_prev[i] + (q_now[i] - q_prev[i]) * t / dt;
      cmd.velocity[i] = (q_now[i] - q_prev[i]) / dt;
      cmd.stiffness[i] = motor_stiffness_[i];
      cmd.damping[i] = motor_damping_[i];
    }
    return true;
  };

  const int ret = BeginServoTask(std::move(task), dt);
  if (ret == 0) {
    prev_servo_joint_ = q_now;
  }
  return ret;
}

int ArmController::ServoJ(const ArmMotion::Pose6Dof& target_pose, double dt) {
  const MotionState s = motion_state_.load();
  if (s != MotionState::Idle && s != MotionState::Servo) {
    ARM_LOG_ERROR("[ServoJ] failed: motion state is not idle or servo");
    return -1;
  }
  if (dt <= 0.0) {
    ARM_LOG_ERROR("[ServoJ] failed: duration must be positive");
    return -1;
  }

  // 优先用上一次 servo 的目标关节作为逆解参考，保证连续 servo 时解分支稳定
  ArmMotion::Joint6Dof reference_joint;
  if (prev_servo_joint_.has_value()) {
    reference_joint = prev_servo_joint_.value();
  } else {
    if (GetCurrentJoint(reference_joint) != 0) {
      ARM_LOG_ERROR("[ServoJ] failed: current joint state is not available");
      return -1;
    }
  }

  ArmMotion::IKSolves ik_solves;
  if (!kinematic_solver_->IK(tool_frame_.Remove(target_pose), reference_joint, ik_solves)) {
    ARM_LOG_ERROR("[ServoJ] failed: IK at target pose=({}, {}, {})",
                  target_pose.x, target_pose.y, target_pose.z);
    return -1;
  }
  ArmMotion::Joint6Dof target_joint;
  if (!SelectBestIKSolve(ik_solves, reference_joint, target_joint)) {
    ARM_LOG_ERROR("[ServoJ] failed: no valid IK solve at target pose=({}, {}, {})",
                  target_pose.x, target_pose.y, target_pose.z);
    return -1;
  }

  return ServoJ(target_joint, dt);
}

int ArmController::ServoL(const ArmMotion::Pose6Dof& target_pose, double dt) {
  const MotionState s = motion_state_.load();
  if (s != MotionState::Idle && s != MotionState::Servo) {
    ARM_LOG_ERROR("[ServoL] failed: motion state is not idle or servo");
    return -1;
  }
  if (dt <= 0.0) {
    ARM_LOG_ERROR("[ServoL] failed: duration must be positive");
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  if (GetCurrentJoint(current_joint) != 0) {
    ARM_LOG_ERROR("[ServoL] failed: current joint state is not available");
    return -1;
  }
  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  // 单独处理首次servo
  ArmMotion::Pose6Dof p_prev;
  if (prev_servo_tcp_.has_value()) {
    p_prev = prev_servo_tcp_.value();
  } else {
    const ArmMotion::Joint6Dof start_joint = ResolveStartJoint(current_joint);
    ArmMotion::Pose6Dof flange;
    if (!kinematic_solver_->FK(start_joint, flange)) {
      ARM_LOG_ERROR("[ServoL] failed: flange pose is not available");
      return -1;
    }
    p_prev = tool_frame_.Compose(flange);
  }

  ArmMotion::Pose6Dof p_now = target_pose;

  const ToolFrame tool = tool_frame_;
  SampleTask task = [this, p_prev, p_now, dt, current_joint, tool](double t, ArmMotion::JointCommand& cmd) mutable {
    ArmMotion::Pose6Dof interp_pose;
    // 线性插值（位置）
    const double s = std::clamp(t / dt, 0.0, 1.0);
    interp_pose.x = p_prev.x + (p_now.x - p_prev.x) * s;
    interp_pose.y = p_prev.y + (p_now.y - p_prev.y) * s;
    interp_pose.z = p_prev.z + (p_now.z - p_prev.z) * s;

    // 球面线性插值 (Slerp) 四元数
    Eigen::Quaterniond q0(p_prev.quat.w, p_prev.quat.x, p_prev.quat.y, p_prev.quat.z);
    Eigen::Quaterniond q1(p_now.quat.w, p_now.quat.x, p_now.quat.y, p_now.quat.z);
    q0.normalize();
    q1.normalize();
    Eigen::Quaterniond qi = q0.slerp(s, q1);
    interp_pose.quat.x = qi.x();
    interp_pose.quat.y = qi.y();
    interp_pose.quat.z = qi.z();
    interp_pose.quat.w = qi.w();

    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(tool.Remove(interp_pose), current_joint, ik_solves)) {
      ARM_LOG_ERROR("[ServoL] failed: IK at t={}, pose=({}, {}, {})",
                    t, interp_pose.x, interp_pose.y, interp_pose.z);
      return false;
    }

    ArmMotion::Joint6Dof target_joint;
    if (!SelectBestIKSolve(ik_solves, current_joint, target_joint)) {
      ARM_LOG_ERROR("[ServoL] failed: no valid IK solve at t={}", t);
      return false;
    }

    for (int i = 0; i < 6; ++i) {
      if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
        ARM_LOG_ERROR("[ServoL] trajectory point joint[{}]={:.4f} out of limit at t={}",
                      i, target_joint[i], t);
        return false;
      }
    }

    // 差分：速度 = 相邻参考解之差 × 采样频率
    for (int i = 0; i < 6; ++i) {
      cmd.position[i] = target_joint[i];
      cmd.velocity[i] = (target_joint[i] - current_joint[i]) * sample_frequency_;
      cmd.stiffness[i] = motor_stiffness_[i];
      cmd.damping[i] = motor_damping_[i];
    }
    // 本地更新参考关节角
    current_joint = target_joint;
    return true;
  };

  const int ret = BeginServoTask(std::move(task), dt);
  if (ret == 0) {
    prev_servo_tcp_ = p_now;
  }
  return ret;
}

void ArmController::ServoStop() {
  prev_servo_joint_.reset();
  prev_servo_tcp_.reset();
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (motion_state_.load() == MotionState::Servo) {
    motion_state_.store(MotionState::Idle);
  }
}
