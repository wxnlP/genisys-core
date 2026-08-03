#include "arm_controller.h"

#include <cmath>
#include <cstring>
#include <thread>
#include <Eigen/Dense>
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

  for (int i = 0; i < 8; ++i) {
    const char j1_flag = ik_solves.solveFlag[i][0];
    const char j23_flag = ik_solves.solveFlag[i][1];
    const char wrist_flag = ik_solves.solveFlag[i][2];

    // 0 表示不可达；-1 表示奇异位形仍可用但可作为次优。
    if (j1_flag == 0 || j23_flag == 0 || wrist_flag == 0) {
      continue;
    }

    double score = 0.0;
    for (int j = 0; j < 6; ++j) {
      double diff = ik_solves.solves[i].q[j] - reference_joint.q[j];
      while (diff > kPi) {
        diff -= kTwoPi;
      }
      while (diff <= -kPi) {
        diff += kTwoPi;
      }
      score += diff * diff;

      // 对接近限位的解增加惩罚，优先选择远离限位的解
      double val = ik_solves.solves[i].q[j];
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
      selected_joint = ik_solves.solves[i];
      // 将候选解展开到 reference_joint 邻域，避免 ±2pi 等价角导致的突跳。
      for (int j = 0; j < 6; ++j) {
        double d = selected_joint.q[j] - reference_joint.q[j];
        while (d > kPi) {
          d -= kTwoPi;
        }
        while (d <= -kPi) {
          d += kTwoPi;
        }
        selected_joint.q[j] = reference_joint.q[j] + d;
      }
      found_valid_solve = true;
    }
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
    motion_state_.store(MotionState::IDLE);
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
    enum class Phase { BOOTSTRAP, STEADY, BACKOFF };
    ArmMotion::JointCommand cmd;
    while (!stoken.stop_requested()) {
      SampleContext context;
      {
        std::unique_lock<std::mutex> lock(sample_mutex_);
        if (sample_queue_.empty()) {
          motion_state_.store(MotionState::IDLE);  // 无轨迹，转入空闲
          // 睡眠
          sample_cv_.wait(lock, [this, &stoken]() {
            return !sample_queue_.empty() || stoken.stop_requested();
          });
          if (stoken.stop_requested()) { break; }
        }
        // 有轨迹，准备采样
        motion_state_.store(MotionState::ACTIVE);
        context = std::move(sample_queue_.front());
        sample_queue_.pop_front();
      }

      Phase phase = Phase::BOOTSTRAP;
      double t = 0.0;
      double dt = 1.0 / sample_frequency_;
      auto t_last_push = std::chrono::steady_clock::now();

      while (t <= context.duration + 1e-7 && !stoken.stop_requested()) {
        // 采样
        if (!context.task(t, cmd)) break;

        // 入队（满则重试，永不丢点）
        while (!command_queue_.Push(cmd) && !stoken.stop_requested()) {
          phase = Phase::BACKOFF;
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
    }
  });
}

int ArmController::GetCurrentPose(ArmMotion::Pose6Dof& current_pose) {
  ArmMotion::Joint6Dof current_joint;
  GetCurrentJoint(current_joint);
  if (!kinematic_solver_->FK(current_joint, current_pose)) {
    ARM_LOG_ERROR("[GetCurrentPose] failed: FK");
    return -1;
  }
  return 0;
}

int ArmController::GetCurrentJoint(ArmMotion::Joint6Dof& current_joint) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_joint = current_state_.position;
  return 0;
}

int ArmController::JointToPose(const ArmMotion::Joint6Dof& joint, ArmMotion::Pose6Dof& pose) {
  if (!kinematic_solver_->FK(joint, pose)) {
    ARM_LOG_ERROR("[JointToPose] failed: FK");
    return -1;
  }
  return 0;
}

void ArmController::SetToolPose(const ArmMotion::Pose6Dof& tool_pose) {
  kinematic_solver_->SetToolPose(tool_pose);
}

void ArmController::ClearToolPose() {
  kinematic_solver_->ClearToolPose();
}

void ArmController::SetJointState(const ArmMotion::JointState& joint_state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_state_ = joint_state;
}

bool ArmController::GetJointCommand(ArmMotion::JointCommand& joint_command) {
  if (command_queue_.Pop(joint_command)) {
    return true;
  }
  joint_command.damping = motor_damping_;
  joint_command.stiffness = motor_stiffness_;
  return false;
}

int ArmController::WaitForCompletion() {
  while (motion_state_.load() != MotionState::IDLE || !command_queue_.IsEmpty()) {
    std::this_thread::yield();
  }
  return 0;
}

int ArmController::WaitForCompletion(ArmMotion::Joint6Dof& target, uint32_t timeout_ms) {
  auto t0 = std::chrono::steady_clock::now();
  double period = 1.0 / sample_frequency_;
  ArmMotion::Joint6Dof current;
  for (int i=0; i < timeout_ms; ++i) {
    GetCurrentJoint(current);
    if (JointPathLength(current, target) < 0.01) {
      return 0;
    }
    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (ms > timeout_ms) {
      ARM_LOG_WARN("[WaitForCompletion] timeout: current joint={:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}",
                   current[0], current[1], current[2], current[3], current[4], current[5]);
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period * 1000)));
  }
  return -1;
}

int ArmController::WaitForCompletion(ArmMotion::Pose6Dof& target, uint32_t timeout_ms) {
  auto t0 = std::chrono::steady_clock::now();
  double period = 1.0 / sample_frequency_;
  ArmMotion::Pose6Dof current;
  for (int i=0; i < timeout_ms; ++i) {
    GetCurrentPose(current);
    if (CartesianPosDistance(current, target) < 0.005) {
      return 0;
    }
    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (ms > timeout_ms) {
      ARM_LOG_WARN("[WaitForCompletion] timeout: current pose={:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}",
                   current.x, current.y, current.z, current.quat.x, current.quat.y, current.quat.z, current.quat.w);
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(period * 1000)));
  }
  return -1;
}

int ArmController::MoveL(ArmMotion::Pose6Dof target_pose, double v, double a) {
  ArmMotion::Joint6Dof current_joint;
  GetCurrentJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  // 通过正运动学获取当前笛卡尔位置
  ArmMotion::Pose6Dof current_pose;
  if (!kinematic_solver_->FK(current_joint, current_pose)) {
    ARM_LOG_ERROR("[MoveL] failed: FK at start");
    return -1;
  }

  const double path_length = CartesianPosDistance(current_pose, target_pose);
  if (path_length <= 1e-7) {
    ARM_LOG_WARN("[MoveL] skip: path length too small");
    return 0;
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
  auto path = std::make_shared<CartesianLinearPath>(current_pose, target_pose);
  // 创建笛卡尔轨迹
  auto trajectory = std::make_shared<CartesianTrajectory>(path, std::move(time_scaler));
  double duration = trajectory->GetDuration();

  SampleTask task = [trajectory, current_joint, this](double t, ArmMotion::JointCommand& cmd) mutable {
    ArmMotion::CartesianTrajectoryPoint cart_point;
    if (!trajectory->Sample(t, cart_point)) {
      ARM_LOG_ERROR("[MoveL] failed: Sample at t={}", t);
      return false;
    }

    // 通过逆运动学求解目标关节角
    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(cart_point.pose, current_joint, ik_solves)) {
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
    jacobian_->Update(target_joint, 1.0 / sample_frequency_);

    ArmMotion::Joint6Dof joint_vel;
    jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6);

    ArmMotion::Joint6Dof joint_acc;
    jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6);

    cmd.position = target_joint;
    cmd.velocity = joint_vel;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    // s6h4d特殊处理
    if (robot_name_ == "s6h4d") {
      cmd.effort = joint_acc;
    } else {
      // 重力补偿
      dynamics_->GravityVector(target_joint, cmd.effort);
    }
    // 本地更新参考关节角
    current_joint = target_joint;
    return true;
  };

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), duration);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::MoveJ(ArmMotion::Joint6Dof target_joint, double v, double a) {
  for (int i = 0; i < 6; i++) {
    if (target_joint[i] < joint_min_[i] - kLimitEps || target_joint[i] > joint_max_[i] + kLimitEps) {
      ARM_LOG_ERROR("[MoveJ] target joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
                    i, target_joint[i], joint_min_[i], joint_max_[i]);
      return -1;
    }
  }

  ArmMotion::Joint6Dof current_joint;
  GetCurrentJoint(current_joint);

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
    } else {
      // 重力补偿
      dynamics_->GravityVector(point.position, cmd.effort);
    }
    return true;
  };

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), duration);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::MoveLMulti(const std::vector<ArmMotion::Pose6Dof>& points, double r, double v, double a) {
  if (points.size() < 1) {
    ARM_LOG_ERROR("[MoveLMulti] failed: points size {} < 1", points.size());
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  GetCurrentJoint(current_joint);

  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  ArmMotion::Pose6Dof current_pose;
  if (!kinematic_solver_->FK(current_joint, current_pose)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: FK at start");
    return -1;
  }

  std::vector<ArmMotion::Pose6Dof> path_points;
  path_points.reserve(points.size() + 1);
  path_points.push_back(current_pose);
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

  SampleTask task = [trajectory, current_joint, this](double t, ArmMotion::JointCommand& cmd) mutable {
    ArmMotion::CartesianTrajectoryPoint cart_point;
    if (!trajectory->Sample(t, cart_point)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: Sample at t={}", t);
      return false;
    }

    ArmMotion::IKSolves ik_solves;
    if (!kinematic_solver_->IK(cart_point.pose, current_joint, ik_solves)) {
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

    jacobian_->Update(target_joint, 1.0 / sample_frequency_);
    ArmMotion::Joint6Dof joint_vel;
    jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6);
    ArmMotion::Joint6Dof joint_acc;
    jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6);

    cmd.position = target_joint;
    cmd.velocity = joint_vel;
    cmd.stiffness = motor_stiffness_;
    cmd.damping = motor_damping_;
    if (robot_name_ == "s6h4d") {
      cmd.effort = joint_acc;
    } else {
      dynamics_->GravityVector(target_joint, cmd.effort);
    }
    // 本地更新参考关节角
    current_joint = target_joint;
    return true;
  };

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), duration);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::MoveJMulti(const std::vector<ArmMotion::Joint6Dof>& points, double r, double v, double a) {
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
  GetCurrentJoint(current_joint);

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
    } else {
      dynamics_->GravityVector(point.position, cmd.effort);
    }
    return true;
  };

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), duration);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::ServoJ(const ArmMotion::Joint6Dof& target_joint, double dt) {
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
  if (!prev_servo_joint_.has_value()) {
    ArmMotion::Joint6Dof current_joint;
    GetCurrentJoint(current_joint);
    prev_servo_joint_ = current_joint;
  } 

  ArmMotion::Joint6Dof q_prev = prev_servo_joint_.value();
  ArmMotion::Joint6Dof q_now = target_joint;
  prev_servo_joint_ = q_now;

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

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), dt);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}

int ArmController::ServoL(const ArmMotion::Pose6Dof& target_pose, double dt) {
  if (dt <= 0.0) {
    ARM_LOG_ERROR("[ServoL] failed: duration must be positive");
    return -1;
  }

  ArmMotion::Joint6Dof current_joint;
  GetCurrentJoint(current_joint);
  for (int i = 0; i < 6; ++i) {
    if (current_joint[i] < joint_min_[i]) {
      current_joint[i] = joint_min_[i];
    } else if (current_joint[i] > joint_max_[i]) {
      current_joint[i] = joint_max_[i];
    }
  }

  // 单独处理首次servo
  if (!prev_servo_pose_.has_value()) {
    ArmMotion::Pose6Dof current_pose;
    GetCurrentPose(current_pose);
    prev_servo_pose_ = current_pose;
  }

  ArmMotion::Pose6Dof p_prev = prev_servo_pose_.value();
  ArmMotion::Pose6Dof p_now = target_pose;
  prev_servo_pose_ = p_now;

  SampleTask task = [this, p_prev, p_now, dt, current_joint](double t, ArmMotion::JointCommand& cmd) mutable {
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
    if (!kinematic_solver_->IK(interp_pose, current_joint, ik_solves)) {
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

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_queue_.emplace_back(std::move(task), dt);
    motion_state_.store(MotionState::ACTIVE);
  }
  sample_cv_.notify_one();
  return 0;
}


void ArmController::ServoStop() {
  prev_servo_joint_.reset();
  prev_servo_pose_.reset();
}