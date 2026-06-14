#include "arm_instance.h"

#include <cmath>
#include <limits>
#include <thread>
#include <algorithm>
#include "arm_logger.h"

namespace {

double JointPathLength(const ArmMotion::Joint6Dof_t& q0,
                       const ArmMotion::Joint6Dof_t& q1) {
  double sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double d = q1.q[i] - q0.q[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

double CartesianPosDistance(const ArmMotion::Pose6Dof_t& p0,
                            const ArmMotion::Pose6Dof_t& p1) {
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double dz = p1.z - p0.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

bool ArmInstance::SelectBestIKSolve(
    const ArmMotion::IKSolves_t& ik_solves,
    const ArmMotion::Joint6Dof_t& reference_joint,
    ArmMotion::Joint6Dof_t& selected_joint) const {
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

void ArmInstance::SetMotorParams(ArmMotion::Joint6Dof_t stiffness, ArmMotion::Joint6Dof_t damping) {
  motor_stiffness_ = stiffness;
  motor_damping_ = damping;
}

void ArmInstance::SetJointLimits(ArmMotion::Joint6Dof_t joint_min, ArmMotion::Joint6Dof_t joint_max) {
  joint_min_ = joint_min;
  joint_max_ = joint_max;
}

void ArmInstance::SetTrajectoryMode(TrajectoryMode mode) {
  traj_planning_mode = mode;
}

void ArmInstance::Init(ArmMotion::Joint6Dof_t stiffness, ArmMotion::Joint6Dof_t damping,
                       ArmMotion::Joint6Dof_t joint_min, ArmMotion::Joint6Dof_t joint_max) {
  SetMotorParams(stiffness, damping);
  SetJointLimits(joint_min, joint_max);
  arm_ctrl_.Init();
}

void ArmInstance::Enable() { arm_ctrl_.Enable(); }

void ArmInstance::Disable() { arm_ctrl_.Disable(); }

int ArmInstance::MoveL(ArmMotion::Pose6Dof_t target_pose, double v, double a,
                       double t) {
  // 获取当前关节状态作为起始位置
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[MoveL] failed: GetJointState at start");
    return -1;
  }

  // 通过正运动学获取当前笛卡尔位置
  ArmMotion::Pose6Dof_t current_pose;
  if (!kinematic_solver_->FK(current_state.position, current_pose)) {
    ARM_LOG_ERROR("[MoveL] failed: FK at start");
    return -1;
  }

  // 创建笛卡尔路径（直线路径）
  CartesianLinearPath path(current_pose, target_pose);

  const double path_length = CartesianPosDistance(current_pose, target_pose);
  if (path_length <= 1e-9) {
    ARM_LOG_WARN("[MoveL] skip: path length too small");
    return 0;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a /path_length;

  // 处理时间规划器选择
  TimeScalerBase::PlanConfig config;
  switch (traj_planning_mode) {
    case TrajectoryMode::CUBIC: {
      double t_v = 1.5 / normalized_v;
      double t_a = std::sqrt((6.0 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<CubicTimeScaler>();
      break;
    }
    case TrajectoryMode::QUINTIC: {
      double t_v = 1.875 / normalized_v;
      double t_a = std::sqrt((5.7735 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<QuinticTimeScaler>();
      break;
    }
    case TrajectoryMode::LSPB: {
      config.T = t;
      config.a_max = normalized_a;
      config.v_max = normalized_v;
      time_scaler_ = std::make_unique<LSPBTimeScaler>();
      break;
    }
    default:
      ARM_LOG_ERROR("[MoveL] failed: time scaler setting");
      return -1;
      break;
  }

  if (!time_scaler_->Plan(config)) {
    // 处理规划失败的情况
    ARM_LOG_ERROR("[MoveL] failed: time scaler plan");
    return -1;
  }

  // 创建笛卡尔轨迹
  cartesian_trajectory_ = std::make_unique<CartesianTrajectory>(path, *time_scaler_);
  double duration = cartesian_trajectory_->GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    if (current_time > duration) {
      current_time = duration;
    }
    ArmMotion::CartesianTrajectoryPoint_t cart_point;
    if (!cartesian_trajectory_->Sample(current_time, cart_point)) {
      // 处理采样失败的情况
      ARM_LOG_ERROR("[MoveL] failed: Sample at t={}", current_time);
      return -1;
    }

    // 通过逆运动学求解目标关节角
    ArmMotion::IKSolves_t ik_solves;
    if (!kinematic_solver_->IK(cart_point.pose, current_state.position,
                               ik_solves)) {
      ARM_LOG_ERROR("[MoveL] failed: IK at t={}, pose=({}, {}, {})",
                    current_time, cart_point.pose.x, cart_point.pose.y,
                    cart_point.pose.z);
      return -1;
    }

    // 选择最优逆解：过滤不可达解并优先选与当前姿态最接近的解。
    ArmMotion::Joint6Dof_t target_joint;
    if (!SelectBestIKSolve(ik_solves, current_state.position, target_joint)) {
      ARM_LOG_ERROR("[MoveL] failed: no valid IK solve at t={}", current_time);
      return -1;
    }

    // 通过雅可比矩阵将笛卡尔速度/加速度转换为关节速度/加速度
    jacobian_->Update(target_joint, dt_);

    ArmMotion::Joint6Dof_t joint_vel;
    jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6);

    ArmMotion::Joint6Dof_t joint_acc;
    jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6);

    // 发送控制命令到机械臂
    ArmMotion::JointCommand_t joint_cmd;
    joint_cmd.position = target_joint;
    joint_cmd.velocity = joint_vel;
    joint_cmd.stiffness = motor_stiffness_;
    joint_cmd.damping = motor_damping_;
    // TODO: 力矩支持
    if (!arm_ctrl_.SetJointState(joint_cmd)) {
      ARM_LOG_ERROR("[MoveL] failed: SetJointState at t={}", current_time);
      return -1;
    }

    // 更新当前关节状态用于下一步的逆解初值
    current_state.position = target_joint;

    // 等待下一次循环，确保按照 dt_ 的频率执行
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int ArmInstance::MoveJ(ArmMotion::Joint6Dof_t target_joint, double v,
                       double a, double t) {
  for (int i = 0; i < 6; i++) {
    if (target_joint.q[i] < joint_min_.q[i] || target_joint.q[i] > joint_max_.q[i]) {
      ARM_LOG_ERROR("[MoveJ] target joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
                    i, target_joint.q[i], joint_min_.q[i], joint_max_.q[i]);
      return -1;
    }
  }

  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[MoveJ] failed: GetJointState at start");
    return -1;
  }

  ArmMotion::Joint6Dof_t start_joint = current_state.position;
  const double path_length = JointPathLength(start_joint, target_joint);
  if (path_length <= 1e-9) {
    ARM_LOG_WARN("[MoveJ] skip: path length too small");
    return 0;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a /path_length;

  JointPath path(start_joint, target_joint);
  
  // 处理时间规划器选择
  TimeScalerBase::PlanConfig config;
  switch (traj_planning_mode) {
    case TrajectoryMode::CUBIC: {
      double t_v = 1.5 / normalized_v;
      double t_a = std::sqrt((6.0 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<CubicTimeScaler>();
      break;
    }
    case TrajectoryMode::QUINTIC: {
      double t_v = 1.875 / normalized_v;
      double t_a = std::sqrt((5.7735 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<QuinticTimeScaler>();
      break;
    }
    case TrajectoryMode::LSPB: {
      config.T = t;
      config.a_max = normalized_a;
      config.v_max = normalized_v;
      time_scaler_ = std::make_unique<LSPBTimeScaler>();
      break;
    }
    default:
      ARM_LOG_ERROR("[MoveL] failed: time scaler setting");
      return -1;
      break;
  }

  if (!time_scaler_->Plan(config)) {
    ARM_LOG_ERROR("[MoveJ] failed: time scaler plan");
    return -1;
  }

  joint_trajectory_ = std::make_unique<JointTrajectory>(path, *time_scaler_);
  double duration = joint_trajectory_->GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    if (current_time > duration) {
      current_time = duration;
    }
    ArmMotion::JointTrajectoryPoint_t point;
    if (!joint_trajectory_->Sample(current_time, point)) {
      ARM_LOG_ERROR("[MoveJ] failed: Sample at t={}", current_time);
      return -1;
    }

    ArmMotion::JointCommand_t joint_cmd;
    joint_cmd.position = point.position;
    joint_cmd.velocity = point.velocity;
    joint_cmd.stiffness = motor_stiffness_;
    joint_cmd.damping = motor_damping_;
    // TODO: 力矩支持
    if (!arm_ctrl_.SetJointState(joint_cmd)) {
      ARM_LOG_ERROR(
          "[MoveJ] failed: SetJointState at t={}, q0={}, q1={}, q2={}",
          current_time, point.position.q[0], point.position.q[1],
          point.position.q[2]);
      return -1;
    }

    // 采样后
    for (int i = 0; i < 6; ++i) {
      if (point.position.q[i] < joint_min_.q[i] || point.position.q[i] > joint_max_.q[i]) {
        ARM_LOG_WARN("[MoveJ] trajectory point joint[{}]={:.4f} out of limit at t={}",
                     i, point.position.q[i], current_time);
        return -1;
      }
    }

    // 等待下一次循环，确保按照 dt_ 的频率执行
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int ArmInstance::MoveJMulti(const std::vector<ArmMotion::Joint6Dof_t>& points,
                            double r, double v, double a, double t) {
  if (points.size() < 2) {
    ARM_LOG_ERROR("[MoveJMulti] failed: points size {} < 2", points.size());
    return -1;
  }

  // 检查输入点关节限位
  for (size_t p = 0; p < points.size(); ++p) {
    for (int i = 0; i < 6; ++i) {
      if (points[p].q[i] < joint_min_.q[i] || points[p].q[i] > joint_max_.q[i]) {
        ARM_LOG_ERROR(
            "[MoveJMulti] input point {} joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
            p, i, points[p].q[i], joint_min_.q[i], joint_max_.q[i]);
        return -1;
      }
    }
  }

  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[MoveJMulti] failed: GetJointState at start");
    return -1;
  }

  std::vector<ArmMotion::Joint6Dof_t> path_points;
  path_points.reserve(points.size() + 1);
  path_points.push_back(current_state.position);
  path_points.insert(path_points.end(), points.begin(), points.end());

  MultiPointJointPath path(std::move(path_points), r);
  const double path_length = path.GetTotalLength();
  if (path_length <= 1e-9) {
    ARM_LOG_ERROR("[MoveJMulti] failed: invalid path total length");
    return -1;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a /path_length;

  // 处理时间规划器选择
  TimeScalerBase::PlanConfig config;
  switch (traj_planning_mode) {
    case TrajectoryMode::CUBIC: {
      double t_v = 1.5 / normalized_v;
      double t_a = std::sqrt((6.0 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<CubicTimeScaler>();
      break;
    }
    case TrajectoryMode::QUINTIC: {
      double t_v = 1.875 / normalized_v;
      double t_a = std::sqrt((5.7735 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<QuinticTimeScaler>();
      break;
    }
    case TrajectoryMode::LSPB: {
      config.T = t;
      config.a_max = normalized_a;
      config.v_max = normalized_v;
      time_scaler_ = std::make_unique<LSPBTimeScaler>();
      break;
    }
    default:
      ARM_LOG_ERROR("[MoveL] failed: time scaler setting");
      return -1;
      break;
  }

  if (!time_scaler_->Plan(config)) {
    ARM_LOG_ERROR("[MoveJMulti] failed: time scaler plan");
    return -1;
  }

  JointTrajectory trajectory(path, *time_scaler_);
  double duration = trajectory.GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    if (current_time > duration) {
      current_time = duration;
    }

    ArmMotion::JointTrajectoryPoint_t point;
    if (!trajectory.Sample(current_time, point)) {
      ARM_LOG_ERROR("[MoveJMulti] failed: Sample at t={}", current_time);
      return -1;
    }

    // 轨迹点限位保护
    for (int i = 0; i < 6; ++i) {
      if (point.position.q[i] < joint_min_.q[i] || point.position.q[i] > joint_max_.q[i]) {
        ARM_LOG_ERROR("[MoveJMulti] failed: sampled joint[{}]={:.4f} out of limit at t={}",
                      i, point.position.q[i], current_time);
        return -1;
      }
    }

    ArmMotion::JointCommand_t joint_cmd;
    joint_cmd.position = point.position;
    joint_cmd.velocity = point.velocity;
    joint_cmd.stiffness = motor_stiffness_;
    joint_cmd.damping = motor_damping_;
    // TODO: 力矩、刚度、阻尼支持
    if (!arm_ctrl_.SetJointState(joint_cmd)) {
      ARM_LOG_ERROR("[MoveJMulti] failed: SetJointState at t={}", current_time);
      return -1;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int ArmInstance::MoveLMulti(const std::vector<ArmMotion::Pose6Dof_t>& points,
                            double r, double v, double a, double t) {
  if (points.size() < 1) {
    ARM_LOG_ERROR("[MoveLMulti] failed: points size {} < 1", points.size());
    return -1;
  }

  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: GetJointState at start");
    return -1;
  }

  ArmMotion::Pose6Dof_t current_pose;
  if (!kinematic_solver_->FK(current_state.position, current_pose)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: FK at start");
    return -1;
  }

  std::vector<ArmMotion::Pose6Dof_t> path_points;
  path_points.reserve(points.size() + 1);
  path_points.push_back(current_pose);
  path_points.insert(path_points.end(), points.begin(), points.end());

  CartesianMultiPointPath path(std::move(path_points), r);
  const double path_length = path.GetTotalLength();
  if (path_length <= 1e-9) {
    ARM_LOG_ERROR("[MoveLMulti] failed: invalid path total length");
    return -1;
  }

  // 单位归一化
  double normalized_v = v / path_length;
  double normalized_a = a /path_length;
  
  // 处理时间规划器选择
  TimeScalerBase::PlanConfig config;
  switch (traj_planning_mode) {
    case TrajectoryMode::CUBIC: {
      double t_v = 1.5 / normalized_v;
      double t_a = std::sqrt((6.0 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<CubicTimeScaler>();
      break;
    }
    case TrajectoryMode::QUINTIC: {
      double t_v = 1.875 / normalized_v;
      double t_a = std::sqrt((5.7735 / normalized_a));
      config.T = std::max(t_v, t_a);
      time_scaler_ = std::make_unique<QuinticTimeScaler>();
      break;
    }
    case TrajectoryMode::LSPB: {
      config.T = t;
      config.a_max = normalized_a;
      config.v_max = normalized_v;
      time_scaler_ = std::make_unique<LSPBTimeScaler>();
      break;
    }
    default:
      ARM_LOG_ERROR("[MoveL] failed: time scaler setting");
      return -1;
      break;
  }

  if (!time_scaler_->Plan(config)) {
    ARM_LOG_ERROR("[MoveLMulti] failed: time scaler plan");
    return -1;
  }

  CartesianTrajectory trajectory(path, *time_scaler_);
  double duration = trajectory.GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    if (current_time > duration) {
      current_time = duration;
    }

    ArmMotion::CartesianTrajectoryPoint_t cart_point;
    if (!trajectory.Sample(current_time, cart_point)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: Sample at t={}", current_time);
      return -1;
    }

    ArmMotion::IKSolves_t ik_solves;
    if (!kinematic_solver_->IK(cart_point.pose, current_state.position, ik_solves)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: IK at t={}, pose=({}, {}, {})",
                    current_time, cart_point.pose.x, cart_point.pose.y, cart_point.pose.z);
      return -1;
    }

    ArmMotion::Joint6Dof_t target_joint;
    if (!SelectBestIKSolve(ik_solves, current_state.position, target_joint)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: no valid IK solve at t={}", current_time);
      return -1;
    }

    jacobian_->Update(target_joint, dt_);

    ArmMotion::Joint6Dof_t joint_vel;
    jacobian_->InverseVelocity(cart_point.velocity, joint_vel, 1e-6);

    ArmMotion::Joint6Dof_t joint_acc;
    jacobian_->InverseAccel(cart_point.acceleration, joint_acc, 1e-6);

    ArmMotion::JointCommand_t joint_cmd;
    joint_cmd.position = target_joint;
    joint_cmd.velocity = joint_vel;
    joint_cmd.stiffness = motor_stiffness_;
    joint_cmd.damping = motor_damping_;
    // TODO: 力矩支持
    if (!arm_ctrl_.SetJointState(joint_cmd)) {
      ARM_LOG_ERROR("[MoveLMulti] failed: SetJointState at t={}", current_time);
      return -1;
    }

    current_state.position = target_joint;

    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int ArmInstance::GetCurrentPose(ArmMotion::Pose6Dof_t& current_pose) {
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[GetCurrentPose] failed: GetJointState");
    return -1;
  }
  if (!kinematic_solver_->FK(current_state.position, current_pose)) {
    ARM_LOG_ERROR("[GetCurrentPose] failed: FK");
    return -1;
  }
  return 0;
}

int ArmInstance::GetCurrentJoint(ArmMotion::Joint6Dof_t& current_joint) {
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[GetCurrentJoint] failed: GetJointState");
    return -1;
  }
  current_joint = current_state.position;
  return 0;
}

void ArmInstance::SetToolPose(const ArmMotion::Pose6Dof_t& tool_pose) {
  kinematic_solver_->SetToolPose(tool_pose);
}

void ArmInstance::ClearToolPose() {
  kinematic_solver_->ClearToolPose();
}
