#include "arm_instance.h"

#include <cmath>
#include <limits>
#include <thread>

#include "arm_logger.h"

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
      double diff = ik_solves.solves[i].theta[j] - reference_joint.theta[j];
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
      found_valid_solve = true;
    }
  }

  return found_valid_solve;
}

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

  // 创建 LSPB 时间缩放器用于速度加速度约束
  LSPBTimeScaler time_scaler;
  TimeScalerBase::PlanConfig config;
  config.T = t;
  config.v_max = v;
  config.a_max = a;

  if (!time_scaler.Plan(config)) {
    // 处理规划失败的情况
    ARM_LOG_ERROR("[MoveL] failed: time scaler plan");
    return -1;
  }

  // 创建笛卡尔轨迹
  cartesian_trajectory_ = std::make_unique<CartesianTrajectory>(path, time_scaler);
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
    ArmMotion::JointState_t joint_cmd{
        .position = target_joint, .velocity = joint_vel, .accel = joint_acc};
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
    if (target_joint.theta[i] < joint_min_[i] || target_joint.theta[i] > joint_max_[i]) {
      ARM_LOG_ERROR("[MoveJ] target joint[{}]={:.4f} out of limit [{:.4f}, {:.4f}]",
                    i, target_joint.theta[i], joint_min_[i], joint_max_[i]);
      return -1;
    }
  }
  
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[MoveJ] failed: GetJointState at start");
    return -1;
  }

  ArmMotion::Joint6Dof_t start_joint = current_state.position;
  JointPath path(start_joint, target_joint);
  CubicTimeScaler time_scaler;
  CubicTimeScaler::PlanConfig config;
  config.T = t;

  if (!time_scaler.Plan(config)) {
    ARM_LOG_ERROR("[MoveJ] failed: time scaler plan");
    return -1;
  }

  joint_trajectory_ = std::make_unique<JointTrajectory>(path, time_scaler);
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

    ArmMotion::JointState_t joint_cmd{.position = point.position,
                                      .velocity = point.velocity,
                                      .accel = point.acceleration};
    if (!arm_ctrl_.SetJointState(joint_cmd)) {
      ARM_LOG_ERROR(
          "[MoveJ] failed: SetJointState at t={}, q0={}, q1={}, q2={}",
          current_time, point.position.theta[0], point.position.theta[1],
          point.position.theta[2]);
      return -1;
    }

    // 采样后
    for (int i = 0; i < 6; ++i) {
      if (point.position.theta[i] < joint_min_[i] || point.position.theta[i] > joint_max_[i]) {
        ARM_LOG_WARN("[MoveJ] trajectory point joint[{}]={:.4f} out of limit at t={}",
                    i, point.position.theta[i], current_time);
        return -1;
      }
    }

    // 等待下一次循环，确保按照 dt_ 的频率执行
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int ArmInstance::GetCurrentPose(ArmMotion::Pose6Dof_t &current_pose) {
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

int ArmInstance::GetCurrentJoint(ArmMotion::Joint6Dof_t &current_joint) {
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    ARM_LOG_ERROR("[GetCurrentJoint] failed: GetJointState");
    return -1;
  }
  current_joint = current_state.position;
  return 0;
}