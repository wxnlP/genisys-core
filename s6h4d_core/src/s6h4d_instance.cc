#include "s6h4d_instance.h"

#include <cmath>
#include <limits>
#include <thread>

namespace {

double WrapToPi(double angle) {
  constexpr double kTwoPi = 2.0 * M_PI;
  while (angle > M_PI) {
    angle -= kTwoPi;
  }
  while (angle <= -M_PI) {
    angle += kTwoPi;
  }
  return angle;
}

double JointDistanceSq(const ArmMotion::Joint6Dof_t& lhs,
                       const ArmMotion::Joint6Dof_t& rhs) {
  double dist_sq = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double diff = WrapToPi(lhs.theta[i] - rhs.theta[i]);
    dist_sq += diff * diff;
  }
  return dist_sq;
}

}  // namespace

bool S6h4dInstance::SelectBestIKSolve(
    const ArmMotion::IKSolves_t& ik_solves,
    const ArmMotion::Joint6Dof_t& reference_joint,
    ArmMotion::Joint6Dof_t& selected_joint) const {
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

    double score = JointDistanceSq(ik_solves.solves[i], reference_joint);

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

int S6h4dInstance::MoveL(ArmMotion::Pose6Dof_t target_pose, double v, double a,
                         double t) {
  // 获取当前关节状态作为起始位置
  ArmMotion::JointState_t current_state;
  if (!arm_ctrl_.GetJointState(current_state)) {
    return -1;
  }

  // 通过正运动学获取当前笛卡尔位置
  ArmMotion::Pose6Dof_t current_pose;
  if (!kinematic_solver_->FK(current_state.position, current_pose)) {
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
    return -1;
  }

  // 创建笛卡尔轨迹
  cartesian_trajectory_ =
      std::make_unique<CartesianTrajectory>(path, time_scaler);
  double duration = cartesian_trajectory_->GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    ArmMotion::CartesianTrajectoryPoint_t cart_point;
    if (!cartesian_trajectory_->Sample(current_time, cart_point)) {
      // 处理采样失败的情况
      return -1;
    }

    // 通过逆运动学求解目标关节角
    ArmMotion::IKSolves_t ik_solves;
    if (!kinematic_solver_->IK(cart_point.pose, current_state.position,
                               ik_solves)) {
      // 处理逆解失败的情况
      return -1;
    }

    // 选择最优逆解：过滤不可达解并优先选与当前姿态最接近的解。
    ArmMotion::Joint6Dof_t target_joint;
    if (!SelectBestIKSolve(ik_solves, current_state.position, target_joint)) {
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
    arm_ctrl_.SetJointState(joint_cmd);

    // 更新当前关节状态用于下一步的逆解初值
    current_state.position = target_joint;

    // 等待下一次循环，确保按照 dt_ 的频率执行
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}

int S6h4dInstance::MoveJ(ArmMotion::Joint6Dof_t target_joint, double v,
                         double a, double t) {
  ArmMotion::Joint6Dof_t start_joint;
  JointPath path(start_joint, target_joint);
  CubicTimeScaler time_scaler;
  CubicTimeScaler::PlanConfig config;
  config.T = t;

  if (!time_scaler.Plan(config)) {
    // 处理规划失败的情况
    return -1;
  }

  joint_trajectory_ = std::make_unique<JointTrajectory>(path, time_scaler);
  double duration = joint_trajectory_->GetDuration();

  double current_time = 0.0;
  while (current_time <= (duration + 1e-7)) {
    ArmMotion::JointTrajectoryPoint_t point;
    if (!joint_trajectory_->Sample(current_time, point)) {
      // 处理采样失败的情况
      return -1;
    }

    // 发送控制命令到机械臂
    ArmMotion::JointState_t joint_cmd{.position = point.position,
                                      .velocity = point.velocity,
                                      .accel = point.acceleration};
    arm_ctrl_.SetJointState(joint_cmd);

    // 等待下一次循环，确保按照 dt_ 的频率执行
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_));
    current_time += dt_;
  }
  return 0;
}