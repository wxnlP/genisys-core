#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include "kinematic.h"

// 假设 KinematicSolver 类中的常量和结构体已被正确定义
// const double ArmMotion::RAD_TO_DEG = 57.295777754771045; //
// 假设这是类内部的常量
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

using namespace std;

double wrap_to_pi(double angle) {
  constexpr double kTwoPi = 2.0 * M_PI;
  while (angle > M_PI) {
    angle -= kTwoPi;
  }
  while (angle <= -M_PI) {
    angle += kTwoPi;
  }
  return angle;
}

double joint_distance_sq(const ArmMotion::Joint6Dof& j1,
                         const ArmMotion::Joint6Dof& j2) {
  double sum_sq = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double diff = wrap_to_pi(j1.q[i] - j2.q[i]);
    sum_sq += diff * diff;
  }
  return sum_sq;
}

double solve_score(const ArmMotion::IKSolves& solves, int idx,
                   const ArmMotion::Joint6Dof& ref_joint, bool* is_valid) {
  const char j1_flag = solves.solveFlag[idx][0];
  const char j23_flag = solves.solveFlag[idx][1];
  const char wrist_flag = solves.solveFlag[idx][2];

  if (j1_flag == 0 || j23_flag == 0 || wrist_flag == 0) {
    *is_valid = false;
    return std::numeric_limits<double>::infinity();
  }

  *is_valid = true;
  double score = joint_distance_sq(solves.solves[idx], ref_joint);

  // 与实例中的 SelectBestIKSolve 保持一致：奇异位形可用但轻惩罚。
  if (j1_flag < 0) {
    score += 1e-4;
  }
  if (j23_flag < 0) {
    score += 1e-4;
  }
  if (wrist_flag < 0) {
    score += 1e-4;
  }

  return score;
}

int select_best_ik_solve_index(const ArmMotion::IKSolves& solves,
                               const ArmMotion::Joint6Dof& ref_joint) {
  double best_score = std::numeric_limits<double>::infinity();
  int best_idx = -1;

  for (int i = 0; i < 8; ++i) {
    bool valid = false;
    const double score = solve_score(solves, i, ref_joint, &valid);
    if (valid && score < best_score) {
      best_score = score;
      best_idx = i;
    }
  }

  return best_idx;
}

// --- C++ 版本的 demo_forward_kinematics ---
void demo_forward_kinematics(KinematicSolver& kinematic_solver,
                             ArmMotion::Pose6Dof& outPose,
                             ArmMotion::Joint6Dof& outJoint) {
  cout << "======================================================" << endl;
  cout << "## 1. 正运动学验证 (Forward Kinematics)" << endl;

  // ===== 1. 定义并打印输入角度 =====
  ArmMotion::Joint6Dof initial_theta_deg(17.1887, -22.9183, 11.4592, 5.7296, 90.0000, 28.6479);

  cout << "[INPUT] 关节角输入 (Degree):" << endl;
  cout << "  ";
  for (int i = 0; i < 6; ++i) {
    cout << "J" << i + 1 << ":" << setw(8) << initial_theta_deg.q[i]
         << (i < 5 ? " | " : "");
  }
  cout << endl;

  // 转换为弧度
  ArmMotion::Joint6Dof initial_theta_rad;
  for (int i = 0; i < 6; ++i)
    initial_theta_rad.q[i] = initial_theta_deg.q[i] * DEG2RAD;

  cout << "[INPUT] 关节角输入 (Radian):" << endl;
  cout << "  ";
  for (int i = 0; i < 6; ++i) {
    cout << "J" << i + 1 << ":" << setw(8) << initial_theta_rad.q[i]
         << (i < 5 ? " | " : "");
  }
  cout << "\n" << endl;

  outJoint = initial_theta_rad;

  // ===== 2. 执行 FK 计算 =====
  kinematic_solver.FK(initial_theta_rad, outPose);

  // ===== 3. 打印计算结果 =====
  cout << fixed << setprecision(6);
  cout << "✅ 正运动学计算完成!" << endl;

  // 位置信息
  cout << "\n[RESULT] 末端位置 (Position m):" << endl;
  cout << "  X: " << outPose.x << "  Y: " << outPose.y << "  Z: " << outPose.z
       << endl;

  // 四元数信息
  cout << "\n[RESULT] 末端姿态 (Quaternion x,y,z,w):" << endl;
  cout << "  QX: " << outPose.quat.x << "  QY: " << outPose.quat.y
       << "  QZ: " << outPose.quat.z << "  QW: " << outPose.quat.w << endl;

  // 旋转矩阵信息 (从 outPose.R 恢复)
  // 注意：Eigen 默认列优先，打印时需注意对应关系
  cout << "\n[RESULT] 旋转矩阵 (Rotation Matrix R06):" << endl;
  for (int i = 0; i < 3; ++i) {
    cout << "  [ ";
    for (int j = 0; j < 3; ++j) {
      // outPose.R 存储的是 Matrix3d.data()，默认列优先
      cout << setw(10) << outPose.R[i + j * 3] << " ";
    }
    cout << "]" << endl;
  }

  cout << "======================================================" << endl;
}

// --- C++ 版本的 demo_inverse_kinematics ---
void demo_inverse_kinematics(KinematicSolver& kinematic_solver,
                             const ArmMotion::Pose6Dof& pose,
                             const ArmMotion::Joint6Dof& lastJoint_rad) {
  cout << "## 2. 逆运动学验证 (Inverse Kinematics)" << endl;
  cout << fixed << setprecision(4);

  // 关键点：创建一个副本并强制设置使用四元数进行计算
  ArmMotion::Pose6Dof inputPose = pose;
  inputPose.vaildR = false;  // 告诉 IK 使用 qx, qy, qz, qw 而不是 R 矩阵

  cout << "* 输入末端位置 (m): (" << inputPose.x << ", " << inputPose.y << ", "
       << inputPose.z << ")" << endl;
  cout << "* 输入末端姿态 四元数: (" << inputPose.quat.x << ", "
       << inputPose.quat.y << ", " << inputPose.quat.z << ", "
       << inputPose.quat.w << ")" << endl;

  ArmMotion::IKSolves outputSolves;

  // 调用 IK（此时内部会执行 P_06_quat.toRotationMatrix()）
  kinematic_solver.IK(inputPose, lastJoint_rad, outputSolves);

  cout << "\n✅ 逆运动学解 (基于四元数输入):" << endl;

  for (int i = 0; i < 8; ++i) {
    bool valid = false;
    const double score = solve_score(outputSolves, i, lastJoint_rad, &valid);

    cout << "解 " << i + 1 << ": ";

    for (int j = 0; j < 6; ++j) {
      double deg = outputSolves.solves[i].q[j] * RAD2DEG;
      cout << setw(9) << deg << " ";
    }
    cout << " | flag=[" << static_cast<int>(outputSolves.solveFlag[i][0]) << ","
         << static_cast<int>(outputSolves.solveFlag[i][1]) << ","
         << static_cast<int>(outputSolves.solveFlag[i][2]) << "]";

    if (valid) {
      cout << " | score=" << score;
    } else {
      cout << " | score=invalid";
    }
    cout << endl;
  }

  const int best_idx = select_best_ik_solve_index(outputSolves, lastJoint_rad);

  if (best_idx >= 0) {
    cout << "\n* 寻优结果: 最接近输入的解为第 " << best_idx + 1 << " 组"
         << endl;
  }
  cout << "\n---" << endl;
}

int main() {

  ArmMotion::MDHParams mdh_params = {
    .alpha = {0.0, -M_PI_2, 0.0, 0.0, -M_PI_2, M_PI_2},  // alpha
    .a = {0.0, 0.02, 0.26, 0.24668, 0.075, 0.0},             // a
    .d = {0.1207, 0.0, 0.0, 0.0, 0.0, 0.0265},       // d
    .theta = {0.0, -M_PI, (M_PI - 0.233176), 0.233176, M_PI_2, 0.0}          // theta (初始偏移量)
  };

  // ArmMotion::MDHParams mdh_params = {
  //   .alpha = {0.0, -M_PI_2, 0.0, -M_PI_2, M_PI_2, -M_PI_2},  // alpha
  //   .a = {0.0, 0.04, 0.2, 0.059, 0.0, 0.0},             // a
  //   .d = {0.123, 0.0, 0.0, 0.2, 0.0, 0.104},       // d
  //   .theta = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0}          // theta (初始偏移量)
  // };

  KinematicSolver kSolver(mdh_params);
  kSolver.SetIKMode(KinematicSolver::IKMode::ParallelWrist);
  // kSolver.SetIKMode(KinematicSolver::IKMode::SphericalWrist);

  ArmMotion::Joint6Dof joint;
  ArmMotion::Pose6Dof pose;

  demo_forward_kinematics(kSolver, pose, joint);
  demo_inverse_kinematics(kSolver, pose, joint);

  return 0;
}