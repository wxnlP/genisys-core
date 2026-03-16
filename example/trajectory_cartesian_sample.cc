#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cfloat>
#include "trajectory_cartesian_planner.h"
#include "kinematic.h"

// bool SelectBestIKSolve(const ArmMotion::IKSolves_t& ikSolves,
//                        const ArmMotion::Joint6Dof_t& qCurr,
//                        ArmMotion::Joint6Dof_t& bestSolve) {
//   double minCostA = DBL_MAX;
//   double minCostB = DBL_MAX;
//   int bestA = -1;
//   int bestB = -1;

//   for (int i = 0; i < 8; ++i) {
//     bool hasZero = false;
//     bool hasSingular = false;

//     // 1️⃣ 检查标志
//     for (int j = 0; j < 3; ++j) {
//       if (ikSolves.solveFlag[i][j] == 0) {
//         hasZero = true;
//         break;
//       }
//       if (ikSolves.solveFlag[i][j] == -1) {
//         hasSingular = true;
//       }
//     }

//     // 不可达，直接跳过
//     if (hasZero) continue;

//     // 2️⃣ 计算关节空间距离
//     double cost = 0.0;
//     for (int k = 0; k < 6; ++k) {
//       double diff = ikSolves.solves[i].theta[k] - qCurr.theta[k];
//       cost += diff * diff;
//     }

//     // 3️⃣ 分类记录
//     if (!hasSingular) {
//       // A 类：全 1
//       if (cost < minCostA) {
//         minCostA = cost;
//         bestA = i;
//       }
//     } else {
//       // B 类：奇异解
//       if (cost < minCostB) {
//         minCostB = cost;
//         bestB = i;
//       }
//     }
//   }

//   // 4️⃣ 优先返回 A 类，其次 B 类
//   if (bestA >= 0) {
//     bestSolve = ikSolves.solves[bestA];
//     return true;
//   }

//   if (bestB >= 0) {
//     bestSolve = ikSolves.solves[bestB];
//     return true;
//   }

//   // 所有解都不可用
//   return false;
// }

// bool SelectBestIKSolve(const ArmMotion::IKSolves_t& ikSolves,
//                        const ArmMotion::Joint6Dof_t& qCurr,
//                        ArmMotion::Joint6Dof_t& bestSolve) {
//   double minCost = DBL_MAX;
//   int bestIdx = -1;

//   for (int i = 0; i < 8; ++i) {
//     // 检查解的有效性
//     bool valid = true;
//     for (int j = 0; j < 3; ++j) {
//       if (ikSolves.solveFlag[i][j] <= 0) {
//         valid = false;
//         break;
//       }
//     }
//     if (!valid) continue;

//     // 计算与当前关节角的距离
//     double cost = 0.0;
//     for (int k = 0; k < 6; ++k) {
//       double diff = ikSolves.solves[i].theta[k] - qCurr.theta[k];
      
//       // 处理角度周期性：将差值包裹到 [-PI, PI] 区间
//       while (diff > M_PI) diff -= 2 * M_PI;
//       while (diff < -M_PI) diff += 2 * M_PI;
      
//       cost += diff * diff;
//     }

//     // 选择最小成本的解
//     if (cost < minCost) {
//       minCost = cost;
//       bestIdx = i;
//     }
//   }

//   // 没有有效解
//   if (bestIdx == -1) return false;

//   bestSolve = ikSolves.solves[bestIdx];
//   return true;
// }

bool SelectBestIKSolve(const ArmMotion::IKSolves_t& ikSolves,
                       const ArmMotion::Joint6Dof_t& qCurr,
                       ArmMotion::Joint6Dof_t& bestSolve) {
  double minCost = DBL_MAX;
  int bestIdx = -1;

  // 记录最小差值，用于最后修正
  double bestDiff[6] = {0};

  for (int i = 0; i < 8; ++i) {
    bool valid = true;
    for (int j = 0; j < 3; ++j) {
      if (ikSolves.solveFlag[i][j] <= 0) {
        valid = false;
        break;
      }
    }
    if (!valid) continue;

    double cost = 0.0;
    double diffArr[6];

    for (int k = 0; k < 6; ++k) {
      double diff = ikSolves.solves[i].theta[k] - qCurr.theta[k];

      while (diff > M_PI) diff -= 2 * M_PI;
      while (diff < -M_PI) diff += 2 * M_PI;

      diffArr[k] = diff;
      cost += diff * diff;
    }

    if (cost < minCost) {
      minCost = cost;
      bestIdx = i;
      memcpy(bestDiff, diffArr, sizeof(diffArr));
    }
  }

  if (bestIdx == -1) return false;

  // 修正输出角度为“连续值”
  for (int k = 0; k < 6; ++k) {
    bestSolve.theta[k] = qCurr.theta[k] + bestDiff[k];
  }

  return true;
}


int main()
{
    /* ================= 参数设置 ================= */
    const double dt = 0.01;   // 10 ms 采样周期
    // 注意：总时间 T 由路径点的最后一个时间决定，这里不再硬编码 T

    /* ================= 4点 笛卡尔路径设计 ================= */
    // std::vector<ArmMotion::Pose6Dof_t> path;
    // std::vector<double> time;

    // // 第1点：起点
    // ArmMotion::Pose6Dof_t p0{};
    // p0.x = 0.4;  p0.y = -0.1;  p0.z = 0.4;
    // p0.qw = 1.0; p0.qx = 0.0;  p0.qy = 0.0;  p0.qz = 0.0;
    // path.push_back(p0);
    // time.push_back(0.0);

    // // 第2点：沿 Y 轴移动
    // ArmMotion::Pose6Dof_t p1 = p0;
    // p1.y = 0.2;
    // path.push_back(p1);
    // time.push_back(2.0);

    // // 第3点：在保持位置的同时旋转姿态（yaw 180°）
    // ArmMotion::Pose6Dof_t p2 = p1;
    // // 这里用四元数表示 yaw 180° 旋转
    // // yaw 180° 对应四元数：qw=0, qx=0, qy=1, qz=0（绕 z 轴旋转 180°）
    // p2.qw = 0.0;
    // p2.qx = 0.0;
    // p2.qy = 1.0;
    // p2.qz = 0.0;
    // path.push_back(p2);
    // time.push_back(4.0);

    // // 第4点：沿 Z 轴上升，并保持 yaw 180°
    // ArmMotion::Pose6Dof_t p3 = p2;
    // p3.z = 0.5;
    // path.push_back(p3);
    // time.push_back(6.0);
    // // 我们设计一个在 Y-Z 平面上的矩形轨迹
    // // P0 -> P1 -> P2 -> P3
    std::vector<ArmMotion::Pose6Dof_t> path;
    std::vector<double> time;

    // --- 第1点 (起步点) ---
    // 位置: x=0.4, y=-0.1, z=0.4
    ArmMotion::Pose6Dof_t p0{};
    p0.x = 0.4;  p0.y = -0.1;  p0.z = 0.4;
    p0.quat.w = 1.0; p0.quat.x = 0.0; p0.quat.y = 0.0; p0.quat.z = 0.0; // 姿态保持不变，便于观察位置
    path.push_back(p0);
    time.push_back(0.0); // t=0s

    // --- 第2点 (沿 Y 轴移动) ---
    // 位置: x=0.4, y=0.2, z=0.4 (向左平移 30cm)
    ArmMotion::Pose6Dof_t p1 = p0;
    p1.y = 0.2; 
    path.push_back(p1);
    time.push_back(2.0); // t=2s

    // --- 第3点 (沿 Z 轴上升) ---
    // 位置: x=0.4, y=0.2, z=0.5 (向上平移 10cm)
    ArmMotion::Pose6Dof_t p2 = p1;
    p2.z = 0.5;
    path.push_back(p2);
    time.push_back(4.0); // t=4s

    // --- 第4点 (沿 Y 轴返回) ---
    // 位置: x=0.4, y=-0.1, z=0.5 (向右平移 30cm)
    ArmMotion::Pose6Dof_t p3 = p2;
    p3.y = -0.1;
    path.push_back(p3);
    time.push_back(6.0); // t=6s

    double total_duration = time.back(); // 总时长 6.0s

    /* ================= 轨迹规划器 ================= */
    // 这里的 accel 限制可能需要根据你的实际规划器逻辑调整
    // 如果想要平滑效果，记得应用前面讨论的 Quintic Scaling
    CartesianTrajectoryPlanner cartPlanner(0.1);
    
    if (!cartPlanner.Plan(path, time)) {
        std::cerr << "Cartesian trajectory plan failed\n";
        return -1;
    }
    std::cout << "Trajectory Plan Success! Total points: " << path.size() << "\n";

    /* ================= 运动学初始化 ================= */
    KinematicSolver ik(0.123, 0.04, 0.20, 0.059, 0.2, 0.104);

    /* ================= 初始姿态 IK ================= */
    ArmMotion::Joint6Dof_t q_last{};
    // 给定一个合理的初始猜测，避免第一次解就算错
    for (int i = 0; i < 6; ++i) q_last.theta[i] = 0.0; 
    q_last.theta[1] = -0.5; // 稍微给点角度，避开奇异

    CartesianTrajectoryPlanner::CartesianPoint_t cp0;
    cartPlanner.Sample(time.front(), cp0);

    ArmMotion::Pose6Dof_t pose0{};
    pose0.x  = cp0.x.pos; 
    pose0.y  = cp0.y.pos; 
    pose0.z  = cp0.z.pos;
    pose0.quat.w = cp0.q.w;    
    pose0.quat.x = cp0.q.x;    
    pose0.quat.y = cp0.q.y;    
    pose0.quat.z = cp0.q.z;

    ArmMotion::IKSolves_t ik0;
    if (!ik.IK(pose0, q_last, ik0)) {
        std::cerr << "Initial IK failed at Start Point\n";
        return -1;
    }

    ArmMotion::Joint6Dof_t q0;
    if (!SelectBestIKSolve(ik0, q_last, q0)) {
        std::cerr << "Initial IK selection failed\n";
        return -1;
    }
    q_last = q0; // 更新为计算出的真实初始角

    /* ================= CSV 输出 ================= */
    std::ofstream jointCsv("joint_pos.csv");
    std::ofstream cartCsv("cartesian.csv");
    // 增加 CSV 头部信息，方便绘图
    jointCsv << "t,q1,q2,q3,q4,q5,q6\n";
    cartCsv  << "t,x,y,z,vx,vy,vz,qw,qx,qy,qz\n"; // 增加了速度列 vx,vy,vz 方便调试

    std::cout << "Start Sampling...\n";

    /* ================= 主循环 ================= */
    // 循环直到 total_duration
    for (double t = 0.0; t <= total_duration + 1e-4; t += dt) {

        CartesianTrajectoryPlanner::CartesianPoint_t cp;
        
        // 采样轨迹
        bool sample_ret = cartPlanner.Sample(t, cp);
        if (!sample_ret) {
             std::cerr << "Sample failed at t=" << t << "\n";
             break;
        }

        ArmMotion::Pose6Dof_t pose{};
        pose.x  = cp.x.pos; 
        pose.y  = cp.y.pos; 
        pose.z  = cp.z.pos;
        pose.quat.w = cp.q.w;
        pose.quat.x = cp.q.x;
        pose.quat.y = cp.q.y;
        pose.quat.z = cp.q.z;

        // IK 解算
        ArmMotion::IKSolves_t ikSolves;
        if (!ik.IK(pose, q_last, ikSolves)) {
            std::cerr << "IK failed at t=" << t << " (Singularity or Out of Workspace)\n";
            // 这里不退出，而是尝试继续（实际控制中应该报错停止）
            continue; 
        }

        ArmMotion::Joint6Dof_t q_now;
        if (!SelectBestIKSolve(ikSolves, q_last, q_now)) {
            std::cerr << "No valid IK solution at t=" << t << "\n";
            continue;
        }

        // 写入关节数据
        jointCsv << std::fixed << std::setprecision(5) << t;
        for (int i = 0; i < 6; ++i) {
            jointCsv << "," << q_now.theta[i];
        }
        jointCsv << "\n";

        // 写入笛卡尔数据 (包含速度以便观察平滑度)
        cartCsv << std::fixed << std::setprecision(5)
                << t << ","
                << pose.x << "," << pose.y << "," << pose.z << ","
                << cp.x.vel << "," << cp.y.vel << "," << cp.z.vel << "," // 记录速度
                << pose.quat.w << "," << pose.quat.x << "," << pose.quat.y 
                << "," << pose.quat.z << "\n";

        q_last = q_now;
    }

    std::cout << "Test finished.\n";
    std::cout << "Data saved to joint_pos.csv and cartesian.csv\n";

    return 0;
}
