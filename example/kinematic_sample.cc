#include "s6h4d_interface.h"
#include "kinematic.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <numeric>

// 假设 KinematicSolver 类中的常量和结构体已被正确定义
// const double ArmMotion::RAD_TO_DEG = 57.295777754771045; // 假设这是类内部的常量
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

using namespace std;

// --- 辅助函数：将 Joint6Dof_t 结构体转换为 std::vector<double>
std::vector<double> joint_to_vector(const ArmMotion::Joint6Dof_t& joint) {
    return {
        joint.theta[0], joint.theta[1], joint.theta[2],
        joint.theta[3], joint.theta[4], joint.theta[5]
    };
}

// --- 辅助函数：计算两个 Joint6Dof_t 之间的差值平方和
double calculate_diff_sq(const ArmMotion::Joint6Dof_t& j1, const ArmMotion::Joint6Dof_t& j2) {
    double sum_sq = 0.0f;
    for (int i = 0; i < 6; ++i) {
        double diff = j1.theta[i] - j2.theta[i];
        sum_sq += diff * diff;
    }
    return sum_sq;
}

// --- C++ 版本的 demo_forward_kinematics ---
void demo_forward_kinematics(KinematicSolver& kinematic_solver, 
                             ArmMotion::Pose6Dof_t& outPose, 
                             ArmMotion::Joint6Dof_t& outJoint) 
{
    cout << "## 1. 正运动学验证 (Forward Kinematics)" << endl;

    // ===== 输入关节角（Deg → Rad）=====
    ArmMotion::Joint6Dof_t initial_theta_deg(
        1.88, -90.0, 33.33, 11.98, -89.9, 9.9
    );

    ArmMotion::Joint6Dof_t initial_theta_rad;
    for (int i = 0; i < 6; ++i)
        initial_theta_rad.theta[i] = initial_theta_deg.theta[i] * DEG2RAD;

    outJoint = initial_theta_rad;

    cout << "* 初始关节角 (Deg): [";
    for (int i = 0; i < 6; ++i)
        cout << initial_theta_deg.theta[i] << (i < 5 ? ", " : "");
    cout << "]" << endl;

    // ===== FK =====
    bool success = kinematic_solver.FK(initial_theta_rad, outPose);
    if (!success) {
        cout << "❌ 正运动学计算失败!" << endl;
        return;
    }

    cout << fixed << setprecision(4);
    cout << "\n✅ 正运动学结果:" << endl;

    cout << "  - 末端位置 (m): ("
         << outPose.x << ", " << outPose.y << ", " << outPose.z << ")" << endl;

    cout << "  - 末端姿态 Euler (Deg): ("
         << outPose.roll  * RAD2DEG << ", "
         << outPose.pitch * RAD2DEG << ", "
         << outPose.yaw   * RAD2DEG << ")" << endl;

    cout << "\n---" << endl;
}


// --- C++ 版本的 demo_inverse_kinematics ---
void demo_inverse_kinematics(KinematicSolver& kinematic_solver, 
                             const ArmMotion::Pose6Dof_t& pose, 
                             const ArmMotion::Joint6Dof_t& lastJoint_rad) 
{
    cout << "## 2. 逆运动学验证 (Inverse Kinematics)" << endl;
    cout << fixed << setprecision(4);

    cout << "* 输入末端位置 (m): ("
         << pose.x << ", " << pose.y << ", " << pose.z << ")" << endl;

    cout << "* 输入末端姿态 Euler (Deg): ("
         << pose.roll  * RAD2DEG << ", "
         << pose.pitch * RAD2DEG << ", "
         << pose.yaw   * RAD2DEG << ")" << endl;

    cout << "* 上次关节角 (Deg): [";
    for (int i = 0; i < 6; ++i)
        cout << lastJoint_rad.theta[i] * RAD2DEG << (i < 5 ? ", " : "");
    cout << "]" << endl;

    ArmMotion::IKSolves_t outputSolves;
    kinematic_solver.IK(pose, lastJoint_rad, outputSolves);

    cout << "\n✅ 逆运动学解 (8组):" << endl;

    double min_diff = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (int i = 0; i < 8; ++i) {
        double diff = 0.0;
        cout << "解 " << i + 1 << ": ";

        for (int j = 0; j < 6; ++j) {
            double deg = outputSolves.solves[i].theta[j] * RAD2DEG;
            cout << setw(9) << deg << " ";
            double d = outputSolves.solves[i].theta[j] - lastJoint_rad.theta[j];
            diff += d * d;
        }

        cout << endl;

        if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        cout << "\n* 最优解: 第 " << best_idx + 1 << " 组" << endl;
    }

    cout << "\n---" << endl;
}


int main()
{
    KinematicSolver kSolver(0.23, 0.05, 0.30, 0.10, 0.35, 0.10);

    ArmMotion::Joint6Dof_t joint;
    ArmMotion::Pose6Dof_t pose;

    // bool ret = kSolver.FK(joint, pose);
    // double ret = sinf(1.5708f);
    // std::cout << ret << std::endl;

    demo_forward_kinematics(kSolver, pose, joint);
    demo_inverse_kinematics(kSolver, pose, joint);


    return 0;
}