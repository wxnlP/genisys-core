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
    cout << "======================================================" << endl;
    cout << "## 1. 正运动学验证 (Forward Kinematics)" << endl;

    // ===== 1. 定义并打印输入角度 =====
    ArmMotion::Joint6Dof_t initial_theta_deg(1.88, -90.0, 33.33, 11.98, -89.9, 9.9);

    cout << "[INPUT] 关节角输入 (Degree):" << endl;
    cout << "  ";
    for (int i = 0; i < 6; ++i) {
        cout << "J" << i+1 << ":" << setw(8) << initial_theta_deg.theta[i] << (i < 5 ? " | " : "");
    }
    cout << endl;

    // 转换为弧度
    ArmMotion::Joint6Dof_t initial_theta_rad;
    for (int i = 0; i < 6; ++i)
        initial_theta_rad.theta[i] = initial_theta_deg.theta[i] * DEG2RAD;

    cout << "[INPUT] 关节角输入 (Radian):" << endl;
    cout << "  ";
    for (int i = 0; i < 6; ++i) {
        cout << "J" << i+1 << ":" << setw(8) << initial_theta_rad.theta[i] << (i < 5 ? " | " : "");
    }
    cout << "\n" << endl;

    outJoint = initial_theta_rad;

    // ===== 2. 执行 FK 计算 =====
    bool success = kinematic_solver.FK(initial_theta_rad, outPose);
    if (!success) {
        cout << "❌ 正运动学计算失败!" << endl;
        return;
    }

    // ===== 3. 打印计算结果 =====
    cout << fixed << setprecision(6);
    cout << "✅ 正运动学计算成功!" << endl;
    
    // 位置信息
    cout << "\n[RESULT] 末端位置 (Position m):" << endl;
    cout << "  X: " << outPose.x << "  Y: " << outPose.y << "  Z: " << outPose.z << endl;

    // 四元数信息
    cout << "\n[RESULT] 末端姿态 (Quaternion x,y,z,w):" << endl;
    cout << "  QX: " << outPose.qx << "  QY: " << outPose.qy 
         << "  QZ: " << outPose.qz << "  QW: " << outPose.qw << endl;

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
                             const ArmMotion::Pose6Dof_t& pose, 
                             const ArmMotion::Joint6Dof_t& lastJoint_rad) 
{
    cout << "## 2. 逆运动学验证 (Inverse Kinematics)" << endl;
    cout << fixed << setprecision(4);

    // 关键点：创建一个副本并强制设置使用四元数进行计算
    ArmMotion::Pose6Dof_t inputPose = pose;
    inputPose.vaildR = false; // 告诉 IK 使用 qx, qy, qz, qw 而不是 R 矩阵

    cout << "* 输入末端位置 (m): (" << inputPose.x << ", " << inputPose.y << ", " << inputPose.z << ")" << endl;
    cout << "* 输入末端姿态 四元数: (" 
         << inputPose.qx << ", " << inputPose.qy << ", " 
         << inputPose.qz << ", " << inputPose.qw << ")" << endl;

    ArmMotion::IKSolves_t outputSolves;
    
    // 调用 IK（此时内部会执行 P_06_quat.toRotationMatrix()）
    kinematic_solver.IK(inputPose, lastJoint_rad, outputSolves);

    cout << "\n✅ 逆运动学解 (基于四元数输入):" << endl;

    double min_diff = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (int i = 0; i < 8; ++i) {
        // 检查该索引是否有解（对应你代码中的 solveFlag）
        // 这里简化处理，直接打印所有解并寻找最接近上一时刻的解
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
        cout << "\n* 寻优结果: 最接近输入的解为第 " << best_idx + 1 << " 组" << endl;
    }
    cout << "\n---" << endl;
}


int main()
{
    KinematicSolver kSolver(0.123, 0.04, 0.20, 0.059, 0.2, 0.104);

    ArmMotion::Joint6Dof_t joint;
    ArmMotion::Pose6Dof_t pose;

    // bool ret = kSolver.FK(joint, pose);
    // double ret = sinf(1.5708f);
    // std::cout << ret << std::endl;

    demo_forward_kinematics(kSolver, pose, joint);
    demo_inverse_kinematics(kSolver, pose, joint);


    return 0;
}