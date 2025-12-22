#include "s6h4d_interface.h"
#include "kinematic.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <numeric>

// 假设 KinematicSolver 类中的常量和结构体已被正确定义
// const float KinematicSolver::RAD_TO_DEG = 57.295777754771045; // 假设这是类内部的常量

using namespace std;

// --- 辅助函数：将 Joint6Dof_t 结构体转换为 std::vector<float>
std::vector<float> joint_to_vector(const KinematicSolver::Joint6Dof_t& joint) {
    return {
        joint.theta[0], joint.theta[1], joint.theta[2],
        joint.theta[3], joint.theta[4], joint.theta[5]
    };
}

// --- 辅助函数：计算两个 Joint6Dof_t 之间的差值平方和
float calculate_diff_sq(const KinematicSolver::Joint6Dof_t& j1, const KinematicSolver::Joint6Dof_t& j2) {
    float sum_sq = 0.0f;
    for (int i = 0; i < 6; ++i) {
        float diff = j1.theta[i] - j2.theta[i];
        sum_sq += diff * diff;
    }
    return sum_sq;
}

// --- C++ 版本的 demo_forward_kinematics ---
void demo_forward_kinematics(KinematicSolver& kinematic_solver, 
                             KinematicSolver::Pose6Dof_t& outPose, 
                             KinematicSolver::Joint6Dof_t& outJoint) 
{
    cout << "## 1. 正运动学验证 (Forward Kinematics)" << endl;

    // 设定的关节角 (度)
    // C++ 结构体初始化可以直接使用列表初始化
    KinematicSolver::Joint6Dof_t initial_theta(10.0f, 0.0f, 1.0f, 20.0f, -20.5f, 10.9f);
    outJoint = initial_theta; // 将初始关节角存入 outJoint

    cout << "* 初始关节角 (Deg): [";
    for(int i = 0; i < 6; ++i) {
        cout << initial_theta.theta[i] << (i < 5 ? ", " : "");
    }
    cout << "]" << endl;

    // 进行正运动学解算
    bool success = kinematic_solver.FK(initial_theta, outPose);

    if (!success) {
        cout << "\n❌ 正运动学计算失败!" << endl;
        return;
    }

    // 结果输出
    cout << "\n✅ 正运动学计算结果:" << endl;
    
    // 设置输出精度
    cout << fixed << setprecision(4);

    cout << "  - 末端位置 P(x, y, z) (m): ("
         << outPose.x << ", " << outPose.y << ", " << outPose.z << ")" << endl;
    cout << "  - 末端姿态 Euler(roll, pitch, yaw) (Deg): ("
         << outPose.roll << ", " << outPose.pitch << ", " << outPose.yaw << ")" << endl;
    
    // 假设 roll/pitch/yaw 对应 Python 中的 z/y/x 欧拉角
    
    cout << "\n---" << endl;
}

// --- C++ 版本的 demo_inverse_kinematics ---
void demo_inverse_kinematics(KinematicSolver& kinematic_solver, 
                             const KinematicSolver::Pose6Dof_t& pose, 
                             const KinematicSolver::Joint6Dof_t& lastJoint) 
{
    cout << "## 2. 逆运动学验证 (Inverse Kinematics)" << endl;

    // 设置输出精度
    cout << fixed << setprecision(4);
    
    // 输入位姿
    cout << "* 输入末端位置 P(x, y, z) (m): ("
         << pose.x << ", " << pose.y << ", " << pose.z << ")" << endl;
    cout << "* 输入末端姿态 Euler(roll, pitch, yaw) (Deg): ("
         << pose.roll << ", " << pose.pitch << ", " << pose.yaw << ")" << endl;
    
    cout << "* 上次关节角 (用于解的选择/奇异点处理) (Deg): [";
    for(int i = 0; i < 6; ++i) {
        cout << lastJoint.theta[i] << (i < 5 ? ", " : "");
    }
    cout << "]" << endl;
    
    // 将上次关节角转换为弧度，用于 IK 函数（假设 KinematicSolver::RAD_TO_DEG 可访问）
    KinematicSolver::Joint6Dof_t lastJoint_rad;
    const float RAD_TO_DEG_VAL = 57.295777754771045f; // 实际应从类中获取
    for (int i = 0; i < 6; ++i) {
        lastJoint_rad.theta[i] = lastJoint.theta[i] / RAD_TO_DEG_VAL;
    }

    // 进行逆运动学解算
    KinematicSolver::IKSolves_t outputSolves;
    kinematic_solver.IK(pose, lastJoint_rad, outputSolves);
    
    cout << "\n✅ 逆运动学计算结果 (8组解):" << endl;

    // 表格头
    cout << setfill('-') << setw(115) << "" << endl;
    cout << setfill(' ');
    cout << setw(2) << "#" << " "
         << setw(8) << "J1 Flag" << " "
         << setw(8) << "J23 Flag" << " "
         << setw(10) << "J456 Flag" << " | "
         << setw(10) << "J1 (Deg)" << " "
         << setw(10) << "J2 (Deg)" << " "
         << setw(10) << "J3 (Deg)" << " "
         << setw(10) << "J4 (Deg)" << " "
         << setw(10) << "J5 (Deg)" << " "
         << setw(10) << "J6 (Deg)" << endl;
    cout << setfill('-') << setw(115) << "" << endl;
    cout << setfill(' ');

    // 查找与输入关节角最接近的一组解
    float min_diff_sq = std::numeric_limits<float>::max();
    int best_match_idx = -1;

    for (int i = 0; i < 8; ++i) {
        // 标志位输出
        cout << setw(2) << i + 1 << " ";
        for (int j = 0; j < 3; ++j) {
            cout << setw(8) << static_cast<int>(outputSolves.solveFlag[i][j]) << " ";
        }
        cout << "| ";

        // 关节角 (度)
        float diff_sq = 0.0f;
        for (int j = 0; j < 6; ++j) {
            float angle_deg = outputSolves.solves[i].theta[j];
            cout << setw(10) << angle_deg << " ";
            
            // 计算差值平方和
            float diff = angle_deg - lastJoint.theta[j];
            diff_sq += diff * diff;
        }

        // 记录最佳匹配
        if (diff_sq < min_diff_sq) {
            min_diff_sq = diff_sq;
            best_match_idx = i;
        }

        cout << endl;
    }

    cout << setfill('-') << setw(115) << "" << endl;
    cout << setfill(' ');
    
    if (best_match_idx != -1) {
        cout << "\n* 最佳匹配解 (与输入关节角最接近): 第 " << best_match_idx + 1 << " 组解" << endl;
        cout << "* 最佳解关节角 (Deg): [";
        for(int i = 0; i < 6; ++i) {
            cout << outputSolves.solves[best_match_idx].theta[i] << (i < 5 ? ", " : "");
        }
        cout << "]" << endl;
    } else {
        cout << "\n* 未找到有效解。" << endl;
    }

    cout << "\n---" << endl;
}

int main()
{
    KinematicSolver kSolver(0.23f, 0.05f, 0.30f, 0.10f, 0.35f, 0.10f);

    KinematicSolver::Joint6Dof_t joint;
    KinematicSolver::Pose6Dof_t pose;

    // bool ret = kSolver.FK(joint, pose);
    // double ret = sinf(1.5708f);
    // std::cout << ret << std::endl;

    demo_forward_kinematics(kSolver, pose, joint);
    demo_inverse_kinematics(kSolver, pose, joint);


    return 0;
}