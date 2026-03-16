#include <iostream>
#include <iomanip>
#include <cmath>
#include "jacobian.h"
#include <Eigen/Dense>

// 辅助函数：计算两个向量的欧氏距离误差
double calculateError(const std::array<double, 6>& v1, const Eigen::Matrix<double, 6, 1>& v2) {
    double err = 0;
    for(int i=0; i<6; ++i) {
        err += std::pow(v1[i] - v2[i], 2);
    }
    return std::sqrt(err);
}

int main() {
    // 1. 初始化对象 (给定一些合理的连杆长度)
    // L_BS, D_BS, L_AM, L_FA, D_EW, L_WT
    Jacobian6Dof jac(0.2, 0.1, 0.4, 0.35, 0.05, 0.1);

    // 2. 构造一个随机但合理的关节角度 q (弧度)
    ArmMotion::Joint6Dof_t q_current;
    q_current.theta[0] = 0.0;   // 约 28度
    q_current.theta[1] = -0.0;  // 避开直立奇异点
    q_current.theta[2] = 0;
    q_current.theta[3] = 0.0;
    q_current.theta[4] = 0;
    q_current.theta[5] = 0.0;

    // 3. 构造一个极小的关节速度 dq (单位: rad/s)
    // 设 dt = 0.001s，那么我们可以模拟 1ms 内的位移
    Jacobian6Dof::JointVel dq(1, -0.0, 0.0, 0.0, 0.0, -0.0);
    double dt = 0.001;

    // 4. 更新雅可比矩阵
    jac.Update(q_current);

    // 5. 使用雅可比计算预测的末端速度 v = J * dq
    Jacobian6Dof::CartesianVel v_predicted(0,0,0,0,0,0);
    jac.ForwardVelocity(dq, v_predicted);

    // 6. 验证：通过数值法（FK差分）验证
    // 我们需要计算 q_next = q + dq * dt 时的末端位置
    ArmMotion::Joint6Dof_t q_next;
    for(int i=0; i<6; ++i) q_next.theta[i] = q_current.theta[i] + dq.qv[i] * dt;

    // 这里需要你 Jacobian6Dof 内部能暴露 FK 结果，或者我们手动算一下
    // 为了演示，我们假设 Update 执行后，内部存储了末端位置 pe
    // 如果你的 pe 在 Update 后没有存，建议在 Impl 里加一个 getEndEffectorPos()
    
    std::cout << "--- 雅可比速度映射测试 ---" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "预测末端线速度 vx, vy, vz: " 
              << v_predicted.v[0] << ", " << v_predicted.v[1] << ", " << v_predicted.v[2] << std::endl;
    std::cout << "预测末端角速度 wx, wy, wz: " 
              << v_predicted.w[0] << ", " << v_predicted.w[1] << ", " << v_predicted.w[2] << std::endl;

    // 7. 逆映射验证 (DLS)
    // 理论上 dq_reconstructed 应该接近我们初始的 dq
    Jacobian6Dof::JointVel dq_reconstructed(0,0,0,0,0,0);
    jac.InverseVelocity(v_predicted, dq_reconstructed, 0.01);

    std::cout << "\n--- 逆雅可比映射验证 (DLS) ---" << std::endl;
    std::cout << "原始 dq[0-2]: " << dq.qv[0] << ", " << dq.qv[1] << ", " << dq.qv[2] << std::endl;
    std::cout << "逆算 dq[0-2]: " << dq_reconstructed.qv[0] << ", " << dq_reconstructed.qv[1] << ", " << dq_reconstructed.qv[2] << std::endl;

    return 0;
}