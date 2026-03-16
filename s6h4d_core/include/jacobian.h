#pragma once

#include <memory>
#include "arm_type.h"

class Jacobian6Dof {
public: 
    struct CartesianVel {
        CartesianVel() = default;
        CartesianVel(double v1, double v2, double v3, double w1, double w2, double w3)
            : v{ v1, v2, v3 }, w{ w1, w2, w3 } {}
        double v[3];
        double w[3];
    };

    struct JointVel {
        JointVel() = default;
        JointVel(double q1, double q2, double q3, double q4, double q5, double q6)
            : qv{ q1, q2, q3, q4, q5, q6 } {}
        double qv[6];
    };
    
    Jacobian6Dof(double L_BS, double D_BS, double L_AM, double L_FA, double D_EW, double L_WT);
    ~Jacobian6Dof();
    Jacobian6Dof(const Jacobian6Dof&) = delete;
    Jacobian6Dof& operator=(const Jacobian6Dof&) = delete;

    void Update(const ArmMotion::Joint6Dof_t &inputJoint);
    bool ForwardVelocity(const JointVel &inputVel, CartesianVel &outputVel);
    bool InverseVelocity(const CartesianVel &inputVel, JointVel &outputVel, double damping);
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    // 记录机械角度偏移
    double jointOffset_[6];
};