#pragma once

#include <memory>
#include "arm_type.h"

class Jacobian6Dof {
public: 
    Jacobian6Dof(double L_BS, double D_BS, double L_AM, double L_FA, double D_EW, double L_WT);
    ~Jacobian6Dof();


    void Update(const ArmMotion::Joint6Dof_t &inputJoint, double dt);
    bool ForwardVelocity(const ArmMotion::Joint6Dof_t &inputVel, ArmMotion::CartesianTwist_t &outputVel);
    bool InverseVelocity(const ArmMotion::CartesianTwist_t &inputVel, ArmMotion::Joint6Dof_t &outputVel, double damping);
    bool ForwardAccel(const ArmMotion::Joint6Dof_t &inputAcc, ArmMotion::CartesianTwist_t &outputAcc);
    bool InverseAccel(const ArmMotion::CartesianTwist_t &inputAcc, ArmMotion::Joint6Dof_t &outputAcc, double damping);
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    // 记录机械角度偏移
    double jointOffset_[6];
};