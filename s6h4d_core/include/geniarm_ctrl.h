#pragma once

#include "arm_ctrl_base.h"
#include <vector>
#include "dm_motor_driver.h"


class GeniArmCtrl : public ArmCtrlBase
{
public:
    explicit GeniArmCtrl(std::vector<DmMotorDriver>& motor_driver);
    ~GeniArmCtrl() override;

    bool Init() override;
    void Enable() override;
    void Disable() override;
    bool SendJointStateRequest();
    bool SetJointState(const ArmMotion::JointCommand_t& joint_cmd) override;
    bool GetJointState(ArmMotion::JointState_t& joint_state) override;

private:
    std::vector<DmMotorDriver>& motor_driver_;
};