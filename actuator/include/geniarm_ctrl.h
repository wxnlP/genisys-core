#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include "arm_ctrl_base.h"
#include "dm_motor_driver.h"


class GeniArmCtrl : public ArmCtrlBase
{
public:
    explicit GeniArmCtrl(std::vector<DmMotorDriver>& motor_driver);
    ~GeniArmCtrl() override;

    bool Init() override;
    void Enable() override;
    void Disable() override;
    void Shutdown() override;
    bool SetJointState(const ArmCtrlBase::JointCommand& joint_cmd) override;
    bool GetJointState(ArmCtrlBase::JointState& joint_state) override;
    bool SetCtrlMode(ArmCtrlBase::CtrlMode mode) override;

private:
    std::vector<DmMotorDriver>& motor_driver_;
    std::mutex state_mutex_;
    std::jthread state_update_thread_;

    ArmCtrlBase::JointState latest_joint_state_{};
};