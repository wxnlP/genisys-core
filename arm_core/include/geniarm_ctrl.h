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
    bool SetJointState(const ArmMotion::JointCommand_t& joint_cmd) override;
    bool GetJointState(ArmMotion::JointState_t& joint_state) override;
    bool SetCtrlMode(DmMotorDriver::CtrlMode mode);

private:
    std::vector<DmMotorDriver>& motor_driver_;
    std::mutex state_mutex_;
    std::jthread state_update_thread_;

    ArmMotion::JointState_t latest_joint_state_{};
};