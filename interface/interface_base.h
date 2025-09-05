/**
 * angle: °
 * vel: °/s
 * accel: °/s″
 * 
 */

#pragma once

#include "common/common_inc.h"
#include <cstddef>

#define JOINT_MAX   6


class InterfaceBase {
public:
    InterfaceBase() = default;

    /* 功能实现 */
    void Reset();
    JointAngle GetJointAngle();
    void ArmPosition(uint8_t paw);
    void ArmPosition(const JointAngle & joint);
    void ArmPosition(const JointAngle & joint, uint8_t paw);
    void ArmPosition(const S6h4dTxMsg & msg);
    void ArmPosition(uint8_t jointIndex, float angle, float vel, float accel);

    /* 初始化 */
    virtual void Init() = 0;
    virtual void DeInit() = 0;

    /* 基础通信 */
    virtual size_t WriteBytes(uint8_t *bytes, size_t size) = 0;
    virtual size_t ReadBytes(uint8_t *bytes, size_t size) = 0;

private:
    void Float2Bytes(uint8_t *buffer, const JointMsg *joint, size_t size);
    void Bytes2Float(JointAngle & joint, const uint8_t *bytes);
    void SendMsg(const S6h4dTxMsg & msg);
    void DefaultPosition(uint8_t cmd1, const JointAngle & joint, uint8_t paw);
    
private:
    // 记录反馈
    JointAngle currentJointAngle_{};
    // 记录上一次的角度控制
    JointAngle lastJointAngle_{};
    // 记录上一次的夹爪控制
    uint8_t lastPaw_ = 255;

    uint8_t txBuffer_[78]{};
    uint8_t rxBuffer_[356]{};

};