#include "interface_base.h"
#include <cstring>


/* -----------------------------------PRIVATE---------------------------------------- */

void InterfaceBase::Float2Bytes(uint8_t *buffer, const JointMsg *joint, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        size_t offset = i * 3 * sizeof(float);
        memcpy(buffer + offset + 0 * sizeof(float), &joint[i].angle, sizeof(float));
        memcpy(buffer + offset + 1 * sizeof(float), &joint[i].vel, sizeof(float));
        memcpy(buffer + offset + 2 * sizeof(float), &joint[i].accel, sizeof(float));
    }
}

void InterfaceBase::Bytes2Float(JointAngle & joint, const uint8_t *bytes)
{
    for (int i = 0; i < 6; i++) {
        memcpy(&joint[i], bytes + i * 4, sizeof(float));
    }
}

void InterfaceBase::SendMsg(const S6h4dTxMsg & msg) 
{
    txBuffer_[0] = 0xEB;
    txBuffer_[1] = msg.cmd[0];
    txBuffer_[2] = msg.cmd[1];
    txBuffer_[3] = msg.cmd[2];
    txBuffer_[76] = msg.cmd[3];
    txBuffer_[77] = 0xEC;

    // 将12个float转为72个byte
    Float2Bytes(&txBuffer_[4], msg.joint, 6);

    // 发送指令
    WriteBytes(txBuffer_, 78);
}

void InterfaceBase::DefaultPosition(uint8_t cmd1, const JointAngle & joint, uint8_t paw)
{
    S6h4dTxMsg msg;
    msg.cmd[0] = 0x02;
    msg.cmd[1] = cmd1;
    msg.cmd[2] = paw;
    msg.cmd[3] = 0x00;

    // 使用默认配置的速度
    for (int i = 0; i < 6; i++) {
        msg.joint[i].angle = joint[i];
        msg.joint[i].vel = 40.0;
        msg.joint[i].accel = 0.0;
    }

    SendMsg(msg); 

    // 记录角度
    lastJointAngle_ = joint;
    lastPaw_ = paw;
}

/* -----------------------------------PUBLIC----------------------------------------- */

void InterfaceBase::Reset()
{
    JointAngle resetAngle{};
    resetAngle[2] = 105.0;
    DefaultPosition(0xC8, resetAngle, 255);
}

void InterfaceBase::ArmPosition(uint8_t paw)
{
    DefaultPosition(0x01, lastJointAngle_, paw);
}


void InterfaceBase::ArmPosition(const JointAngle & joint)
{
    DefaultPosition(0x01, joint, lastPaw_);
}


void InterfaceBase::ArmPosition(const JointAngle & joint, uint8_t paw)
{
    DefaultPosition(0x01, joint, paw);
}

void InterfaceBase::ArmPosition(const S6h4dTxMsg & msg)
{
    SendMsg(msg);
    // 记录角度
    for (int i = 0; i < 6; i++) {
        lastJointAngle_[i] = msg.joint[i].angle;
    }
    
    lastPaw_ = msg.cmd[2];
}


void InterfaceBase::ArmPosition(uint8_t jointIndex, float angle, float vel, float accel)
{
    S6h4dTxMsg msg;
    msg.cmd[0] = 0x02;
    msg.cmd[1] = 0x01;
    msg.cmd[2] = lastPaw_;
    msg.cmd[3] = 0x00;

    // 使用默认配置的速度
    for (int i = 0; i < 6; i++) {
        msg.joint[i].angle = lastJointAngle_[i];
        msg.joint[i].vel = 40.0;
        msg.joint[i].accel = 0.0;
    }
    msg.joint[jointIndex].angle = angle;
    msg.joint[jointIndex].vel = vel;
    msg.joint[jointIndex].accel = accel;

    SendMsg(msg); 

    // 记录角度
    lastJointAngle_[jointIndex] = angle;
}

InterfaceBase::JointAngle InterfaceBase::GetJointAngle()
{
    bool flag = true;
    while (flag) {
        ReadBytes(rxBuffer_, 2);
        // 判断帧头
        if (rxBuffer_[0] == 0x5A && rxBuffer_[1] == 0xA5) {
            flag = false;
        }
    }
    ReadBytes(&rxBuffer_[2], 354);
    JointAngle resJoint{};
    Bytes2Float(resJoint, &rxBuffer_[324]);
    return resJoint;
}

