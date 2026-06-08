#include "s6h4d_ctrl.h"

#include <chrono>
#include <cstring>

namespace {
constexpr uint8_t kFrameHead = 206;  // 0xCE
constexpr uint8_t kFrameTail = 207;  // 0xCF
}  // namespace

/* ------------------------------ PRIVATE ---------------------------------- */
void S6h4dCtrl::Float2Bytes(const ArmMotion::JointCommand_t& joint,
                            uint8_t* buffer) {
  for (size_t i = 0; i < 6; i++) {
    const size_t offset = i * 3 * sizeof(float);
    const float pos = static_cast<float>(joint.position.q[i]);
    const float vel = static_cast<float>(joint.velocity.q[i]);
    const float eff = static_cast<float>(joint.effort.q[i]);
    memcpy(buffer + offset + 0 * sizeof(float), &pos, sizeof(float));
    memcpy(buffer + offset + 1 * sizeof(float), &vel, sizeof(float));
    memcpy(buffer + offset + 2 * sizeof(float), &eff, sizeof(float));
  }
}

void S6h4dCtrl::Bytes2Float(const uint8_t* bytes,
                            ArmMotion::Joint6Dof_t& joint) {
  for (int i = 0; i < 6; i++) {
    // a[3]~a[26] 对应 1~6 轴角度，每轴4字节float
    float angle = 0.0f;
    memcpy(&angle, bytes + 3 + i * sizeof(float), sizeof(float));
    joint.q[i] = static_cast<double>(angle);
  }
}

/* ------------------------------ PUBLIC ---------------------------------- */
bool S6h4dCtrl::Init() {
  if (!serial_.isOpen()) {
    serial_.open();
    if (!serial_.isOpen()) {
      return false;
    }
  }
  serial::Timeout timeout(100,  // inter_byte_timeout
                          500,  // read_timeout_constant (总超时基数)
                          0,    // read_timeout_multiplier
                          100,  // write_timeout_constant
                          0);   // write_timeout_multiplier
  serial_.setTimeout(timeout);
  joint_state_thread_ = std::jthread([this](std::stop_token stoken) {
    uint8_t frame[36]{};
    while (!stoken.stop_requested()) {
      size_t head_read = serial_.read(&frame[0], 1);
      if (head_read != 1) {
        continue;
      }
      // 判断帧头
      if (frame[0] != kFrameHead) {
        continue;
      }

      // 读取剩余部分
      size_t body_read = serial_.read(&frame[1], 36 - 1);
      if (body_read != 36 - 1) {
        continue;
      }

      if (frame[36 - 1] != kFrameTail) {
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        Bytes2Float(frame, last_joint_state_.position);
      }
    }
  });
  return true;
}

bool S6h4dCtrl::SetJointState(const ArmMotion::JointCommand_t& joint_cmd) {
  uint8_t buffer[78]{};
  buffer[0] = 0xEB;   // 帧头
  buffer[1] = 0x02;   // 通道号
  buffer[2] = 0x01;   // 命令，01定位、02外部轴、03急停、04取消急停、C8复位
  buffer[3] = 0x00;   // 夹爪(不使用)
  buffer[76] = 0x00;  // IO位，不使用
  buffer[77] = 0xEC;  // 帧尾

  Float2Bytes(joint_cmd, &buffer[4]);

  size_t bytes_written = serial_.write(buffer, sizeof(buffer));
  return bytes_written == sizeof(buffer);
}

bool S6h4dCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  joint_state = last_joint_state_;
  return true;
}
