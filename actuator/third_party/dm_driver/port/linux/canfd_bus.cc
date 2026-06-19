#include "canfd_bus.h"
#include "socket_can.h"
#include <cstring>


SocketCANBus::SocketCANBus(const std::string &interface_name, bool fd_mode)
    : can_interface_(interface_name), fd_mode_(fd_mode) {}

SocketCANBus::~SocketCANBus() {
  SocketCAN_Close(socket_fd_);
}

int SocketCANBus::Init() {
  socket_fd_ = SocketCAN_Connect(can_interface_.c_str(), fd_mode_);
  if (socket_fd_ < 0) {
    return -1;
  }
  sample_thread_ = std::jthread([this](std::stop_token st) {
    while (!st.stop_requested()) {
      Sample();
    }
  });
  return socket_fd_;
}

void SocketCANBus::SetReceiverId(uint32_t *id, uint8_t len) {
  std::lock_guard<std::mutex> lock(fb_frames_mutex_);
  for (int i = 0; i < len; ++i) {
    fb_frames_[i].master_id = id[i];
  }
  // 将接收 ID 注册为内核 CAN 过滤器，socket 只接收这些 ID 的帧
  if (socket_fd_ > 0) {
    uint32_t ids[kMAX_MOTOR_NUM];
    for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
      ids[i] = fb_frames_[i].master_id;
    }
    SocketCAN_SetFilters(socket_fd_, ids, kMAX_MOTOR_NUM);
  }
}

void SocketCANBus::Sample() {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8]{};
  int bytes;
  if (fd_mode_) {
    bytes = SocketCANFD_Receive(socket_fd_, &id, &dlc, data);
  } else {
    bytes = SocketCAN_Receive(socket_fd_, &id, &dlc, data);
  }
  if (bytes > 0) {
    std::lock_guard<std::mutex> lock(fb_frames_mutex_);
    for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
      if (fb_frames_[i].master_id == id) {
        memcpy(fb_frames_[i].data, data, dlc);
        fb_frames_[i].valid = true;
        break;
      }
    }
  }
}

int SocketCANBus::Send(uint32_t id, const uint8_t *data, uint8_t len) {
  if (fd_mode_) {
    return SocketCANFD_Send(socket_fd_, id, len, false, data);
  } else {
    return SocketCAN_Send(socket_fd_, id, len, data);
  }
}

int SocketCANBus::Receive(uint32_t id, uint8_t *data) {
  std::lock_guard<std::mutex> lock(fb_frames_mutex_);
  for (int i = 0; i < kMAX_MOTOR_NUM; ++i) {
    if (fb_frames_[i].master_id == id + 0x10 && fb_frames_[i].valid) {
      memcpy(data, fb_frames_[i].data, 8);
      return 0;
    }
  }
  return -1;
}
