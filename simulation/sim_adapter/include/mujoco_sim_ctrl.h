#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "arm_ctrl_base.h"
#include "httplib.h"

class MujocoSimCtrl : public ArmCtrlBase {
 public:
  MujocoSimCtrl() = default;
  ~MujocoSimCtrl() override;

  bool Init() override;
  bool SetJointState(const ArmMotion::JointState_t& joint_cmd) override;
  bool GetJointState(ArmMotion::JointState_t& joint_state) override;

 private:
  bool SendHttpRequest(const ArmMotion::JointState_t& joint_cmd);
  bool GetHttpResponse(ArmMotion::JointState_t& joint_state);
  bool StartStateReceiver();

  httplib::Server svr_;

  std::string client_host_{"http://127.0.0.1:50080"};
  std::string server_listen_ip_{"127.0.0.1"};
  int server_listen_port_{50081};
  const std::string pub_channel_url_{
      "/channel/%2Fs6h4d%2Fjoint_cmd/"
      "pb%3Aaimrt.protocols.sensor.JointCommandArray"};
  const std::string sub_channel_url_{
      "/channel/%2Fmujoco%2Fjoint_state/"
      "pb%3Aaimrt.protocols.sensor.JointStateArray"};

  std::thread receiver_thread_;
  std::atomic<bool> receiver_started_{false};
  bool has_state_{false};
  std::mutex state_mtx_;
  ArmMotion::JointState_t current_joint_state_{};
};