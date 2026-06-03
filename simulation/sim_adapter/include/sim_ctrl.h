#pragma once

#include "arm_ctrl_base.h"
#include "httplib.h"

class SimCtrl : public ArmCtrlBase {
 public:
  SimCtrl() = default;
  ~SimCtrl() override = default;

  bool Init() override;
  bool SetJointState(const ArmMotion::JointCommand_t& joint_cmd) override;
  bool GetJointState(ArmMotion::JointState_t& joint_state) override;

 private:
  bool SendHttpRequest(const ArmMotion::JointCommand_t& joint_cmd);
  bool GetHttpResponse(ArmMotion::JointState_t& joint_state);

  std::string http_host_{"127.0.0.1"};
  int subscriber_port_{8080};
  int publisher_port_{8080};
  const std::string pub_channel_url_{"/sdk/joint_cmd"};
  const std::string sub_channel_url_{"/sdk/joint_state"};

  ArmMotion::JointState_t current_joint_state_{};
};