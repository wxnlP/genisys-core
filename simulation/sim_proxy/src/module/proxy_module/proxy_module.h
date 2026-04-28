#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "joint_state.pb.h"

using JointStateArray = aimrt::protocols::sensor::JointStateArray;

namespace sim_proxy::proxy_module {

class ProxyModule : public aimrt::ModuleBase {
 public:
  ProxyModule() = default;
  ~ProxyModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ProxyModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void MujocoJointStateCallback(const std::shared_ptr<const JointStateArray>& joint_state_array);

 private:
  aimrt::CoreRef core_;
  aimrt::channel::SubscriberRef joint_state_sub_;

  // 外部读取的数据暂存变量
  JointStateArray current_joint_state_;
  std::string mujoco_joint_state_topic_;
  // std::string mujoco_joint_cmd_topic_;
  std::string proxy_http_joint_state_topic_;
  // std::string proxy_http_cmd_state_topic_;
};

}  // namespace sim_proxy::proxy_module
