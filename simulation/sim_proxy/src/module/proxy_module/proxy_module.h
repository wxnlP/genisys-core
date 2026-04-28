#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "joint_state.pb.h"
#include "httplib.h"
#include <mutex>

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
  void GetCurrentJointState(JointStateArray& joint_state_array) {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_state_array = current_joint_state_;
  }
  void UpdateCurrentJointState(const JointStateArray& joint_state_array) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_joint_state_ = joint_state_array;
  }
  void HttpServerThread();
  void MujocoJointStateCallback(const std::shared_ptr<const JointStateArray>& joint_state_array);

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef http_server_executor_;
  aimrt::channel::SubscriberRef joint_state_sub_;

  httplib::Server http_server_;
  std::string http_ececutor_name_;
  // 外部读取的数据暂存变量
  JointStateArray current_joint_state_;
  std::string mujoco_joint_state_topic_;
  // std::string mujoco_joint_cmd_topic_;
  std::string proxy_http_joint_state_topic_;
  // std::string proxy_http_cmd_state_topic_;

  std::mutex mutex_;
  std::promise<void> stop_sig_;
};

}  // namespace sim_proxy::proxy_module
