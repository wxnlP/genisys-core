#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "joint_state.pb.h"


// ROS2 protocol message
using JointState = sensor_msgs::msg::JointState;
// AimRT protocol message
using JointStateArray = aimrt::protocols::sensor::JointStateArray;

namespace sim_proxy::rviz_module {

class RvizModule : public aimrt::ModuleBase {
 public:
  RvizModule() = default;
  ~RvizModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "RvizModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void SubProxyJointState(const std::shared_ptr<const JointStateArray>& msg);

 private:
  aimrt::CoreRef core_;
  aimrt::channel::SubscriberRef proxy_joint_state_sub_;
  aimrt::channel::PublisherRef rviz_joint_state_pub_;

  std::string proxy_joint_state_topic_;
  std::string_view rviz_topic_ = "/joint_states";

};

}  // namespace sim_proxy::rviz_module
