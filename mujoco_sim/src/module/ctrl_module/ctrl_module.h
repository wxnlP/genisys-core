#pragma once

#include "aimrt_module_cpp_interface/module_base.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "joint_state.pb.h"
#include "joint_command.pb.h"

using JointCommandArray = aimrt::protocols::sensor::JointCommandArray;
using JointStateArray = aimrt::protocols::sensor::JointStateArray;

namespace mujoco_sim::ctrl_module {

class CtrlModule : public aimrt::ModuleBase {
 public:
  CtrlModule() = default;
  ~CtrlModule() override = default;

  aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "CtrlModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;

  bool Start() override;

  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;

};

}  // namespace mujoco_sim::ctrl_module
