#pragma once

#include "aimrt_module_cpp_interface/module_base.h"

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

 private:
  aimrt::CoreRef core_;
};

}  // namespace sim_proxy::proxy_module
