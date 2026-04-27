#include "ctrl_module/ctrl_module.h"

#include "yaml-cpp/yaml.h"


namespace mujoco_sim::ctrl_module {

bool CtrlModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      
    }

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool CtrlModule::Start() {
  AIMRT_INFO("ctrl_module Started succeeded.");
  return true;
}

void CtrlModule::Shutdown() {
  AIMRT_INFO("ctrl_module Shutdown succeeded.");
}

}  // namespace mujoco_sim::ctrl_module
