#include "proxy_module/proxy_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "yaml-cpp/yaml.h"

namespace sim_proxy::proxy_module {

bool ProxyModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      proxy_http_joint_state_topic_ = cfg_node["proxy_http_joint_state_topic"].as<std::string>();
      mujoco_joint_state_topic_ = cfg_node["mujoco_joint_state_topic"].as<std::string>();
    }

    joint_state_sub_ = core_.GetChannelHandle().GetSubscriber(mujoco_joint_state_topic_);
    AIMRT_CHECK_ERROR(joint_state_sub_, "Failed to get channel subscriber for topic: {}",
                      mujoco_joint_state_topic_);

    aimrt::channel::Subscribe<JointStateArray>()


  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ProxyModule::Start() {
  AIMRT_INFO("proxy_module Started succeeded.");
  return true;
}

void ProxyModule::Shutdown() {
  AIMRT_INFO("proxy_module Shutdown succeeded.");
}

}  // namespace sim_proxy::proxy_module
