#include "rviz_module/rviz_module.h"
#include "aimrt_module_cpp_interface/executor/executor.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

#include "yaml-cpp/yaml.h"

namespace sim_proxy::rviz_module {

bool RvizModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      proxy_joint_state_topic_ = cfg_node["proxy_joint_state_topic"].as<std::string>();
    }

    proxy_joint_state_sub_ = core_.GetChannelHandle().GetSubscriber(proxy_joint_state_topic_);
    AIMRT_CHECK_ERROR(proxy_joint_state_sub_, "Create subscriber failed, topic: {}", proxy_joint_state_topic_);

    rviz_joint_state_pub_ = core_.GetChannelHandle().GetPublisher(rviz_topic_);
    AIMRT_CHECK_ERROR(rviz_joint_state_pub_, "Create publisher failed, topic: {}", rviz_topic_);

    bool ret;
    // 订阅Proxy Module的关节状态信息
    ret = aimrt::channel::Subscribe<JointStateArray>(
        proxy_joint_state_sub_, 
        std::bind(&RvizModule::SubProxyJointState, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR(ret, "Subscribe failed, topic: {}", proxy_joint_state_topic_);

    // 发布关节状态到RViz
    ret = aimrt::channel::RegisterPublishType<JointState>(rviz_joint_state_pub_);
    AIMRT_CHECK_ERROR(ret, "Register publish type failed, topic: {}", rviz_topic_);

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool RvizModule::Start() {
  AIMRT_INFO("rviz_module Started succeeded.");
  return true;
}

void RvizModule::Shutdown() {
  AIMRT_INFO("rviz_module Shutdown succeeded.");
}

void RvizModule::SubProxyJointState(const std::shared_ptr<const JointStateArray>& msg) {
  JointState rviz_msg;
  rviz_msg.header.frame_id = msg->header().frame_id();
  // auto total_ns = msg->header().time_stamp();
  // rviz_msg.header.stamp.sec = static_cast<int32_t>(total_ns / 1000000000ull);
  // rviz_msg.header.stamp.nanosec = static_cast<uint32_t>(total_ns % 1000000000ull);
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
  rviz_msg.header.stamp.sec = static_cast<int32_t>(seconds.count());
  rviz_msg.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds.count());

  int joint_count = msg->joints_size();
  rviz_msg.name.resize(joint_count);
  rviz_msg.position.resize(joint_count);
  rviz_msg.velocity.resize(joint_count);
  rviz_msg.effort.resize(joint_count);
  for (int i = 0; i < joint_count; i++) {
    const auto& joint_state = msg->joints(i);
    rviz_msg.name[i] = joint_state.name();
    rviz_msg.position[i] = joint_state.position();
    rviz_msg.velocity[i] = joint_state.velocity();
    rviz_msg.effort[i] = joint_state.effort();
  }

  aimrt::channel::Publish(rviz_joint_state_pub_, rviz_msg);
}

}  // namespace sim_proxy::rviz_module
