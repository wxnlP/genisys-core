#include "proxy_module/proxy_module.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "yaml-cpp/yaml.h"
#include "nlohmann_json.hpp"

namespace sim_proxy::proxy_module {

bool ProxyModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));
      sdk_http_joint_state_topic_ = cfg_node["sdk_http_joint_state_topic"].as<std::string>();
      sdk_http_joint_cmd_topic_ = cfg_node["sdk_http_joint_cmd_topic"].as<std::string>();
      proxy_joint_state_topic_ = cfg_node["proxy_joint_state_topic"].as<std::string>();
      proxy_joint_cmd_topic_ = cfg_node["proxy_joint_cmd_topic"].as<std::string>();
      http_executor_name_ = cfg_node["http_server_executor"].as<std::string>();
    }

    http_server_executor_ = core_.GetExecutorManager().GetExecutor(http_executor_name_);
    AIMRT_CHECK_ERROR(http_server_executor_, "Failed to get executor: {}", http_executor_name_);

    proxy_joint_state_sub_ = core_.GetChannelHandle().GetSubscriber(proxy_joint_state_topic_);
    AIMRT_CHECK_ERROR(proxy_joint_state_sub_, "Failed to get channel subscriber for topic: {}",
                      proxy_joint_state_topic_);

    proxy_joint_cmd_pub_ = core_.GetChannelHandle().GetPublisher(proxy_joint_cmd_topic_);
    AIMRT_CHECK_ERROR(proxy_joint_cmd_pub_, "Failed to get channel publisher for topic: {}",
                      proxy_joint_cmd_topic_);

    bool ret = aimrt::channel::Subscribe<JointStateArray>(
        proxy_joint_state_sub_,
        std::bind(&ProxyModule::SubProxyJointState, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR(ret, "Failed to subscribe to topic: {}", proxy_joint_state_topic_);

    ret = aimrt::channel::RegisterPublishType<JointCommandArray>(proxy_joint_cmd_pub_);
    AIMRT_CHECK_ERROR(ret, "Failed to register publish type for topic: {}", proxy_joint_cmd_topic_);

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool ProxyModule::Start() {
  try {
    http_server_executor_.Execute(std::bind(&ProxyModule::HttpServerThread, this));
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }
  AIMRT_INFO("proxy_module Started succeeded.");
  return true;
}

void ProxyModule::Shutdown() {
  try {
    AIMRT_INFO("Stopping HTTP server...");
    http_server_.stop();
    stop_sig_.get_future().wait();
  } catch (std::exception& e) {
    AIMRT_ERROR("Shutdown failed, {}", e.what());
    return;
  }
  AIMRT_INFO("proxy_module Shutdown succeeded.");
}

void ProxyModule::HttpServerThread() {
  AIMRT_INFO("HTTP server thread started.");
  // Post1: SDK请求获取仿真数据
  http_server_.Post(sdk_http_joint_state_topic_, [this](const httplib::Request& req, httplib::Response& res) {
    try {
      AIMRT_DEBUG("Received HTTP request for mujoco joint state.");
      nlohmann::json req_json = nlohmann::json::parse(req.body);
      nlohmann::json res_json;
      AIMRT_INFO("Request msg: ", req_json["joints"].dump());
      
      if (req_json.contains("joints")) {
        res_json["joints"] = nlohmann::json::array();
        JointStateArray current_joint_state;
        GetCurrentJointState(current_joint_state);
        for (auto joint : current_joint_state.joints()) {
          nlohmann::json joint_json;
          joint_json["name"] = joint.name();
          joint_json["position"] = joint.position();
          joint_json["velocity"] = joint.velocity();
          joint_json["effort"] = joint.effort();
          res_json["joints"].push_back(joint_json);
        }
        res.status = 200;
        res.set_content(res_json.dump(), "application/json");
      } else {
        res.status = 400; 
        res.set_content("Invalid JSON format", "application/json");
        return;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Failed to parse HTTP request body: {}", e.what());
      res.status = 400; 
      res.set_content("Invalid JSON format", "application/json");
      return;
    }
  });

  // Post2: SDK发起控制仿真
  http_server_.Post(sdk_http_joint_cmd_topic_, [this](const httplib::Request& req, httplib::Response& res) {
    try {
      AIMRT_DEBUG("Received HTTP request for mujoco joint command.");
      nlohmann::json req_json = nlohmann::json::parse(req.body);
      nlohmann::json res_json;
      AIMRT_INFO("Request msg: ", req_json["joints"].dump());

      if (req_json.contains("joints")) {
        JointCommandArray joint_cmd_array;
        for (const auto& joint_json : req_json["joints"]) {
          auto* joint_cmd = joint_cmd_array.add_joints();
          joint_cmd->set_name(joint_json["name"].get<std::string>());
          joint_cmd->set_position(joint_json["position"].get<double>());
          joint_cmd->set_velocity(joint_json["velocity"].get<double>());
          joint_cmd->set_effort(joint_json["effort"].get<double>());
          joint_cmd->set_damping(joint_json["damping"].get<double>());
          joint_cmd->set_stiffness(joint_json["stiffness"].get<double>());
        }

        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto total_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        joint_cmd_array.mutable_header()->set_time_stamp(static_cast<uint64_t>(total_nanoseconds));
        aimrt::channel::Publish(proxy_joint_cmd_pub_, joint_cmd_array);

        res.status = 200;
        res.set_content("Joint command published successfully", "application/json");
      } else {
        res.status = 400; 
        res.set_content("Invalid JSON format", "application/json");
        return;
      }
    } catch (const std::exception& e) {
      AIMRT_WARN("Failed to parse HTTP request body: {}", e.what());
      res.status = 400; 
      res.set_content("Invalid JSON format", "application/json");
      return;
    }
  });
  
  if (!http_server_.listen("127.0.0.1", 8080)) {
    AIMRT_ERROR("Failed to start HTTP server.");
  }
  stop_sig_.set_value();
  AIMRT_INFO("HTTP server thread stopped.");
}

void ProxyModule::SubProxyJointState(const std::shared_ptr<const JointStateArray>& msg) {
  UpdateCurrentJointState(*msg);
  AIMRT_DEBUG("Received joint state array with {} joints.", current_joint_state_.joints_size());
}

}  // namespace sim_proxy::proxy_module
