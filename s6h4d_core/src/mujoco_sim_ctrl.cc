#include "mujoco_sim_ctrl.h"

#include <array>
#include <cctype>
#include <string>

#include "json.hpp"

namespace {

std::string NormalizeJointName(std::string name) {
  constexpr char kActuatorSuffix[] = "_actuator";

  if (name.size() > sizeof(kActuatorSuffix) - 1 &&
      name.compare(name.size() - (sizeof(kActuatorSuffix) - 1),
                   sizeof(kActuatorSuffix) - 1, kActuatorSuffix) == 0) {
    name.erase(name.size() - (sizeof(kActuatorSuffix) - 1));
  }

  for (char& ch : name) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  return name;
}

bool TryGetJointIndex(const std::string& name, std::size_t& index) {
  const std::string normalized = NormalizeJointName(name);

  if (normalized == "shoulder") {
    index = 0;
    return true;
  }
  if (normalized == "upper_arm") {
    index = 1;
    return true;
  }
  if (normalized == "elbow") {
    index = 2;
    return true;
  }
  if (normalized == "wrist1") {
    index = 3;
    return true;
  }
  if (normalized == "wrist2") {
    index = 4;
    return true;
  }
  if (normalized == "wrist3") {
    index = 5;
    return true;
  }

  return false;
}

bool ParseJointStateJson(const nlohmann::json& json_resp,
                         ArmMotion::JointState_t& parsed_state) {
  if (!json_resp.contains("joints") || !json_resp["joints"].is_array()) {
    return false;
  }

  ArmMotion::JointState_t tmp{};
  std::array<bool, 6> assigned{false, false, false, false, false, false};
  std::size_t fallback_index = 0;

  for (const auto& joint_json : json_resp["joints"]) {
    if (!joint_json.is_object()) continue;

    std::size_t idx = 0;
    bool has_named_index = false;
    if (joint_json.contains("name") && joint_json["name"].is_string()) {
      has_named_index =
          TryGetJointIndex(joint_json["name"].get<std::string>(), idx);
    }

    if (!has_named_index) {
      while (fallback_index < 6 && assigned[fallback_index]) {
        ++fallback_index;
      }
      if (fallback_index >= 6) continue;
      idx = fallback_index;
    }

    if (joint_json.contains("position") && joint_json["position"].is_number()) {
      tmp.position.theta[idx] = joint_json["position"].get<double>();
    }
    if (joint_json.contains("velocity") && joint_json["velocity"].is_number()) {
      tmp.velocity.theta[idx] = joint_json["velocity"].get<double>();
    }
    if (joint_json.contains("effort") && joint_json["effort"].is_number()) {
      tmp.effort.theta[idx] = joint_json["effort"].get<double>();
    }

    if (joint_json.contains("accel") && joint_json["accel"].is_number()) {
      tmp.accel.theta[idx] = joint_json["accel"].get<double>();
    } else if (joint_json.contains("acceleration") &&
               joint_json["acceleration"].is_number()) {
      tmp.accel.theta[idx] = joint_json["acceleration"].get<double>();
    }

    assigned[idx] = true;
  }

  for (bool ok : assigned) {
    if (!ok) return false;
  }

  parsed_state = tmp;
  return true;
}

}  // namespace

MujocoSimCtrl::~MujocoSimCtrl() {
  if (receiver_started_) {
    svr_.stop();
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }
}

bool MujocoSimCtrl::SendHttpRequest(const ArmMotion::JointState_t& joint_cmd) {
  // Convert joint_cmd to JSON
  nlohmann::json json_cmd;
  json_cmd["joints"] = nlohmann::json::array();

  auto add_joint = [&](const std::string& name, double pos, double vel,
                       double effort) {
    json_cmd["joints"].push_back({{"name", name},
                                  {"position", pos},
                                  {"velocity", vel},
                                  {"effort", effort},
                                  {"stiffness", 0.0},
                                  {"damping", 0.0}});
  };

  // Keep names aligned with mujoco install tools and XML actuator names.
  add_joint("shoulder_actuator", joint_cmd.position.theta[0],
            joint_cmd.velocity.theta[0], joint_cmd.effort.theta[0]);
  add_joint("upper_arm_actuator", joint_cmd.position.theta[1],
            joint_cmd.velocity.theta[1], joint_cmd.effort.theta[1]);
  add_joint("elbow_actuator", joint_cmd.position.theta[2],
            joint_cmd.velocity.theta[2], joint_cmd.effort.theta[2]);
  add_joint("wrist1_actuator", joint_cmd.position.theta[3],
            joint_cmd.velocity.theta[3], joint_cmd.effort.theta[3]);
  add_joint("wrist2_actuator", joint_cmd.position.theta[4],
            joint_cmd.velocity.theta[4], joint_cmd.effort.theta[4]);
  add_joint("wrist3_actuator", joint_cmd.position.theta[5],
            joint_cmd.velocity.theta[5], joint_cmd.effort.theta[5]);

  httplib::Client cli(client_host_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res =
      cli.Post(pub_channel_url_.c_str(), json_cmd.dump(), "application/json");

  return res && res->status == 200;
}

bool MujocoSimCtrl::GetHttpResponse(ArmMotion::JointState_t& joint_state) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!has_state_) {
    return false;
  }
  joint_state = current_joint_state_;
  return true;
}

bool MujocoSimCtrl::StartStateReceiver() {
  if (receiver_started_) {
    return true;
  }

  svr_.Post(sub_channel_url_.c_str(),
            [this](const httplib::Request& req, httplib::Response& res) {
              nlohmann::json json_resp;
              try {
                json_resp = nlohmann::json::parse(req.body);
              } catch (...) {
                res.status = 400;
                return;
              }

              ArmMotion::JointState_t parsed_state{};
              if (!ParseJointStateJson(json_resp, parsed_state)) {
                res.status = 400;
                return;
              }

              {
                std::lock_guard<std::mutex> lk(state_mtx_);
                current_joint_state_ = parsed_state;
                has_state_ = true;
              }
              res.status = 200;
            });

  if (!svr_.bind_to_port(server_listen_ip_.c_str(), server_listen_port_)) {
    return false;
  }

  receiver_thread_ = std::thread([this]() { svr_.listen_after_bind(); });

  receiver_started_ = true;
  return true;
}

bool MujocoSimCtrl::Init() { return StartStateReceiver(); }

bool MujocoSimCtrl::SetJointState(const ArmMotion::JointState_t& joint_cmd) {
  return SendHttpRequest(joint_cmd);
}

bool MujocoSimCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  return GetHttpResponse(joint_state);
}