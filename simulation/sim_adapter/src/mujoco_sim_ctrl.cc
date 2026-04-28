#include "mujoco_sim_ctrl.h"

#include "nlohmann_json.hpp"
#include <string>

bool MujocoSimCtrl::SendHttpRequest(const ArmMotion::JointState_t& joint_cmd) {
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

  httplib::Client cli(http_host_, publisher_port_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res = cli.Post(pub_channel_url_, json_cmd.dump(), "application/json");

  return res && res->status == 200;
}

bool MujocoSimCtrl::GetHttpResponse(ArmMotion::JointState_t& joint_state) {
  nlohmann::json json_request;

  json_request["joints"] = 6;
  httplib::Client cli(http_host_, subscriber_port_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res = cli.Post(sub_channel_url_, json_request.dump(), "application/json");
  if (!res || res->status != 200) { return false; }

  nlohmann::json json_response;
  try {
    json_response = nlohmann::json::parse(res->body);
  } catch (const std::exception&) {
    return false;
  }

  if (!json_response.contains("joints") || !json_response["joints"].is_array()) {
    return false;
  }

  for (const auto& joint : json_response["joints"]) {
    const std::string& name = joint["name"];
    double position = joint["position"];
    double velocity = joint["velocity"];
    double effort = joint["effort"];

    if (name == "shoulder_sensor") {
      joint_state.position.theta[0] = position;
      joint_state.velocity.theta[0] = velocity;
      joint_state.effort.theta[0] = effort;
    } else if (name == "upper_arm_sensor") {
      joint_state.position.theta[1] = position;
      joint_state.velocity.theta[1] = velocity;
      joint_state.effort.theta[1] = effort;
    } else if (name == "elbow_sensor") {
      joint_state.position.theta[2] = position;
      joint_state.velocity.theta[2] = velocity;
      joint_state.effort.theta[2] = effort;
    } else if (name == "wrist1_sensor") {
      joint_state.position.theta[3] = position;
      joint_state.velocity.theta[3] = velocity;
      joint_state.effort.theta[3] = effort;
    } else if (name == "wrist2_sensor") {
      joint_state.position.theta[4] = position;
      joint_state.velocity.theta[4] = velocity;
      joint_state.effort.theta[4] = effort;
    } else if (name == "wrist3_sensor") {
      joint_state.position.theta[5] = position;
      joint_state.velocity.theta[5] = velocity;
      joint_state.effort.theta[5] = effort;
    }
  }
  return true;
}

bool MujocoSimCtrl::Init() { return true; }

bool MujocoSimCtrl::SetJointState(const ArmMotion::JointState_t& joint_cmd) {
  return SendHttpRequest(joint_cmd);
}

bool MujocoSimCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  return GetHttpResponse(joint_state);
}