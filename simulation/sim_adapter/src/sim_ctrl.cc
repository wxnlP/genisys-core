#include "sim_ctrl.h"

#include "nlohmann_json.hpp"
#include <string>

bool SimCtrl::SendHttpRequest(const ArmMotion::JointCommand_t& joint_cmd) {
  nlohmann::json json_cmd;
  json_cmd["joints"] = nlohmann::json::array();

  auto add_joint = [&](const std::string& name, double pos, double vel,
                       double effort, double stiffness = 0.0, double damping = 0.0) {
    json_cmd["joints"].push_back({{"name", name},
                                  {"position", pos},
                                  {"velocity", vel},
                                  {"effort", effort},
                                  {"stiffness", stiffness},
                                  {"damping", damping}});
  };

  add_joint("shoulder_joint", joint_cmd.position.q[0],
            joint_cmd.velocity.q[0], joint_cmd.effort.q[0], joint_cmd.stiffness.q[0], joint_cmd.damping.q[0]);
  add_joint("upper_arm_joint", joint_cmd.position.q[1],
            joint_cmd.velocity.q[1], joint_cmd.effort.q[1], joint_cmd.stiffness.q[1], joint_cmd.damping.q[1]);
  add_joint("elbow_joint", joint_cmd.position.q[2],
            joint_cmd.velocity.q[2], joint_cmd.effort.q[2], joint_cmd.stiffness.q[2], joint_cmd.damping.q[2]);
  add_joint("wrist1_joint", joint_cmd.position.q[3],
            joint_cmd.velocity.q[3], joint_cmd.effort.q[3], joint_cmd.stiffness.q[3], joint_cmd.damping.q[3]);
  add_joint("wrist2_joint", joint_cmd.position.q[4],
            joint_cmd.velocity.q[4], joint_cmd.effort.q[4], joint_cmd.stiffness.q[4], joint_cmd.damping.q[4]);
  add_joint("wrist3_joint", joint_cmd.position.q[5],
            joint_cmd.velocity.q[5], joint_cmd.effort.q[5], joint_cmd.stiffness.q[5], joint_cmd.damping.q[5]);

  httplib::Client cli(http_host_, publisher_port_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res = cli.Post(pub_channel_url_, json_cmd.dump(), "application/json");

  return res && res->status == 200;
}

bool SimCtrl::GetHttpResponse(ArmMotion::JointState_t& joint_state) {
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

    if (name == "shoulder_joint") {
      joint_state.position.q[0] = position;
      joint_state.velocity.q[0] = velocity;
      joint_state.effort.q[0] = effort;
    } else if (name == "upper_arm_joint") {
      joint_state.position.q[1] = position;
      joint_state.velocity.q[1] = velocity;
      joint_state.effort.q[1] = effort;
    } else if (name == "elbow_joint") {
      joint_state.position.q[2] = position;
      joint_state.velocity.q[2] = velocity;
      joint_state.effort.q[2] = effort;
    } else if (name == "wrist1_joint") {
      joint_state.position.q[3] = position;
      joint_state.velocity.q[3] = velocity;
      joint_state.effort.q[3] = effort;
    } else if (name == "wrist2_joint") {
      joint_state.position.q[4] = position;
      joint_state.velocity.q[4] = velocity;
      joint_state.effort.q[4] = effort;
    } else if (name == "wrist3_joint") {
      joint_state.position.q[5] = position;
      joint_state.velocity.q[5] = velocity;
      joint_state.effort.q[5] = effort;
    }
  }
  return true;
}

bool SimCtrl::Init() { return true; }

bool SimCtrl::SetJointState(const ArmMotion::JointCommand_t& joint_cmd) {
  return SendHttpRequest(joint_cmd);
}

bool SimCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  return GetHttpResponse(joint_state);
}