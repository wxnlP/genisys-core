#include "sim_gripper.h"

#include "nlohmann_json.hpp"


bool SimGripper::SendCommand(double position, double velocity, double effort,
                             double stiffness, double damping) {
  nlohmann::json json_cmd;
  json_cmd["position"] = position;
  json_cmd["velocity"] = velocity;
  json_cmd["effort"] = effort;
  json_cmd["stiffness"] = stiffness;
  json_cmd["damping"] = damping;

  httplib::Client cli(http_host_, publisher_port_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res = cli.Post(gripper_cmd_url_, json_cmd.dump(), "application/json");
  return res && res->status == 200;
}

bool SimGripper::ReceiveState(double& position, double& velocity,
                              double& effort) {
  nlohmann::json json_request;
  json_request["query"] = "gripper_state";

  httplib::Client cli(http_host_, subscriber_port_);
  cli.set_connection_timeout(2, 0);
  cli.set_read_timeout(2, 0);
  cli.set_write_timeout(2, 0);

  auto res =
      cli.Post(gripper_state_url_, json_request.dump(), "application/json");
  if (!res || res->status != 200) {
    return false;
  }

  nlohmann::json json_response;
  try {
    json_response = nlohmann::json::parse(res->body);
  } catch (const std::exception&) {
    return false;
  }

  position = json_response.value("position", 0.0);
  velocity = json_response.value("velocity", 0.0);
  effort = json_response.value("effort", 0.0);
  return true;
}
