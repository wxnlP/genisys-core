#pragma once

#include "gripper_ctrl_base.h"
#include "httplib.h"
#include <string>

class SimGripper : public GripperCtrlBase {
 public:
  SimGripper(double stiffness = 0.0, double damping = 0.0)
      : GripperCtrlBase(stiffness, damping) {};
  ~SimGripper() override = default;

 protected:
  bool SendCommand(double position, double velocity, double effort,
                   double stiffness, double damping) override;
  bool ReceiveState(double& position, double& velocity,
                    double& effort) override;

 private:
  std::string http_host_{"127.0.0.1"};
  int publisher_port_{8080};
  int subscriber_port_{8080};
  const std::string gripper_cmd_url_{"/sdk/gripper_cmd"};
  const std::string gripper_state_url_{"/sdk/gripper_state"};
};