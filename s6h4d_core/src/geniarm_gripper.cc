#include "geniarm_gripper.h"

double GeniArmGripper::GetMaxRange() const {
  return kMaxRange;
}

bool GeniArmGripper::SendCommand(double position, double velocity, double effort, double stiffness, double damping) {

}