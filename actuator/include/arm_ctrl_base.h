#pragma once


class ArmCtrlBase {
 public:
  struct JointCommand {
    double position[6]{};
    double velocity[6]{};
    double effort[6]{};
    double stiffness[6]{};
    double damping[6]{};
  };
  struct JointState {
    double position[6]{};
    double velocity[6]{};
    double effort[6]{};
  };
  
  ArmCtrlBase() = default;
  virtual ~ArmCtrlBase() = default;

  virtual bool Init() { return true; };
  virtual void Enable() { return; };
  virtual void Disable() { return; };
  virtual bool SetJointState(const ArmCtrlBase::JointCommand& joint_cmd) = 0;
  virtual bool GetJointState(ArmCtrlBase::JointState& joint_state) = 0;
};