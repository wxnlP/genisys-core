#pragma once

class GripperCtrlBase {
 public:
  GripperCtrlBase(double stiffness, double damping)
      : motor_stiffness_(stiffness), motor_damping_(damping) {}
  virtual ~GripperCtrlBase() = default;

  virtual void Init() {}
  virtual void Enable() {}
  virtual void Disable() {}

  void SetMaxRange(double max_range) { max_range_ = max_range; }
  double GetMaxRange() const { return max_range_; }
  double GetStiffness() const { return motor_stiffness_; }
  double GetDamping() const { return motor_damping_; }

  virtual bool SendCommand(double position, double velocity,
                           double effort, double stiffness, double damping) = 0;
  virtual bool ReceiveState(double& position, double& velocity,
                            double& effort) = 0;

 protected:
  double motor_stiffness_;
  double motor_damping_;
  double max_range_ = 1.0;
};