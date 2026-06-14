#pragma once

class GripperCtrlBase {
 public:
  GripperCtrlBase(double stiffness, double damping)
      : motor_stiffness_(stiffness), motor_damping_(damping){};
  virtual ~GripperCtrlBase() = default;

  virtual void Init() {};
  virtual void Enable() { return; };
  virtual void Disable() { return; };
  
  void SetMaxRange(double max_range) { max_range_ = max_range; }
  // velocity : 物理速度上限（rad/s 或 m/s，取决于 max_range_ 量纲）
  // accel    : 物理加速度上限（rad/s² 或 m/s²）
  // effort   : 夹持力矩（Nm），≤0 时不做力控
  bool SetPosition(double position, double velocity, double accel, double effort);
  bool SetTorque(double effort, double ramp_rate = 3.0);
  // 获取当前夹爪开合度 [0.0, 1.0]
  bool GetPosition(double& position, double& effort);

 protected:
  virtual bool SendCommand(double position, double velocity,
                           double effort, double stiffness, double damping) = 0;

  virtual bool ReceiveState(double& position, double& velocity,
                            double& effort) = 0;

 private:
  static constexpr double kDefaultDt = 0.01;
  double motor_stiffness_;
  double motor_damping_;
  double max_range_;
};