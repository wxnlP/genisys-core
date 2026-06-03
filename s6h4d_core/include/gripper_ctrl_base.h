#pragma once

class GripperCtrlBase {
 public:
  GripperCtrlBase(double stiffness, double damping)
      : motor_stiffness_(stiffness), motor_damping_(damping){};
  virtual ~GripperCtrlBase() = default;

  // 设置夹爪开合度，范围 [0.0, 1.0]，0.0 闭合，1.0 全开
  // 内部 LSPB 梯形速度平滑（同 MoveJ 风格：物理值 → 内部归一化）
  //
  // velocity : 物理速度上限（rad/s 或 m/s，取决于 GetMaxRange 量纲）
  // accel    : 物理加速度上限（rad/s² 或 m/s²）
  // effort   : 夹持力矩（Nm），≤0 时不做力控
  bool SetPosition(double position, double velocity, double accel, double effort);

  // 获取当前夹爪开合度 [0.0, 1.0]
  bool GetPosition(double& position, double& effort);

 protected:
  // 最大行程：真机返回弧度（如 2.0 rad），仿真返回滑移量（如 0.045 m）
  virtual double GetMaxRange() const = 0;

  virtual bool SendCommand(double position_phys, double velocity_phys,
                           double effort, double stiffness,
                           double damping) = 0;

  virtual bool ReceiveState(double& position_phys, double& velocity_phys,
                            double& effort) = 0;

 private:
  static constexpr double kDefaultDt = 0.01;
  double motor_stiffness_;
  double motor_damping_;
};