# 适配自定义机械臂

适配一个新的六轴机械臂需要完成以下工作：

## 实现驱动层

继承 `ArmCtrlBase` 实现机械臂本体驱动：

```cpp
class MyRobotArmCtrl : public ArmCtrlBase {
  public:
    bool Init() override;
    void Enable() override;
    void Disable() override;
    bool SetJointState(const JointCommand& cmd) override;
    bool GetJointState(JointState& state) override;
    // ...
};
```

若夹爪也需适配，同理继承 `GripperCtrlBase`：

```c++
class MyGripperCtrl : public GripperCtrlBase {
  public:
    void Init() override;
    void Enable() override;
    void Disable() override;
    void Shutdown() override;
    bool ReceiveState(GripperState& state) override;
    bool SendCommand(const GripperCommand& command) override;

    // ...
};
```



## 适配 yaml 管理

如果你希望与 geniarm 一样可以通过 yaml 配置完成硬件初始化，必须在 `ActuatorManager::ActuatorManager()` 中增加 `name == "my_robot"` 的分支以辨识自定义机械臂。

同时，创建 `my_robot_actuator_cfg.yaml`，按实际硬件填写：

```yaml
actuator:
  arm:
    name: "my_robot"        # 与 ActuatorManager 新增分支的 name 一致
    protocol:
      port: "can0"           # 通信端口
    ctrl_mode: "MIT"
    joint_limits:
      lower: [-180, -90, ...]
      upper: [180, 90, ...]
    joints:
      - name: "joint1"
        motor_id: 0x01
        # ...
```

## 配置运动学参数

创建 `my_robot_body_cfg.yaml`，填写 MDH 参数和关节限位：

```yaml
robot:
  name: "my_robot"
  kinematics:
    ik_mode: "UR"            # 或 "Piper" (ParallelWrist)
    MDH:
      alpha: [...]           # MDH 参数 α (度)
      a: [...]               # MDH 参数 a (m)
      d: [...]               # MDH 参数 d (m)
      offset: [...]          # 关节零位偏移 (度)
```

MDH 参数坐标系定义遵循标准改进型 Denavit-Hartenberg 约定。

## 创建顶层配置

> 如果直接使用[robotic_arm_aimrt](https://github.com/wxnlP/robotic_arm_aimrt)那么该步骤可以跳过。

```yaml
# cfg/my_body_cfg.yaml
actuator_config: "my_robot_actuator_cfg.yaml"
arm_config: "my_robot_body_cfg.yaml"
control_frequency: 200.0
```

## 配置动力学参数

编辑 `dynamics.yaml` 填写各连杆质量、质心、惯性张量及关节摩擦参数，用于力矩前馈。
