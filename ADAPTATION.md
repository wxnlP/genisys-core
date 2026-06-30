# 适配自定义机械臂

适配一个新的六轴机械臂需要完成以下工作：

## 1️⃣ 实现驱动层（若硬件协议不兼容）

如果新机械臂的通信协议与现有的 `GeniArmCtrl`（CAN FD / DM 电机）或 `S6h4dCtrl`（串口）均不兼容，需要继承 `ArmCtrlBase` 实现新驱动类：

```cpp
class MyRobotArmCtrl : public ArmCtrlBase {
    bool Init() override;
    void Enable() override;
    void Disable() override;
    bool SetJointState(const JointCommand& cmd) override;
    bool GetJointState(JointState& state) override;
    // ...
};
```

同时，在 `ActuatorManager::ActuatorManager()` 的工厂逻辑中增加对 `name == "my_robot"` 的分支。

若夹爪也需适配，同理继承 `GripperCtrlBase`。

## 2️⃣ 配置驱动参数

创建 `my_robot_actuator_cfg.yaml`，按实际硬件填写：

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

## 3️⃣ 配置运动学参数

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

## 4️⃣ 创建顶层配置

```yaml
# cfg/my_robo_manager.yaml
actuator_config: "my_robot_actuator_cfg.yaml"
arm_config: "my_robot_body_cfg.yaml"
control_frequency: 200.0
```

## 5️⃣ （可选）配置动力学参数

编辑 `dynamics.yaml` 填写各连杆质量、质心、惯性张量及关节摩擦参数，用于力矩前馈/重力补偿。
