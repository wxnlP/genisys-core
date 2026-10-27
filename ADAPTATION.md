# 适配自定义机械臂

适配一个新的六轴机械臂，需要同时完成 **驱动层 → 构建层 → 配置层 → 运行层** 四部分工作。下面以 `my_robot` 为例，`geniarm` 的现有实现可以直接作为参考。

## 0. 文件清单

| 层级 | 需要新增或修改的文件 |
|:---|:---|
| 驱动接口 | `actuator/include/my_robot_ctrl.h`、`actuator/src/my_robot_ctrl.cc` |
| 总线/协议 | `actuator/include/actuator_manager.h`（需要保存总线对象时）、`actuator/src/actuator_manager.cc` |
| 第三方依赖 | `actuator/third_party/` 下新增，或复用 `dm_driver` / `serial` |
| 构建 | `actuator/CMakeLists.txt` |
| 驱动配置 | `cfg/my_robot_actuator_cfg.yaml` |
| 运动学配置 | `cfg/my_robot_body_cfg.yaml` |
| 动力学配置 | `cfg/dynamics/my_robot_dynamics.yaml`（可选） |

> `cfg/` 是 SDK 的默认配置目录，管理器从运行目录的 `./cfg/` 读取，见第 7 节。

## 1. 实现驱动层

继承 `ArmCtrlBase` 实现机械臂本体驱动。六轴机械臂的关节顺序必须与配置文件和控制器保持一致：

```cpp
#pragma once

#include "arm_ctrl_base.h"

class MyRobotArmCtrl : public ArmCtrlBase {
 public:
  explicit MyRobotArmCtrl(MyBus& bus);
  ~MyRobotArmCtrl() override;

  bool Init() override;
  void Enable() override;
  void Disable() override;
  bool SetJointState(const ArmCtrlBase::JointCommand& joint_cmd) override;
  bool GetJointState(ArmCtrlBase::JointState& joint_state) override;

  // 可选：支持 MIT / POS / VEL 切换时实现
  bool SetCtrlMode(ArmCtrlBase::CtrlMode mode) override;

  // 可选：返回已经完成标零的关节 bitmask
  uint32_t CalibrateZero() override;
};
```

实现时注意：

- `SetJointState()` 由控制循环高频调用，内部不要阻塞等待总线。
- `GetJointState()` 返回最近一次反馈缓存；如果驱动有独立接收线程，需要自己加锁。
- `Enable()` / `Disable()` 只负责使能/失能，不要做耗时初始化。
- `Init()` 返回 `false` 会让 `ActuatorManager` 构造失败。
- `CalibrateZero()` 的返回值按位表示关节，未实现时返回 `0`。

若夹爪也需要适配，继承 `GripperCtrlBase`：

```cpp
class MyGripperCtrl : public GripperCtrlBase {
 public:
  void Init() override;
  void Enable() override;
  void Disable() override;
  void Shutdown() override;
  bool SendCommand(const GripperCommand& command) override;
  bool ReceiveState(GripperState& state) override;

  // 可选
  bool CalibrateZero() override;
};
```

## 2. 注册到 ActuatorManager

`ActuatorManager::ActuatorManager()` 根据 `actuator.arm.name` 选择驱动实现。需要新增一个分支：

```cpp
#include "my_robot_ctrl.h"

// ...

if (arm_name == "my_robot") {
  auto proto = arm_cfg["protocol"];
  std::string port = proto["port"].as<std::string>();
  int baud_rate = proto["baud_rate"].as<int>();

  for (const auto& joint : arm_cfg["joints"]) {
    joint_names_.push_back(joint["name"].as<std::string>());
  }

  my_bus_ = std::make_unique<MyBus>(port, baud_rate);
  my_bus_->Init();
  arm_ctrl_ = std::make_shared<MyRobotArmCtrl>(*my_bus_);
}
```

如果驱动对象需要持有总线生命周期，把 `my_bus_` 加到 `ActuatorManager` 的私有成员中，并在 `Shutdown()` 中关闭总线。

需要注意：

- `joint_names_` 的顺序必须与 `joints` 配置顺序一致。
- `joint_limits` 配置单位是 **度**，进入驱动前是否转弧度由自定义分支决定；`geniarm` 分支会先转成弧度。
- 配置中的 `ctrl_mode` 只支持 `MIT` / `POS` / `VEL`；不填或写 `MIT` 时保持默认。当前这段解析写在 `geniarm` 分支内，自定义分支需要自己调用 `SetCtrlMode()`。
- 夹爪段是可选的，没有 `actuator.gripper` 时不会创建夹爪。
- 如果总线和夹爪共用（例如同一路 CAN），不要重复创建总线对象。

## 3. 加入 CMake 构建

驱动源码不是自动收集的，需要在 `actuator/CMakeLists.txt` 的 `GENISYS_CORE_BUILD_ARM` 分支中显式加入。当前只支持 `geniarm`：

```cmake
if(GENISYS_CORE_BUILD_ARM STREQUAL "geniarm")
  add_subdirectory(third_party/dm_driver)
  set(ARM_SOURCES
    src/actuator_manager.cc
    src/geniarm_ctrl.cc
    src/geniarm_gripper.cc
  )
  set(ARM_LIBS dm_driver)
elseif(GENISYS_CORE_BUILD_ARM STREQUAL "my_robot")
  add_subdirectory(third_party/my_bus)
  set(ARM_SOURCES
    src/actuator_manager.cc
    src/my_robot_ctrl.cc
  )
  set(ARM_LIBS my_bus)
else()
  message(FATAL_ERROR "Unsupported GENISYS_CORE_BUILD_ARM: ${GENISYS_CORE_BUILD_ARM}")
endif()
```

然后配置并构建：

```bash
./build.sh -DGENISYS_CORE_BUILD_ARM=my_robot
```

或直接使用 CMake：

```bash
cmake -B build -DGENISYS_CORE_BUILD_ARM=my_robot
cmake --build build --target install --parallel $(nproc)
```

补充说明：

- `GENISYS_CORE_BUILD_ARM` 是**型号选择**，同一时刻只编译一个型号的驱动。
- 新增头文件会随 `install(DIRECTORY include/)` 自动安装，不需要单独注册。
- 驱动依赖第三方库时，把库的 `add_subdirectory()` 放在对应型号分支里，避免未使用型号也参与编译。
- 串口驱动库 `third_party/serial` 仍保留；如果 `my_robot` 走 UART，在分支中加入：
  ```cmake
  add_subdirectory(third_party/serial)
  set(ARM_LIBS serial)
  ```
- `GENISYS_CORE_BUILD_EXAMPLES` 同时控制本 SDK 和 `dm_driver` 的示例；示例不是运行时所必需。

## 4. 适配 yaml 管理：驱动配置

创建 `my_robot_actuator_cfg.yaml`，按实际硬件填写：

```yaml
actuator:
  arm:
    name: "my_robot"        # 与 ActuatorManager 新增分支的 name 一致
    protocol:
      port: "can0"           # 或 ttyUSB0 / eth0，具体字段由自定义分支读取
      baud_rate: 1000000     # 串口等总线才需要
    ctrl_mode: "MIT"         # MIT / POS / VEL
    joint_limits:
      lower: [-180, -90, -180, -180, -180, -180]
      upper: [180, 90, 180, 180, 180, 180]
    joints:
      - name: "joint1"
        motor_id: 0x01
        feedback_id: 0x11
        model: "MY_MODEL"
      - name: "joint2"
        motor_id: 0x02
        feedback_id: 0x12
        model: "MY_MODEL"
      # ... 共 6 个关节
  gripper:
    name: "my_gripper"
    protocol:
      port: "can0"
    motor_id: 0x07
    feedback_id: 0x17
    model: "MY_MODEL"
```

字段说明：

- `joint_limits` 必须与运动学配置中的限位一致，单位是度。
- `joints` 必须为 6 项，顺序与 `ArmCtrlBase` 的关节顺序一致。
- `feedback_id` 是否必须为 `motor_id + 0x10` 取决于驱动实现；`geniarm` 分支有该约束。
- 夹爪段可以省略；省略后 `GripperController` 也不会创建。

## 5. 配置运动学参数

创建 `my_robot_body_cfg.yaml`，填写 MDH 参数、关节限位和 MIT 参数：

```yaml
robot:
  name: "my_robot"
  sample_frequency: 500.0      # Hz，轨迹规划离散频率
  task_context_size: 10        # 可选，任务历史条数
  joint_limits:
    lower: [-180, -90, -180, -180, -180, -180]
    upper: [180, 90, 180, 180, 180, 180]
  trajectory_mode: "quintic"   # quintic / cubic / lspb
  kinematics:
    ik_mode: "UR"              # UR -> ParallelWrist；Piper -> SphericalWrist
    MDH:
      alpha: [0.0, -90.0, 0.0, 0.0, -90.0, 90.0]   # 度
      a: [0.0, 0.02, 0.26, 0.24668, 0.075, 0.0]    # m
      d: [0.1459, 0.0, 0.0, 0.0, 0.0, 0.0335]      # m
      offset: [0.0, -180.0, 166.64, 13.36, 90.0, 0.0]  # 度
  dynamics: "dynamics/my_robot_dynamics.yaml"
  joints:
    - name: "joint1"
      MIT: { stiffness: 120.0, damping: 5.0 }
    - name: "joint2"
      MIT: { stiffness: 120.0, damping: 5.0 }
    - name: "joint3"
      MIT: { stiffness: 120.0, damping: 5.0 }
    - name: "joint4"
      MIT: { stiffness: 18.0, damping: 2.0 }
    - name: "joint5"
      MIT: { stiffness: 18.0, damping: 2.0 }
    - name: "joint6"
      MIT: { stiffness: 18.0, damping: 2.0 }
  tool:
    frame: "default"
    xyz: [0.0, 0.0, 0.0]        # TCP 相对法兰的位置，m
    rpy: [0.0, 0.0, 0.0]        # ZYX 欧拉角，度
  # gripper:                     # 可选，需要夹爪时填写
  #   name: "my_gripper"
  #   max_range: 1.95
  #   MIT: { stiffness: 8.0, damping: 1.0 }
```

字段说明：

- `robot.name` 目前只作配置标识，不参与驱动选择；驱动选择看 `actuator.arm.name`。
- `sample_frequency` 是轨迹采样频率，不必等于顶层控制循环的 `control_frequency`，但建议不高于它。
- `task_context_size` 可选，默认 10。
- `trajectory_mode` 只接受 `quintic` / `cubic` / `lspb`，非法值会回退到 `quintic`。
- `ik_mode` 只接受 `UR` / `Piper`；`Piper` 对应球腕解析解，`UR` 对应平行腕解析解。
- MDH 的 `alpha` / `offset` 单位是度，`a` / `d` 单位是米；数组必须为 6 项。
- `tool` 描述 TCP 相对法兰的位姿；`Move*` 接口输入的是 TCP 位姿，进入 IK 前会自动去掉工具变换。
- `dynamics` 缺省或删除时不做重力补偿；填写时路径相对于运行时的 `./cfg/`。
- `gripper` 段需要与执行器配置中的夹爪段同时存在。

## 6. 配置动力学参数（可选）

`dynamics.yaml` 用于重力补偿和力矩前馈。当前控制器的重力补偿主要使用 `mass` 与 `com`，惯性和摩擦不是重力补偿的必需项：

```yaml
dynamics:
  gravity: [0.0, 0.0, -9.81]

  links:
    - name: "link1"
      mass: 0.0
      com: [0.0, 0.0, 0.0]
      inertia: { ixx: 0.0, iyy: 0.0, izz: 0.0, ixy: 0.0, ixz: 0.0, iyz: 0.0 }
    # ... 共 6 个连杆

  # 末端工具会融合到指定连杆
  tool:
    attach_to: "link6"
    mass: 0.0
    com: [0.0, 0.0, 0.0]
    inertia: { ixx: 0.0, iyy: 0.0, izz: 0.0, ixy: 0.0, ixz: 0.0, iyz: 0.0 }

  # 预留字段，当前版本不参与重力补偿计算
  friction:
    joint1: { coulomb: 0.0, viscous: 0.0 }
    joint2: { coulomb: 0.0, viscous: 0.0 }
    joint3: { coulomb: 0.0, viscous: 0.0 }
    joint4: { coulomb: 0.0, viscous: 0.0 }
    joint5: { coulomb: 0.0, viscous: 0.0 }
    joint6: { coulomb: 0.0, viscous: 0.0 }
```

约定：

- `links` 必须为 6 项，顺序与 MDH 连杆顺序一致。
- `mass` 单位 kg，`com` 单位 m，`inertia` 单位 kg·m²。
- 坐标均在对应连杆坐标系 `{i}` 中表达，与 MDH 约定一致。
- `tool.attach_to` 使用 `links[].name`；工具质量/质心会融合到该连杆。
- 如果只需要重力补偿，惯性和摩擦可以全部置 0。
- 关节摩擦字段目前只是配置预留，控制器不会读取。

## 7. 配置加载与示例

SDK 的两个管理器都只接收**配置文件名**，内部拼接 `./cfg/`：

```cpp
ActuatorManager actuator_manager("my_robot_actuator_cfg.yaml");
ArmManager arm_manager("my_robot_body_cfg.yaml");
```

因此配置文件统一放在工作目录的 `cfg/` 下：

```text
cfg/
├── my_robot_actuator_cfg.yaml
├── my_robot_body_cfg.yaml
└── dynamics/
    └── my_robot_dynamics.yaml
```

`dynamics` 字段的路径同样相对 `./cfg/`，例如：

```yaml
dynamics: "dynamics/my_robot_dynamics.yaml"
```

构建时 `cfg/` 会安装到 `build/cfg/`，可以从 `build/` 目录运行示例：

```bash
./build.sh
cd build
./real_arm_sample
```

`example/` 下的示例默认加载 `geniarm_*` 配置；适配 `my_robot` 时，修改示例中的文件名，或参考示例自行编写入口。上层应用集成时，同样需要在运行目录提供 `cfg/`，并自行调用 `ActuatorManager` / `ArmManager`。

## 8. 验证与排查

建议按以下顺序验证：

1. 配置能成功加载，日志中打印的关节名、MDH、限位、MIT 参数与 yaml 一致。
2. `Enable()` 后能读到关节反馈，方向与零位正确。
3. 低速执行一次 `MoveJ`，确认关节顺序、减速比、方向没有错。
4. 检查 `MoveL` / `MoveJPose` 的 TCP 位姿是否已经包含 `tool` 偏移。
5. 需要重力补偿时，确认 `dynamics` 路径正确，且日志中打印了 dynamics 初始化信息。
6. 最后再逐步提高速度，并确认关节限位和急停逻辑有效。

常见错误：

| 现象 | 排查方向 |
|:---|:---|
| `Unsupported arm: my_robot` | `ActuatorManager` 中的 `name` 分支与 yaml 不一致 |
| `Failed to initialize arm controller` | 驱动 `Init()` 返回了 `false`，检查总线/串口是否正确打开 |
| `Open erro on port` | CAN/UART 端口名、权限或总线未启动 |
| `Failed to load file ./cfg/...` | 工作目录不是包含 `cfg/` 的目录，或配置文件名不正确 |
| `trajectory point joint[i] out of limit` | 限位、MDH 零位偏移或关节方向配置不正确 |
| 没有重力补偿 | `dynamics` 未配置、路径错误，或质量/质心全为 0 |
| IK 报错或解异常 | `ik_mode` 只能是 `UR` / `Piper`，并检查 MDH 与限位 |
