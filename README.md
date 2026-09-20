# Genisys Core

> Genisys Core——面向六轴协作机械臂的模块化运动控制核心库，覆盖从**硬件驱动 → 运动规划 → 实时控制**的完整软件栈。核心设计目标：**层间解耦、算法可插拔、配置驱动装配**——新增机械臂硬件、轨迹算法、机器人构型时，尽量不触碰已有代码。
>
> **命名由来**：Genisys = genimind + geniarm + genesis。genimind 是作者的第一台轮式机器人（最初的想法），geniarm 是第一台机械臂（想法落地），genisys 则是两者沉淀下来的一套通用核心框架（创世）。

---

## 设计理念

SDK 围绕三个原则构建：

1. **接口解耦**：层与层之间只依赖纯虚接口（`ArmCtrlBase`、`JointPathBase`、`TimeScalerBase` 等），不依赖具体实现；适配层只持有接口指针，不认识任何具体类。
2. **策略化算法**：路径、时间缩放等算法以策略模式组合进轨迹，新增一种算法 = 继承抽象基类 + 注册一个工厂分支，两处改动、零侵入。
3. **配置驱动装配**：对象由 YAML 描述装配，同一套运动规划代码只需更换配置文件即可适配不同机械臂。

---

## 目录结构

```
genisys-core
├── actuator/                     # ── 驱动抽象层（静态库 actuator_controller）──
│   ├── include/
│   │   ├── arm_ctrl_base.h           # 机械臂驱动纯虚接口
│   │   ├── gripper_ctrl_base.h       # 夹爪驱动纯虚接口
│   │   ├── actuator_manager.h        # 驱动层 YAML 工厂
│   │   ├── geniarm_ctrl.h            # GeniArm 驱动实现（CAN FD / DM 电机）
│   │   ├── s6h4d_ctrl.h              # S6H4D 驱动实现（串口）
│   │   └── geniarm_gripper.h         # GeniArm 夹爪驱动实现
│   ├── src/
│   └── third_party/                  # dm_driver / serial 等第三方依赖
│
├── arm_control/                  # ── 运动规划层（静态库 arm_controller）──
│   ├── include/
│   │   ├── arm_type.h                # 公共数据结构（Joint6Dof / Pose6Dof ...）
│   │   ├── kinematic.h               # 运动学求解器（FK / IK，双 IK 模式）
│   │   ├── jacobian.h                # 雅可比矩阵（速度/加速度正逆解）
│   │   ├── arm_controller.h          # 高层运动接口（MoveJ/L/Multi/Servo ...）
│   │   ├── gripper_controller.h      # 夹爪位置控制
│   │   ├── joint_path_base.h         # 关节路径抽象基类
│   │   ├── joint_path.h              # 单段关节直线路径
│   │   ├── joint_multi_point_path.h  # 多点关节路径（抛物线融合）
│   │   ├── cartesian_path_base.h     # 笛卡尔路径抽象基类
│   │   ├── cartesian_linear_path.h   # 单段笛卡尔直线路径
│   │   ├── cartesian_multi_point_path.h # 多点笛卡尔路径（圆弧融合 + Slerp）
│   │   ├── joint_trajectory.h        # 关节轨迹 = 路径 + 时间缩放
│   │   ├── cartesian_trajectory.h    # 笛卡尔轨迹 = 路径 + 时间缩放
│   │   ├── time_scaler_base.h        # 时间缩放抽象基类
│   │   ├── cubic_time_scaler.h       # 三次多项式时间缩放
│   │   ├── lspb_time_scaler.h        # 梯形加减速时间缩放
│   │   ├── quintic_time_scaler.h     # 五次多项式时间缩放
│   │   └── arm_manager.h             # 规划层 YAML 工厂
│   └── src/
│
│
├── cfg/                          # 配置文件（驱动参数 + 机械臂参数）
├── example/                      # 使用示例
├── tools/                        # 轨迹可视化脚本
├── ADAPTATION.md                 # 机械臂适配指南
└── VERSION
```

---

## 架构与依赖方向

SDK 采用**纵向三层解耦**结构，层间只通过接口与布局一致的数据结构通信：

```
┌───────────────────────────────────────────────────────────────┐
│           适配层  由中间件模块自行实现（本项目为 ControlModule）    │
│   实时控制循环：读取驱动反馈 → 取规划指令 → 下发驱动指令              │
└───────────────┬────────────────────────────────┬──────────────┘
                │ 依赖接口 / 高层类                 │ 依赖纯虚接口
        ┌───────▼──────────┐          ┌───────────▼───────────┐
        │   运动规划层       │          │     驱动抽象层         │
        │   arm_controller  │          │  actuator_controller  │
        │   静态库           │          │      静态库            │
        └───────────────────┘          └───────────────────────┘

     规划层 ⇄ 驱动层：零编译期依赖，仅通过内存布局一致的数据结构交换
```

最关键的约束是：**`arm_control` 与 `actuator` 是两个完全独立的静态库**。规划层不知道 CAN、串口、EtherCAT 的存在；驱动层不知道路径、轨迹、运动学的存在。两者通过布局一致的数据结构（`JointCommand` / `JointState`）交换数据，某一层内部演进不影响另一层重新编译。

---

## 核心解耦设计

### 1. 接口解耦：层间只见抽象

SDK 中所有可替换组件都收敛到一组抽象接口：

| 抽象接口 | 所属层 | 职责 |
|----------|--------|------|
| `ArmCtrlBase` | 驱动层 | 机械臂指令下发 / 状态读取（`SetJointState` / `GetJointState` / `SetCtrlMode`） |
| `GripperCtrlBase` | 驱动层 | 夹爪指令下发 / 状态读取 |
| `JointPathBase` / `CartesianPathBase` | 规划层 | 路径几何（`EvaluatePos` / `EvaluateTangent`） |
| `TimeScalerBase` | 规划层 | 时间分配（`Plan` / `Evaluate` / `GetDuration`） |
| `KinematicSolver` | 规划层 | 运动学求解（FK / IK），内置双 IK 模式 |

适配层（本项目中即 `ControlModule`）通过 `shared_ptr` 持有这些接口，控制循环里只做三件事——读反馈、取指令、发指令——从不关心具体驱动走的是 CAN 还是串口，也不关心轨迹是如何规划的。

### 2. 策略模式：算法可插拔

轨迹由**路径**与**时间缩放**两个策略对象组合而成：

```
JointPathBase ─────┐
                   ├──▶ JointTrajectory ──▶ ArmController 采样线程 ──▶ JointCommand
TimeScalerBase ────┘

CartesianPathBase ─┐
                   ├──▶ CartesianTrajectory ──▶ IK ──▶ JointCommand
TimeScalerBase ────┘
```

- **路径**：关节空间直线、多点抛物线融合；笛卡尔空间直线、多点圆弧融合 + Slerp，各自独立实现；
- **时间缩放**：Cubic / LSPB / Quintic 三种，由 `ArmController::CreateTimeScaler()` 工厂按 `TrajectoryMode` 选择，新增平滑算法只需加一个枚举值和一个工厂分支；
- **运动学**：`KinematicSolver` 内置 `SphericalWrist` / `ParallelWrist` 两种 IK 求解模式，可继续扩展新的求解器。

所有现有路径与轨迹类只依赖抽象基类，因此新增算法对已有代码零侵入。

### 3. 配置驱动：YAML 工厂装配

对象装配链路如下：

```
geniarm_body_cfg.yaml ──▶ ArmManager + ActuatorManager
    ├── actuator_config ──▶ ActuatorManager ──▶ name 分支 ──▶ ArmCtrlBase 实现
    └── arm_config     ──▶ ArmManager ──▶ KinematicSolver + ArmController
```

- `ActuatorManager` 依据 YAML 中 `actuator.arm.name` 选择驱动实现（`geniarm` / `s6h4d`），新增硬件只需新增一个 name 分支；
- `ArmManager` 读取 MDH 参数、关节限位、IK 模式、控制参数并构造规划器；
- 换一台不同构型的机械臂，通常**只改 YAML、不改代码**。

### 4. 构建级解耦：集成层可整体替换

SDK 产出三个独立静态库目标：

| 目标 | 内容 | 依赖关系 |
|------|------|----------|
| `actuator_controller` | 驱动实现 + 工厂 | 不依赖规划层 |
| `arm_controller` | 运动学 + 路径 + 轨迹 + 规划器 | 不依赖驱动层 |
| `actuator_controller` | 驱动实现（DM 电机 / CAN） | 不依赖算法层 |
| `arm_controller` | 运动学 + 规划 + 控制器 | 不依赖驱动层 |

示例程序（`example/`）演示了如何直接组合 `ActuatorManager` 与 `ArmManager` 自建控制回路；接入 ROS2、AimRT 等中间件时，按同样方式在中间件模块里编写适配层（参考 [genisys](https://github.com/wxnlP/genisys)）。

---

## 可拓展性速查

| 想做什么 | 需要改哪里 | 影响范围 | 难度 |
|----------|-----------|----------|------|
| 适配新的机械臂硬件 | 继承 `ArmCtrlBase` / `GripperCtrlBase`，`ActuatorManager` 加 name 分支，新增 YAML | 仅驱动层 | ★★☆ |
| 新增轨迹平滑算法 | 继承 `TimeScalerBase`，加 `TrajectoryMode` 枚举值与工厂分支 | 仅规划层 | ★☆☆ |
| 新增路径类型 | 继承 `JointPathBase` / `CartesianPathBase` | 仅规划层 | ★★☆ |
| 适配新构型机械臂 | 配置 MDH / 限位 / IK 模式，必要时扩展 `KinematicSolver` | 配置文件或规划层 | ★★☆ |
| 接入 ROS2 / AimRT 中间件 | 不链接 `genisys_core`，复用两个底层库写通信层 | 仅集成层 | ★☆☆ |

机械臂硬件适配的详细步骤见 [ADAPTATION.md](./ADAPTATION.md)。

---

## 扩展示例：新增一个时间缩放算法

以新增"Jerk 限制"平滑算法为例，只需两步：

1. 继承 `TimeScalerBase` 实现算法：

```cpp
class JerkLimitedTimeScaler : public TimeScalerBase {
 public:
  bool Plan(const PlanConfig& config) override { /* ... */ }
  bool Evaluate(double t, State& state) const override { /* ... */ }
  double GetDuration() const override { /* ... */ }
};
```

2. 在 `ArmController` 中注册（新增枚举值 + 工厂分支）：

```cpp
enum class TrajectoryMode { LSPB, CUBIC, QUINTIC, JERK_LIMITED /* 新增 */ };

std::unique_ptr<TimeScalerBase> ArmController::CreateTimeScaler(...) {
  switch (traj_planning_mode) {
    /* ... 已有分支 ... */
    case TrajectoryMode::JERK_LIMITED:
      return std::make_unique<JerkLimitedTimeScaler>();
  }
}
```

调用方无需任何改动：`arm->MoveJ(target, v, a)` 依旧生效。

---

## 配置文件体系

驱动层与算法层配置分离，是"同代码、多机型"的基础。

**驱动层 `xxx_actuator_cfg.yaml`：**

```yaml
actuator:
  arm:
    name: "geniarm"            # 驱动类型标识，ActuatorManager 据此选择实现
    protocol: { port: "can0" }
    ctrl_mode: "MIT"           # MIT / POS / VEL
    joints:
      - name: "joint1"
        motor_id: 0x01
        model: "DM4340P"
```

**算法层 `xxx_body_cfg.yaml`：**

```yaml
robot:
  name: "geniarm"
  kinematics:
    ik_mode: "UR"              # SphericalWrist
    MDH:
      alpha: [0, -90, 0, 0, -90, 90]
      a: [0, 0.02, 0.26, 0.24668, 0.075, 0]
      d: [0.1207, 0, 0, 0, 0, 0.0265]
      offset: [0, -180, 166.64, 13.36, 90, 0]
  joints:
    - name: "joint1"
      MIT: { stiffness: 120, damping: 5 }
```

---

## 编译

主要依赖下载：

| 库 | 用途 | 获取方式 |
|----|------|----------|
| Eigen3 | 矩阵运算 | 系统安装（`apt install libeigen3-dev`） |
| yaml-cpp | YAML 解析 | 系统安装（`apt install libyaml-cpp-dev`） |

编译：

```bash
./build.sh
```

---

## 快速开始

```cpp
#include "actuator_manager.h"
#include "arm_manager.h"

int main() {
    ActuatorManager actuator_manager("geniarm_actuator_cfg.yaml");
    ArmManager arm_manager("geniarm_body_cfg.yaml");

    auto arm = manager.GetArmController();
    auto gripper = manager.GetGripperController();

    manager.Enable();
    manager.Start();

    // 夹爪控制
    gripper->PosControl(1.0, 0.5, 1.0, 0.0);
    gripper->WaitForCompletion();

    // 关节空间运动（MoveJ / MoveJMulti / ServoJ 同时支持位姿重载）
    ArmMotion::Joint6Dof target{0, 1.0, -1.0, 0, 0, 0};
    arm->MoveJ(target, 0.5, 0.3);
    arm->WaitForCompletion();

    // 笛卡尔空间运动
    ArmMotion::Pose6Dof pose;
    arm->GetCurrentPose(pose);
    pose.y += 0.1;
    arm->MoveL(pose, 0.5, 0.3);
    arm->WaitForCompletion();

    manager.Disable();
    return 0;
}
```

完整的示例参考 `example/real_arm_sample.cc`。

---

## 项目引用与使用方式

1. **纯 CPP 集成**：链接 `actuator_controller` + `arm_controller`，自行编写控制回路（见 `example/real_arm_sample.cc`）；
2. **中间件集成**：适配 ROS2、AimRT 等通信中间件时，基于 `arm_controller` 与 `actuator_controller` 两个独立库目标实现通信架构（参考 [genisys](https://github.com/wxnlP/genisys)）。
