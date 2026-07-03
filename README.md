# Robotic Arm SDK

> 面向六轴协作机械臂的模块化运动控制 SDK，提供从**硬件驱动 → 运动规划 → 实时控制**的完整软件栈。核心设计理念为**分层抽象与接口解耦**，支持适配不同构型的机械臂硬件。

---

## 目录结构

```
robotic_arm_sdk
├── actuator                   # ── 驱动抽象层 ──
│   ├── include
│   │   ├── arm_ctrl_base.h      # 机械臂驱动抽象基类
│   │   ├── gripper_ctrl_base.h  # 夹爪驱动抽象基类
│   │   ├── actuator_manager.h   # 执行器管理
│   ├── src                      # 各驱动实现
│   └── third_party							 # 第三方依赖
│
├── arm_control                # ── 运动规划层 ──
│   ├── include
│   │   ├── arm_type.h           			# 公共数据结构
│   │   ├── kinematic.h          			# 运动学求解器（FK / IK）
│   │   ├── jacobian.h           			# 雅可比矩阵（速度/加速度正逆解）
│   │   ├── arm_controller.h     			# 高层运动接口
│   │   ├── gripper_controller.h 			# 夹爪位置控制
│   │   ├── joint_path_base.h    			# 关节空间路径抽象基类
│   │   ├── joint_path.h         			# 单段关节直线路径
│   │   ├── joint_multi_point_path.h  # 多点关节路径（抛物线融合）
│   │   ├── cartesian_path_base.h     # 笛卡尔空间路径抽象基类
│   │   ├── cartesian_linear_path.h   # 单段笛卡尔直线路径
│   │   ├── cartesian_multi_point_path.h  # 多点笛卡尔路径（圆弧融合 + Slerp）
│   │   ├── joint_trajectory.h         # 关节轨迹（路径 + 时间缩放）
│   │   ├── cartesian_trajectory.h     # 笛卡尔轨迹（路径 + 时间缩放）
│   │   ├── time_scaler_base.h         # 时间缩放抽象基类
│   │   ├── cubic_time_scaler.h        # 三次多项式时间缩放
│   │   ├── lspb_time_scaler.h         # 梯形加减速时间缩放
│   │   ├── quintic_time_scaler.h      # 五次多项式时间缩放
│   │   └── arm_manager.h             
│   └── src
├── cfg/                       # ── 配置文件 ──
│   ├── robo_manager.yaml        	 # 顶层配置（引用 actuator + arm 配置）
│   ├── geniarm_actuator_cfg.yaml  # GeniArm 电机驱动参数
│   ├── geniarm_body_cfg.yaml      # GeniArm 运动学/动力学参数
│   ├── s6h4d_actuator_cfg.yaml    # S6H4D 串口驱动参数
│   ├── s6h4d_body_cfg.yaml        # S6H4D 运动学参数
│   └── dynamics.yaml              # 动力学参数（质量/惯量/摩擦）
│
├── example/                   # 使用示例
│
├── tools/                     # 辅助工具
│   ├── cartesian_plot.py      # 笛卡尔轨迹可视化
│   └── joints_pos_plot.py     # 关节轨迹可视化
│
└── VERSION                    # 版本号
```

---

## 程序架构

SDK 采用**三层纵向解耦**设计，层间通过纯虚接口与 YAML 工厂解耦，每一层均可独立扩展。

```
 ┌──────────────────────────────────────────────────┐
 │              集成调度层 robo_manager               │
 │  RoboManager 协调驱动层与规划层，驱动实时控制循环      │
 ├──────────────────────────────────────────────────┤
 │              运动规划层 arm_control                │
 │  ArmController 运动学 + 路径 + 轨迹规划算法          │
 ├──────────────────────────────────────────────────┤
 │              驱动抽象层 actuator                   │
 │  ArmCtrlBase / GripperCtrlBase 硬件抽象接口        │
 └──────────────────────────────────────────────────┘
```

**扩展切入点：**

| 层 | 扩展方向 | 难度 |
|----|----------|------|
| 驱动抽象层 | 适配不同机械臂硬件（CAN FD / 串口 / EtherCAT ...） | ★★☆ |
| 运动规划层 | 新增路径/时间缩放算法，运动学求解器 | ★☆☆ |
| 集成调度层 | 原则上无需修改，直接复用 | — |

---

### 驱动抽象层 — 适配不同硬件

`ArmCtrlBase` 定义机械臂驱动的纯虚接口，`GripperCtrlBase` 定义夹爪接口。现有 `GeniArmCtrl`（CAN FD / DM 电机）和 `S6h4dCtrl`（串口）两种实现。

**扩展方式：** 继承 `ArmCtrlBase` 实现核心方法，在 `ActuatorManager` 中增加 `name` 分支即可。详情见 [`ADAPTATION.md`](./ADAPTATION.md)。

### 运动规划层 — 可插拔算法

`ArmController` 对外暴露 `MoveJ` / `MoveL` / `MoveJMulti` / `MoveLMulti` 等接口。内部通过策略模式组合：

- **路径**：`JointPathBase` / `CartesianPathBase` 抽象，现有直线、多点抛物线融合、圆弧融合实现；
- **时间缩放**：`TimeScalerBase` 抽象，现有 Cubic / LSPB / Quintic 实现，可自由拓展其他轨迹平滑算法；
- **运动学**：`KinematicSolver` 支持 `SphericalWrist` 和 `ParallelWrist` 两种 IK 模式。

### 集成调度层 — 实时控制循环

`RoboManager` 原则上只是一个调度层示例，强烈推荐使用基于AimRT实现的[robotic_arm_aimrt](https://github.com/wxnlP/robotic_arm_aimrt)通信系统。

### 配置文件体系

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
| yaml-cpp | YAML解析 | 系统安装（`apt install libyaml-cpp-dev`） |

编译：

```bash
./build.sh
```

---

## 快速开始

```cpp
#include "robo_manager.h"

int main() {
    RoboManager manager("robo_manager.yaml");

    auto arm = manager.GetArmController();
    auto gripper = manager.GetGripperController();

    manager.Enable();
    manager.Start();

    // 夹爪控制
    gripper->PosControl(1.0, 0.5, 1.0, 0.0);
    gripper->WaitForCompletion();

    // 关节运动
    ArmMotion::Joint6Dof target{0, 1.0, -1.0, 0, 0, 0};
    arm->MoveJ(target, 0.5, 0.3);
    arm->WaitForCompletion();

    // 笛卡尔运动
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

1. 直接使用纯 CPP SDK（即使用`RoboManager`），直接链接静态库`robotic_arm_sdk`;
2. 适配 ROS2、AimRT等通信中间件，抛弃`RoboManager`，基于单独的静态库目标`arm_controller`和`actuator_controller`做通信架构适配。（参考[robotic_arm_aimrt](https://github.com/wxnlP/robotic_arm_aimrt)）