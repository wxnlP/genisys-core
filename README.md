![](https://tonmoon.obs.cn-east-3.myhuaweicloud.com/img/tonmoon/5b3e8e7458ebe0ff134f3e1d0bcf5fc2.jpg)


## ROS2 示例

```
# 安装依赖

sudo apt install ros-humble-serial-driver

```

```
# 指令
# --- 0x11 --- 0x12 --- 0x21 --- 0x31 --- 0x44 ---
#   夹爪控制  关节角度控制 混合控制 单关节控制  完整控制  

ros2 topic pub /s6h4d/joint_cmd example_ros2/msg/S6h4dJointAngle "{cmd: 0x31, joint_index: 5, angle: [20.0, 20.0, 0.0, 0.0, 0.0, 0.0], vel: [10.0, 20.0, 20.0, 20.0, 20.0, 20.0], accel: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0], paw: 0}" --once 

```

```
# 各关节稳定角度范围

J1  -140 ~ 180
J2  -60  ~ 100
J3   30  ~ 190
J4  -150 ~ 150
J5  -30  ~ 120
J6  -270 ~ 270

```