#!/bin/bash

# --- 1. 默认值设置 (使用弧度) ---

# 定义所有六个关节的默认目标位置（弧度）
# 注意：这些默认值是示例，请根据您的机器人实际零位和目标位进行调整。
DEFAULT_SHOULDER_POS=${1:-0.0}
DEFAULT_UPPER_ARM_POS=${2:-0.0}
DEFAULT_ELBOW_POS=${3:-1.88}
DEFAULT_WRIST1_POS=${4:-0.0}
DEFAULT_WRIST2_POS=${5:-0.0}
DEFAULT_WRIST3_POS=${6:-0.0}


# --- 2. 构建 JSON 数据 ---

JSON_DATA=$(cat <<EOF
{
  "joints": [
    {
      "name": "shoulder_actuator",
      "position": $DEFAULT_SHOULDER_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    },
    {
      "name": "upper_arm_actuator",
      "position": $DEFAULT_UPPER_ARM_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    },
    {
      "name": "elbow_actuator",
      "position": $DEFAULT_ELBOW_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    },
    {
      "name": "wrist1_actuator",
      "position": $DEFAULT_WRIST1_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    },
    {
      "name": "wrist2_actuator",
      "position": $DEFAULT_WRIST2_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    },
    {
      "name": "wrist3_actuator",
      "position": $DEFAULT_WRIST3_POS,
      "velocity": 0.0,
      "effort": 0.0,
      "stiffness": 0.0,
      "damping": 0.0
    }
  ]
}
EOF
)

# --- 3. 发送 JSON 数据到服务器 ---

curl -i \
  -H 'content-type:application/json' \
  -X POST 'http://127.0.0.1:50080/channel/%2Fmujoco%2Fjoint_cmd/pb%3Aaimrt.protocols.sensor.JointCommandArray' \
  -d "$JSON_DATA"