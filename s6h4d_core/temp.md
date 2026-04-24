``` CPP
#include "arm_ctrl_base.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>

class RosSimArmCtrl : public ArmCtrlBase {
 public:
  RosSimArmCtrl(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

  bool Init() override {
    // 1. 创建发布者：发给仿真器
    pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_joint_position", 10);

    // 2. 创建订阅者：从仿真器读回状态
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&RosSimArmCtrl::JointStateCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "RosSimArmCtrl Initialized.");
    return true;
  }

  // 下发：把指令发布出去
  bool SetJointState(const ArmMotion::JointState_t& joint_cmd) override {
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    for (int i = 0; i < 6; ++i) {
      msg->data.push_back(joint_cmd.q[i]);
    }
    pub_->publish(std::move(msg));
    return true;
  }

  // 读取：从 Subscriber 缓存的最新数据中读取
  bool GetJointState(ArmMotion::JointState_t& joint_state) override {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 将订阅到的最新 ROS 数据拷贝给输出参数
    joint_state = latest_received_state_;
    return true;
  }

 private:
  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 假设 ROS 消息里的顺序和你的算法顺序一致
    for (size_t i = 0; i < 6 && i < msg->position.size(); ++i) {
      latest_received_state_.q[i] = msg->position[i];
      latest_received_state_.dq[i] = msg->velocity[i];
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  ArmMotion::JointState_t latest_received_state_{};
  std::mutex data_mutex_;
};
```