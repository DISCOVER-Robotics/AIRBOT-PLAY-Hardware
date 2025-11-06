#include <algorithm>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/eef.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

class AirbotExampleNode : public rclcpp::Node {
 public:
  AirbotExampleNode(size_t size = 0) : Node("airbot_play_pos_ctrl") {
    RCLCPP_INFO(this->get_logger(), "airbot_play_pos_ctrl started");

    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_command", 10, std::bind(&AirbotExampleNode::joint_state_callback, this, std::placeholders::_1));

    executor = airbot::hardware::AsioExecutor::create(8);
    arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                           MotorType::DM, EEFType::NA, MotorType::NA>();

    if (!arm->init(executor->get_io_context(), "can0", 250_hz)) {
      std::cerr << "Failed to initialize arm" << std::endl;
    }

    arm->enable();

    arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

    running = true;

    t = std::thread([&]() {
      while (running) {
        // std::cout << arm->state().is_valid << std::endl;
        std::this_thread::sleep_for(4ms);
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        if (arm && arm->state().is_valid) {
          auto state = arm->state();
          std::vector<double> pos(state.pos.begin(), state.pos.end());
          std::vector<double> vel(state.vel.begin(), state.vel.end());
          std::vector<double> eff(state.eff.begin(), state.eff.end());

          message.position = pos;
          message.velocity = vel;
          message.effort = eff;

          try {
            joint_publisher_->publish(message);
          } catch (std::exception& e) {
            std::cerr << "Failed to publish state data: " << e.what() << std::endl;
          }
        }
      }
    });
    return_zero();
    arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  }

  ~AirbotExampleNode() {
    running = false;
    t.join();
    if (arm) {
      arm->disable();
      arm->uninit();
    }
  }

  bool is_arm_arrive(const std::array<double, 6> arm_target, const std::array<double, 6> arm_pose) {
    if (std::abs(arm_target[0] - arm_pose[0]) <= 0.02 && std::abs(arm_target[1] - arm_pose[1]) <= 0.02 &&
        std::abs(arm_target[2] - arm_pose[2]) <= 0.02 && std::abs(arm_target[3] - arm_pose[3]) <= 0.02 &&
        std::abs(arm_target[4] - arm_pose[4]) <= 0.02 && std::abs(arm_target[5] - arm_pose[5]) <= 0.02) {
      return 1;
    } else {
      return 0;
    }
  }

  void return_zero() {
    while (true) {
      if (!arm->state().is_valid) {
        arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
      } else {
        if (!is_arm_arrive({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, arm->state().pos)) {
          arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
        } else {
          break;
        }
      }
      std::this_thread::sleep_for(4ms);
    }
  }

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6 && arm) {
      for (int i = 0; i < 6; ++i) {
        arm_cmd[i] = msg->position[i];
      }
    }

    while (true) {
      if (!arm->state().is_valid) {
        arm->mit(arm_cmd, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 {10.0, 10.0, 250.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      } else {
        if (!is_arm_arrive(arm_cmd, arm->state().pos)) {
          arm->mit(arm_cmd, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                   {10.0, 10.0, 250.0, 1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        } else {
          break;
        }
      }
      std::this_thread::sleep_for(4ms);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor;
  std::unique_ptr<airbot::hardware::Arm<6>> arm;
  bool running;
  std::thread t;
  std::array<double, 6> arm_cmd;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirbotExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
