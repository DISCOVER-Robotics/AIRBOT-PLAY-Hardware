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
  AirbotExampleNode() : Node("airbot_play_free_drive") {
    this->declare_parameter<std::string>("i", "can0");
    std::string can_interface = this->get_parameter("i").as_string();
    RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "airbot_play_free_drive started");

    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    executor = airbot::hardware::AsioExecutor::create(8);
    arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                           MotorType::DM, EEFType::NA, MotorType::NA>();

    if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
      std::cerr << "Failed to initialize arm" << std::endl;
    }

    arm->enable();

    arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

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

    t_ctrl = std::thread([&]() {
      std::cout << "机械臂会掉落，确保安全！" << std::endl;
      std::cout << "按下回车继续..." << std::endl;
      std::cin.get();
      while (running) {
        arm->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        std::this_thread::sleep_for(4ms);
      }
    });
  }

  ~AirbotExampleNode() {
    running = false;
    t.join();
    t_ctrl.join();
    if (arm) {
      arm->disable();
      arm->uninit();
    }
  }

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6 && arm) {
      for (int i = 0; i < 6; ++i) {
        arm_cmd[i] = msg->position[i];
      }

      if (msg->velocity.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
          arm_vel[i] = msg->velocity[i];
        }
      } else {
        arm_vel = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
      }

      if (msg->effort.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
          arm_eff[i] = msg->effort[i];
        }
      } else {
        arm_eff = {8, 8, 8, 8, 8, 8};
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor;
  std::unique_ptr<airbot::hardware::Arm<6>> arm;
  bool running;
  std::thread t;
  std::thread t_ctrl;
  std::array<double, 6> arm_cmd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> arm_vel = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::array<double, 6> arm_eff = {8, 8, 8, 8, 8, 8};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirbotExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
