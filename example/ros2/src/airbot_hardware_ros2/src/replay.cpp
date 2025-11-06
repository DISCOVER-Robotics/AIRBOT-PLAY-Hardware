#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

class AirbotExampleNode : public rclcpp::Node {
 public:
  AirbotExampleNode() : Node("airbot_replay") {
    this->declare_parameter<std::string>("i", "can0");
    std::string can_interface = this->get_parameter("i").as_string();
    RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "airbot_replay started");

    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    executor_teach = airbot::hardware::AsioExecutor::create(7);
    teach_arm = std::array{
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 1>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 2>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 3>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 4>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 5>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 6>(),
        airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 7>(),
    };

    for (auto&& i : teach_arm) {
      i->init(executor_teach->get_io_context(), can_interface, 250_hz);
      i->enable();
    }

    running = true;

    t = std::thread([&]() {
      while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "eef"};

        for (auto&& i : teach_arm) {
          num++;
          if (!i->ping()) {
            std::cerr << "Failed to send ping command to left motor:" << num << std::endl;
            break;
          }
        }
        num = 0;
        const size_t joint_count = message.name.size();
        message.position.resize(joint_count, 0.0);
        message.velocity.resize(joint_count, 0.0);
        message.effort.resize(joint_count, 0.0);

        for (int i = 0; i < 7; ++i) {
          message.position[i] = teach_arm[i]->state().pos;
          message.velocity[i] = teach_arm[i]->state().vel;
          message.effort[i] = teach_arm[i]->state().eff;
        }
        message.position[6] = 0.0 + (message.position[6] - 0.01) / (-2.7 - 0.01) * (0.07 - 0.0);
        message.position[6] = std::clamp(message.position[6], 0.0, 0.07);

        try {
          joint_publisher_->publish(message);
        } catch (std::exception& e) {
          std::cerr << "Failed to send state data: " << e.what() << std::endl;
        }
      }
    });
  }

  ~AirbotExampleNode() {
    running = false;
    t.join();

    for (auto&& i : teach_arm) {
      i->disable();
      i->uninit();
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor_teach;
  std::array<std::unique_ptr<airbot::hardware::Motor>, 7> teach_arm;
  bool running;
  std::thread t;
  int num = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirbotExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
