#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/eef.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

class AirbotExampleNode : public rclcpp::Node {
 public:
  AirbotExampleNode() : Node("airbot_eef_g2") {
    this->declare_parameter<std::string>("i", "can0");
    std::string can_interface = this->get_parameter("i").as_string();
    RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "airbot_eef_g2 started");

    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    executor = AsioExecutor::create(1);
    eef = EEF<1>::create<EEFType::E2, MotorType::OD>();

    if (!eef->init(executor->get_io_context(), can_interface, 250_hz)) {
      std::cerr << "Failed to initialize eef" << std::endl;
    }

    eef->enable();
    eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

    running = true;

    t = std::thread([&]() {
      while (running) {
        std::this_thread::sleep_for(4ms);
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"eef"};

        if (eef) {
          auto state = eef->state();
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
      while (running) {
        eef->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        std::this_thread::sleep_for(4ms);
      }
    });
  }

  ~AirbotExampleNode() {
    running = false;
    t.join();
    t_ctrl.join();
    if (eef) {
      eef->disable();
      eef->uninit();
    }
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor;
  std::unique_ptr<airbot::hardware::EEF<1>> eef;
  bool running;
  std::thread t;
  std::thread t_ctrl;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirbotExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
