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

    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_command", 10, std::bind(&AirbotExampleNode::joint_state_callback, this, std::placeholders::_1));

    executor = AsioExecutor::create(1);
    eef = EEF<1>::create<EEFType::G2, MotorType::DM>();

    if (!eef->init(executor->get_io_context(), can_interface, 250_hz)) {
      std::cerr << "Failed to initialize eef" << std::endl;
    }

    eef->enable();

    eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

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
        eef->pvt({eef_cmd, eef_vel, eef_eff, 50.0, 1.0, 1.0});
        std::this_thread::sleep_for(4ms);
      }
    });
  }

  ~AirbotExampleNode() {
    running = false;
    if (t.joinable()) t.join();
    if (t_ctrl.joinable()) t_ctrl.join();
    if (eef) {
      eef->disable();
      eef->uninit();
    }
  }

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (eef) {
      if (msg->position.size() >= 1) {
        eef_cmd = msg->position[0];
      } else {
        eef_cmd = 0.0;
      }
      if (msg->velocity.size() >= 1) {
        eef_vel = msg->velocity[0];
      } else {
        eef_vel = 0.3;
      }

      if (msg->effort.size() >= 1) {
        eef_eff = msg->effort[0];
      } else {
        eef_eff = 8;
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor;
  std::unique_ptr<airbot::hardware::EEF<1>> eef;
  bool running;
  std::thread t;
  std::thread t_ctrl;
  double eef_cmd = 0.0;
  double eef_vel = 0.5;
  double eef_eff = 8;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirbotExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
