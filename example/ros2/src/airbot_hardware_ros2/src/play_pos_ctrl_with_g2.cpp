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
  AirbotExampleNode() : Node("airbot_play_pos_ctrl_with_g2") {
    this->declare_parameter<std::string>("i", "can0");
    std::string can_interface = this->get_parameter("i").as_string();
    RCLCPP_INFO(this->get_logger(), "Using CAN interface: %s", can_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "airbot_play_pos_ctrl_with_g2 started");

    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_command", 10, std::bind(&AirbotExampleNode::joint_state_callback, this, std::placeholders::_1));

    executor = airbot::hardware::AsioExecutor::create(8);
    arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                           MotorType::DM, EEFType::NA, MotorType::NA>();

    executor_eef = AsioExecutor::create(1);
    eef = EEF<1>::create<EEFType::G2, MotorType::DM>();

    is_eef_init = true;

    if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
      std::cerr << "Failed to initialize arm" << std::endl;
    }
    if (!eef->init(executor_eef->get_io_context(), can_interface, 250_hz)) {
      std::cout << "WARNING: Failed to initialize eef." << std::endl;
      is_eef_init = false;
    }

    arm->enable();

    arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
    eef->enable();
    eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

    running = true;

    t = std::thread([&]() {
      while (running) {
        // std::cout << arm->state().is_valid << std::endl;
        std::this_thread::sleep_for(4ms);
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "eef"};

        if (arm) {
          auto state = arm->state();
          std::vector<double> pos(state.pos.begin(), state.pos.end());
          std::vector<double> vel(state.vel.begin(), state.vel.end());
          std::vector<double> eff(state.eff.begin(), state.eff.end());

          if (eef) {
            pos.push_back(eef->state().pos[0]);
            vel.push_back(eef->state().vel[0]);
            eff.push_back(eef->state().eff[0]);
          } else {
            pos.push_back(0.0);
            vel.push_back(0.0);
            eff.push_back(0.0);
          }

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
        if (is_eef_init) {
          arm->pvt(arm_cmd, arm_vel, arm_eff);
          eef->pvt({eef_pos, 5.0, 0.0, 50.0, 1.0, 1.0});
          std::this_thread::sleep_for(4ms);
        } else {
          arm->pvt(arm_cmd, arm_vel, arm_eff);
          std::this_thread::sleep_for(4ms);
        }
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
    if (eef) {
      eef->disable();
      eef->uninit();
    }
  }

 private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6 && arm) {
      for (int i = 0; i < 6; ++i) {
        arm_cmd[i] = msg->position[i];
      }
      eef_pos = msg->position[6];

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
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor;
  std::unique_ptr<airbot::hardware::AsioExecutor> executor_eef;
  std::unique_ptr<airbot::hardware::Arm<6>> arm;
  std::unique_ptr<airbot::hardware::EEF<1>> eef;
  bool is_eef_init;
  bool running;
  std::thread t;
  std::thread t_ctrl;
  double eef_pos = 0.0;
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
