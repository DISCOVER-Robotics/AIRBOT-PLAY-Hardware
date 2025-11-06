#include <airbot_hardware/handlers/chassis.hpp>
#include <iostream>
#include <thread>

using namespace airbot::hardware;

int main() {
  auto chassis = Chassis::create<ChassisType::STAT>();

  chassis->init(nullptr, "192.168.11.1", 0);
  chassis->set_param("base.max_moving_speed", "high");
  chassis->set_param("base.max_angular_speed", "high");

  std::atomic<bool> running{true};

  std::thread([&]() {
    while (running.load()) {
      auto state = chassis->state();
      std::cerr << state.format() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }).detach();

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
    chassis->cmd_vel({0.0, 0.0, 1.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
