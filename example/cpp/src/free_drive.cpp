#include <cmath>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/motor.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

int main() {
  auto executor = AsioExecutor::create(1);
  auto motors = std::array{
      Motor::create<MotorType::OD, 1>(), Motor::create<MotorType::OD, 2>(), Motor::create<MotorType::OD, 3>(),
      Motor::create<MotorType::DM, 4>(), Motor::create<MotorType::DM, 5>(), Motor::create<MotorType::DM, 6>(),
  };

  for (auto &&i : motors) {
    i->init(executor->get_io_context(), "can0", 1000_hz);
    i->enable();
    i->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  }

  auto ts = std::thread([&]() {
    while (true) {
      std::cout << "========== Motor States: ===========" << std::endl;
      for (size_t i = 0; i < motors.size(); ++i) std::cout << motors[i]->state().format() << std::endl;
      std::this_thread::sleep_for(10ms);
    }
  });
  ts.detach();

  auto start = std::chrono::steady_clock::now();
  auto running = true;
  while (running && std::chrono::steady_clock::now() - start < std::chrono::seconds(120)) {
    // float now_p = (std::sin(std::chrono::steady_clock::now().time_since_epoch().count() / 1000000000.0 * 3) + 1) *
    // 0.5f;
    for (auto &&i : motors) {
      if (!i->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        std::cerr << "Failed to send ping command to motor" << std::endl;
        running = false;
        break;
      }
      std::this_thread::sleep_for(1ms);
    }
  }

  for (auto &&i : motors) {
    i->disable();
    i->uninit();
  }

  return 0;
}
