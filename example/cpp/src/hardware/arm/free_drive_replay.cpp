#include <cmath>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

int main() {
  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<7>::create<MotorType::EC, MotorType::EC, MotorType::EC, MotorType::EC, MotorType::EC,
                                              MotorType::EC, EEFType::E2, MotorType::EC>();

  if (!arm->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  // std::this_thread::sleep_for(1s);

  auto now = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - now <= 10s) {
    arm->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::cerr << arm->state().pos[0] << " " << arm->state().pos[1] << " " << arm->state().pos[2] << " "
              << arm->state().pos[3] << " " << arm->state().pos[4] << " " << arm->state().pos[5] << " "
              << arm->state().pos[6] << std::endl;
    std::this_thread::sleep_for(4ms);
  }

  arm->disable();
  arm->uninit();
  return 0;
}
