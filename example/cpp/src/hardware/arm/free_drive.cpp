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
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();
  if (!arm->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  // std::this_thread::sleep_for(1s);

  auto now = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - now <= 10s) {
    arm->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(4ms);
  }

  arm->disable();
  arm->uninit();
  return 0;
}
