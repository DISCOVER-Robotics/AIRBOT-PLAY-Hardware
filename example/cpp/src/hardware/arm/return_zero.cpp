#include <cmath>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

bool all_zero(const std::array<double, 7>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

int main() {
  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<7>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::G2, MotorType::DM>();
  if (!arm->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  auto now = std::chrono::steady_clock::now();
  while (!all_zero(arm->state().pos) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
             {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(1ms);
  }

  arm->disable();
  arm->uninit();
  return 0;
}
