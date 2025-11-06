#include <cmath>
#include <iostream>
#include <thread>
#include <utility>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/eef.hpp"
using namespace std::chrono_literals;
using namespace airbot::hardware;

int main() {
  auto executor = AsioExecutor::create(1);
  auto eef = EEF<1>::create<EEFType::E2, MotorType::OD>();

  eef->init(executor->get_io_context(), "can0", 250_hz);
  eef->enable();
  eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

  auto ts = std::thread([&]() {
    while (true) {
      std::cout << "========== EEF State: ===========" << std::endl;
      std::cout << eef->state().format() << std::endl;
      std::this_thread::sleep_for(10ms);
    }
  });
  ts.detach();

  auto start = std::chrono::steady_clock::now();
  auto running = true;
  while (running && std::chrono::steady_clock::now() - start < std::chrono::seconds(120)) {
    eef->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }

  eef->disable();
  eef->uninit();

  return 0;
}
