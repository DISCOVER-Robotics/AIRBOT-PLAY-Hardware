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
  auto eef = EEF<1>::create<EEFType::G2, MotorType::DM>();

  eef->init(executor->get_io_context(), "can0", 1000_hz);
  eef->enable();
  eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

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
    float now_p =
        (std::sin(std::chrono::steady_clock::now().time_since_epoch().count() / 1000000000.0 * 15) + 1) * 0.5f;
    eef->pvt({0.03 * now_p, 5.0, 0.0, 50.0, 1.0, 1.0});
    std::this_thread::sleep_for(1ms);
  }

  eef->disable();
  eef->uninit();

  return 0;
}
