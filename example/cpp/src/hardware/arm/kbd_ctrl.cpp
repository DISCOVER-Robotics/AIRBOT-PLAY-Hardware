#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <thread>

#include "airbot_hardware/comm/comm.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/utils.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

static constexpr const char* INSTRUCTIONS =
    "Keyboard control examples (Low-level SDK, joint-space)\n"
    "Key 1, Key 2: Motor #1\n"
    "Key 3, Key 4: Motor #2\n"
    "Key 5, Key 6: Motor #3\n"
    "Key 7, Key 8: Motor #4\n"
    "Key 9, Key 0: Motor #5\n"
    "Key -, Key +: Motor #6\n"
    "Key a: Decrease max velocity\n"
    "Key s: Reset max velocity to PI rad/s\n"
    "Key d: Increase max velocity\n"
    "Joint limit is not effective. Use with care!";

int main() {
  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();
  if (!arm->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::CSV));

  auto tty_comm = CommHandler<char>::create("tty_comm", "tty", "", 1000, executor->get_io_context());

  std::atomic<uint32_t> key_event_latched = 0;
  std::array<std::chrono::steady_clock::time_point, 6> key_trigger_time;
  std::atomic<double> acc = 0.1 * M_PI;
  std::atomic<double> dec = 0.01 * M_PI;
  std::atomic<double> vel_max = M_PI;
  std::atomic<bool> running = true;

  std::shared_mutex vel_target_mutex;
  std::array<double, 6> vel_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  tty_comm->add_read_callback("test", [&](const char& c) {
    auto now = std::chrono::steady_clock::now();
    switch (c) {
      case '1':
        key_event_latched |= 1 << 0;
        key_trigger_time[0] = now;
        break;
      case '2':
        key_event_latched |= 1 << 1;
        key_trigger_time[0] = now;
        break;
      case '3':
        key_event_latched |= 1 << 2;
        key_trigger_time[1] = now;
        break;
      case '4':
        key_event_latched |= 1 << 3;
        key_trigger_time[1] = now;
        break;
      case '5':
        key_event_latched |= 1 << 4;
        key_trigger_time[2] = now;
        break;
      case '6':
        key_event_latched |= 1 << 5;
        key_trigger_time[2] = now;
        break;
      case '7':
        key_event_latched |= 1 << 6;
        key_trigger_time[3] = now;
        break;
      case '8':
        key_event_latched |= 1 << 7;
        key_trigger_time[3] = now;
        break;
      case '9':
        key_event_latched |= 1 << 8;
        key_trigger_time[4] = now;
        break;
      case '0':
        key_event_latched |= 1 << 9;
        key_trigger_time[4] = now;
        break;
      case '-':
        key_event_latched |= 1 << 10;
        key_trigger_time[5] = now;
        break;
      case '=':
        key_event_latched |= 1 << 11;
        key_trigger_time[5] = now;
        break;
      case 'a':
        if (vel_max > 0.1) {
          vel_max.store(vel_max.load() - 0.1);
          std::cerr << "Max joint velocity decreased to " << vel_max.load() << " rad/s" << std::endl;
        }
        break;
      case 's':
        vel_max = M_PI;
        std::cerr << "Max joint velocity reset to " << vel_max.load() << " rad/s" << std::endl;
        break;
      case 'd':
        if (vel_max < M_PI * 2 - 0.1) {
          vel_max.store(vel_max.load() + 0.1);
          std::cerr << "Max joint velocity increased to " << vel_max.load() << " rad/s" << std::endl;
        }
        break;
      default:
        break;
    }
  });

  std::thread throttle_thread([&]() {
    while (running) {
      auto now = std::chrono::steady_clock::now();
      for (uint8_t i = 0; i < 12; i++)
        if (key_event_latched & (1 << i) && now - key_trigger_time[i / 2] > 100ms) key_event_latched &= ~(1 << i);
      std::this_thread::sleep_for(1ms);
    }
  });

  std::thread driver_thread([&]() {
    while (running) {
      {
        std::unique_lock<std::shared_mutex> lock(vel_target_mutex);
        for (uint8_t i = 0; i < 6; i++) {
          if (key_event_latched & (1 << (2 * i))) {
            if (vel_target[i] < vel_max - acc)
              vel_target[i] += acc.load();
            else
              vel_target[i] = vel_max;
          } else if (key_event_latched & (1 << (2 * i + 1))) {
            if (vel_target[i] > -vel_max + acc)
              vel_target[i] -= acc.load();
            else
              vel_target[i] = -vel_max;
          } else {
            if (vel_target[i] > dec)
              vel_target[i] -= dec;
            else if (vel_target[i] < -dec)
              vel_target[i] += dec;
            else
              vel_target[i] = 0;
          }
        }
      }
      std::this_thread::sleep_for(1ms);
    }
  });

  std::cerr << INSTRUCTIONS << std::endl;

  if (!tty_comm->start()) {
    std::cout << "Failed to start tty_comm" << std::endl;
    return 1;
  }

  while (true) {
    {
      std::shared_lock<std::shared_mutex> lock(vel_target_mutex);
      arm->csv(vel_target);
    }
    std::this_thread::sleep_for(4ms);
  }

  tty_comm->delete_read_callback("test");
  running = false;
  throttle_thread.join();
  driver_thread.join();
  return 0;
}
