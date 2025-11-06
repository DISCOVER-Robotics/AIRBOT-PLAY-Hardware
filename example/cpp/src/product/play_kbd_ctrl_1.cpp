#include <argparse/argparse.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
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

std::atomic<bool> running(true);
std::atomic<bool> need_return_to_zero(false);
std::atomic<bool> emergency_stop(false);

void signal_handler(int signal) {
  if (signal == SIGINT) {
    if (!need_return_to_zero) {
      std::cout << "\nReceived SIGINT, returning to zero position..." << std::endl;
      need_return_to_zero = true;
      running = false;
    } else {
      std::cout << "\nForcing immediate shutdown..." << std::endl;
      emergency_stop = true;
    }
  }
}

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

// Function to create state JSON
boost::json::object stateToJson(const std::vector<double>& positions, const std::vector<double>& velocity,
                                const std::vector<double>& effort) {
  boost::json::object json_obj;

  // Convert vectors to JSON arrays
  boost::json::array pos_array;
  boost::json::array vel_array;
  boost::json::array eff_array;
  for (const auto& pos : positions) pos_array.push_back(pos);
  for (const auto& vel : velocity) vel_array.push_back(vel);
  for (const auto& eff : effort) eff_array.push_back(eff);

  json_obj["position"] = pos_array;
  json_obj["velocity"] = vel_array;
  json_obj["effort"] = eff_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

bool is_arm_arrive(const std::array<double, 6> arm_target, const std::array<double, 6> arm_pose,
                   double tolerance = 0.02) {
  for (int i = 0; i < 6; i++) {
    if (std::abs(arm_target[i] - arm_pose[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, signal_handler);

  argparse::ArgumentParser program("airbot_play_kbd_ctrl_1", "1.0");

  program.add_argument("-i", "--can-interface")
      .help("CAN interface name (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));

  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string can_interface = program.get<std::string>("--can-interface");
  std::string remote_ip = program.get<std::string>("--remote-ip");
  int port = program.get<int>("--port");

  std::cout << "Using CAN interface: " << can_interface << std::endl;
  std::cout << "Sending data to Plotjuggler at " << remote_ip << ":" << port << std::endl;

  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();

  if (!arm->init(executor->get_io_context(), can_interface.c_str(), 250)) {
    std::cerr << "Failed to initialize arm on interface " << can_interface << std::endl;
    return 1;
  }

  arm->enable();

  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  std::array<double, 6> zero_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> move_velocity = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::array<double, 6> move_effort = {8, 8, 8, 8, 8, 8};

  std::array<double, 6> MIN_THRESHOLDS = {-3.1416, -2.9671, -0.087266, -3.0107, -1.7628, -3.0107};
  std::array<double, 6> MAX_THRESHOLDS = {2.0944, 0.17453, 3.1416, 3.0107, 1.7628, 3.0107};

  while (running) {
    if (!arm->state().is_valid) {
      arm->pvt(zero_position, move_velocity, move_effort);
    } else {
      if (!is_arm_arrive(zero_position, arm->state().pos)) {
        arm->pvt(zero_position, move_velocity, move_effort);
      } else {
        break;
      }
    }
    std::this_thread::sleep_for(4ms);
  }

  if (!running) {
    arm->disable();
    arm->uninit();
    return 0;
  }

  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::CSV));

  auto tty_comm = CommHandler<char>::create("tty_comm", "tty", "", 1000, executor->get_io_context());

  std::atomic<uint32_t> key_event_latched = 0;
  std::array<std::chrono::steady_clock::time_point, 6> key_trigger_time;
  std::atomic<double> acc = 0.002 * M_PI;
  std::atomic<double> dec = 0.001 * M_PI;
  std::atomic<double> vel_max = M_PI * 0.25;

  std::shared_mutex vel_target_mutex;
  std::array<double, 6> vel_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(remote_ip), port);

  try {
    socket.open(boost::asio::ip::udp::v4());
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    arm->disable();
    arm->uninit();
    return 1;
  }

  std::thread plotjuggler_thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz

      auto state = arm->state();
      std::vector<double> pos(state.pos.begin(), state.pos.end());
      std::vector<double> vel(state.vel.begin(), state.vel.end());
      std::vector<double> eff(state.eff.begin(), state.eff.end());

      std::string state_string = boost::json::serialize(stateToJson(pos, vel, eff));

      try {
        socket.send_to(boost::asio::buffer(state_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send state data: " << e.what() << std::endl;
      }
    }
  });

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
    running = false;
  }

  while (running) {
    {
      std::shared_lock<std::shared_mutex> lock(vel_target_mutex);

      std::array<double, 6> cmd = vel_target;
      cmd[3] *= 0.5;
      cmd[4] *= 0.5;
      cmd[5] *= 0.5;

      if (arm->state().is_valid) {
        auto state_pos = arm->state().pos;
        for (size_t i = 0; i < state_pos.size(); ++i) {
          if (state_pos[i] < MIN_THRESHOLDS[i]) {
            if (cmd[i] < 0.0) {
              cmd[i] = 0.0;
            }
          }
          if (state_pos[i] > MAX_THRESHOLDS[i]) {
            if (cmd[i] > 0.0) {
              cmd[i] = 0.0;
            }
          }
        }
      }
      arm->csv(cmd);
    }
    std::this_thread::sleep_for(4ms);
  }

  if (tty_comm) {
    tty_comm->delete_read_callback("test");
    tty_comm->stop();
  }

  running = false;

  if (throttle_thread.joinable()) throttle_thread.join();
  if (driver_thread.joinable()) driver_thread.join();
  if (plotjuggler_thread.joinable()) plotjuggler_thread.join();

  // Return to zero position if needed
  if (need_return_to_zero && !emergency_stop) {
    std::cout << "Returning to zero position..." << std::endl;

    // Switch to PVT mode for precise position control
    arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

    auto start_time = std::chrono::steady_clock::now();

    while (!emergency_stop) {
      // Send return-to-zero command
      arm->pvt(zero_position, move_velocity, move_effort);

      // Check if arrived at zero position
      if (arm->state().is_valid && is_arm_arrive(zero_position, arm->state().pos)) {
        std::cout << "Reached zero position. Shutting down." << std::endl;
        break;
      }

      // Check timeout
      auto now = std::chrono::steady_clock::now();
      if (now - start_time > 15s) {
        std::cerr << "Timeout while returning to zero position." << std::endl;
        break;
      }

      std::this_thread::sleep_for(4ms);
    }
  }

  socket.close();
  arm->disable();
  arm->uninit();

  return 0;
}
