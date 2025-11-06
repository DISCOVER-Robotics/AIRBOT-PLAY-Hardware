#include <argparse/argparse.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

std::atomic<bool> running{true};

// Function to create command JSON
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions) {
  boost::json::object json_obj;

  // Convert position vector to JSON array
  boost::json::array pos_array;
  for (const auto& pos : positions) pos_array.push_back(pos);

  json_obj[command_type + "/position"] = pos_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

bool is_arm_arrive(const std::array<double, 6> arm_target, const std::array<double, 6> arm_pose) {
  if (std::abs(arm_target[0] - arm_pose[0]) <= 0.02 && std::abs(arm_target[1] - arm_pose[1]) <= 0.02 &&
      std::abs(arm_target[2] - arm_pose[2]) <= 0.02 && std::abs(arm_target[3] - arm_pose[3]) <= 0.02 &&
      std::abs(arm_target[4] - arm_pose[4]) <= 0.02 && std::abs(arm_target[5] - arm_pose[5]) <= 0.02) {
    return 1;
  } else {
    return 0;
  }
}

void signalHandler(int signal) {
  if (signal == SIGINT) {
    running = false;
  }
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, signalHandler);
  argparse::ArgumentParser program("play_mit_swing", "1.0");

  program.add_argument("-i", "--can-interface")
      .help("CAN interface name (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));

  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);

  program.add_argument("-T", "--period")
      .help("Period of sine wave oscillation (seconds)")
      .scan<'g', double>()
      .default_value(10.0);

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
  double period = program.get<double>("--period");

  double ratio;
  if (period < 2) {
    std::cout << "Parameter T is set too small. The recommended value is between 2 and 20." << std::endl;
    return 0;
  } else if (period > 20) {
    std::cout << "Parameter T is set too big. The recommended value is between 2 and 20." << std::endl;
    return 0;
  } else {
    ratio = 2 * M_PI / period;
  }

  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();
  if (!arm->init(executor->get_io_context(), can_interface.c_str(), 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  // Setup UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(remote_ip), port);
  try {
    socket.open(boost::asio::ip::udp::v4());
    std::cout << "UDP socket opened, sending data to " << remote_ip << ":" << port << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    arm->disable();
    arm->uninit();
    return 1;
  }

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      auto state = arm->state();
      std::vector<double> positions(state.pos.begin(), state.pos.end());
      std::string state_string = boost::json::serialize(commandToJson("state", positions));

      try {
        socket.send_to(boost::asio::buffer(state_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send state data: " << e.what() << std::endl;
      }
    }
  });

  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  while (running) {
    if (!arm->state().is_valid) {
      arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4});
    } else {
      if (!is_arm_arrive({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, arm->state().pos)) {
        arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4});
      } else {
        break;
      }
    }
    std::this_thread::sleep_for(4ms);
  }

  auto synced = false;
  while (running) {
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    if (!synced && std::abs(arm->state().pos[0] - std::sin(static_cast<double>(t) / 1e3 * ratio)) > 0.01) {
      arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
      arm->pvt({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0},
               {M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4}, {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    } else {
      if (!synced) arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
      synced = true;
      arm->mit({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0},
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {250.0, 250.0, 250.0, 50.0, 50.0, 50.0},
               {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      std::vector<double> current_command = {std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0};
      std::string command_string = boost::json::serialize(commandToJson("command", current_command));
      try {
        socket.send_to(boost::asio::buffer(command_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send command data: " << e.what() << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  // running = false;
  t.join();

  socket.close();
  arm->disable();
  arm->uninit();

  return 0;
}
