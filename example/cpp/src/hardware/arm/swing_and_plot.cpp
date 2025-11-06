#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

// UDP server configuration
const std::string UDP_SERVER_IP = "127.0.0.1";  // Change this to your server IP
const int UDP_SERVER_PORT = 9870;               // Change this to your server port

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

int main() {
  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto ratio = 2.0;
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();
  if (!arm->init(executor->get_io_context(), "can0", 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  bool running = true;
  // Setup UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(UDP_SERVER_IP), UDP_SERVER_PORT);
  try {
    socket.open(boost::asio::ip::udp::v4());
    std::cout << "UDP socket opened, sending data to " << UDP_SERVER_IP << ":" << UDP_SERVER_PORT << std::endl;
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

  arm->set_param("motor1.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor2.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor3.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor4.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor5.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor6.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  {
    auto now = std::chrono::steady_clock::now();
    auto synced = false;
    while (std::chrono::steady_clock::now() - now < 30s) {
      auto t =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      if (!synced && std::abs(arm->state().pos[0] - std::sin(static_cast<double>(t) / 1e3 * ratio)) > 0.01) {
        arm->pvt({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0},
                 {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
      } else {
        synced = true;
        arm->pvt({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0});
        std::vector<double> current_command = {std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0};
        std::string command_string = boost::json::serialize(commandToJson("command", current_command));

        try {
          socket.send_to(boost::asio::buffer(command_string), server_endpoint);
        } catch (std::exception& e) {
          std::cerr << "Failed to send command data: " << e.what() << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  arm->set_param("motor1.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("motor2.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("motor3.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("motor4.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("motor5.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("motor6.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));
  {
    auto now = std::chrono::steady_clock::now();
    auto synced = false;
    while (std::chrono::steady_clock::now() - now < 30s) {
      auto t =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
              .count();
      if (!synced && std::abs(arm->state().pos[0] - std::sin(static_cast<double>(t) / 1e3 * ratio)) > 0.01) {
        arm->mit({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {10.0, 10.0, 10.0, 1.0, 1.0, 1.0},
                 {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      } else {
        synced = true;
        arm->mit({std::sin(static_cast<double>(t) / 1e3 * ratio), 0.0, 0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 {250.0, 250.0, 250.0, 50.0, 50.0, 50.0}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
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
  }

  running = false;
  t.join();

  socket.close();
  arm->disable();
  arm->uninit();
  return 0;
}
