#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include "./hand_example.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/brain_co/brain_co_hand.hpp"
#include "airbot_hardware/handlers/datatypes/hand.hpp"
#include "argparse/argparse.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

std::atomic<bool> running(true);
void signal_handler(int) { running = false; }

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-2) return false;
  return true;
}

bool all_reached(const std::array<double, 6>& pos, const std::array<double, 6>& target, double tol = 1e-3) {
  for (size_t i = 0; i < pos.size(); ++i) {
    if (std::abs(pos[i] - target[i]) > tol) {
      return false;
    }
  }
  return true;
}

// UDP server configuration
const std::string UDP_SERVER_IP = "127.0.0.1";
constexpr int UDP_SERVER_PORT = 9870;

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_brain_co_hand_greeting");
  program.add_argument("-i", "--interface")
      .help("CAN interface to use (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote").help("UDP server IP address").default_value(std::string(UDP_SERVER_IP));

  program.add_argument("-P", "--port").help("UDP server port").scan<'i', int>().default_value(UDP_SERVER_PORT);

  program.add_argument("--amp")
      .help("Sine wave amplitude [uint16_t] 0~500")
      .default_value(static_cast<uint16_t>(250u))
      .scan<'u', uint16_t>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto can_interface = program.get<std::string>("-i");
  std::string udp_server_ip = program.get<std::string>("-r");
  int udp_server_port = program.get<int>("-P");

  uint16_t amp = std::clamp(program.get<uint16_t>("--amp"), (uint16_t)0u, (uint16_t)500u);
  auto executor = AsioExecutor::create(8);
  auto hand = brain_co::BrainCoHand::create(1u);
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }

  if (!hand->init(executor->get_io_context(), can_interface, brain_co::BaudRate::BR_115200)) {
    std::cerr << "Failed to initialize BrainCo Hand." << std::endl;
    return 1;
  }

  HandState cmd;
  std::fill(cmd.positions.begin(), cmd.positions.end(), 0);
  std::fill(cmd.forces.begin(), cmd.forces.end(), 0);
  std::fill(cmd.velocities.begin(), cmd.velocities.end(), 0);
  hand->set_pos(cmd);

  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  std::signal(SIGINT, signal_handler);

  // UDP socket setup
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(udp_server_ip), udp_server_port);
  try {
    socket.open(boost::asio::ip::udp::v4());
    std::cout << "UDP socket opened, sending data to " << udp_server_ip << ":" << udp_server_port << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    return 1;
  }

  auto now = std::chrono::steady_clock::now();
  std::cout << "Arm homing to zero..." << std::endl;
  while (!all_zero(arm->state().pos) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(10ms);

    // 发送当前状态
    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    HandState hand_state = hand->get_cached_state();
    std::vector<uint16_t> hand_positions(hand_state.positions.begin(), hand_state.positions.end());
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  while (!all_reached(arm->state().pos, {0.0, 0.0, M_PI / 2, 0.0, 0.0, 0.0}) &&
         std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0.0, 0.0, M_PI / 2, 0.0, 0.0, 0.0}, {M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8});
    std::this_thread::sleep_for(1ms);

    // 发送当前状态
    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    HandState hand_state = hand->get_cached_state();
    std::vector<uint16_t> hand_positions(hand_state.positions.begin(), hand_state.positions.end());
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  std::cout << "Hand waving (sin wave)..." << std::endl;

  uint16_t base_pos = 500;                                                        // HandState 中间值 (0~1000)
  uint16_t amp_clip = std::min<uint16_t>(amp, 500);                               // 振幅不超过500
  std::array<double, 4> phase_offsets = {0.0, M_PI / 4, M_PI / 2, 3 * M_PI / 4};  // 层次感
  auto start_time = std::chrono::steady_clock::now();

  while (running) {
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - start_time).count();

    // ThumbFlex 和 ThumbAux 固定为0 (张开)
    cmd.positions[0] = 0u;
    cmd.positions[1] = 0u;

    // Index, Middle, Ring, Pinky 做正弦波动
    for (size_t i = 0; i < 4; ++i) {
      double phase = 2 * M_PI * t + phase_offsets[i];
      double val = static_cast<double>(base_pos) + static_cast<double>(amp_clip) * std::sin(phase);
      // 限制在 HandState 范围 0~1000 并转换为uint16_t
      uint16_t pos = static_cast<uint16_t>(std::clamp(val, 0.0, 1000.0));
      cmd.positions[i + 2] = pos;
    }
    hand->set_pos(cmd);

    // 发送当前状态
    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    HandState hand_state = hand->get_cached_state();
    std::vector<uint16_t> hand_positions(hand_state.positions.begin(), hand_state.positions.end());
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  now = std::chrono::steady_clock::now();
  std::cout << "Arm homing to zero..." << std::endl;
  while (!all_zero(arm->state().pos) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(10ms);

    // 发送当前状态
    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    HandState hand_state = hand->get_cached_state();
    std::vector<uint16_t> hand_positions(hand_state.positions.begin(), hand_state.positions.end());
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  // Close hand (set to maximum positions)
  std::fill(cmd.positions.begin(), cmd.positions.end(), 1000);  // All fingers closed
  hand->set_pos(cmd);
  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  return 0;
}
