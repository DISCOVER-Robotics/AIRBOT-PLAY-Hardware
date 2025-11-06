#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
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

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

// UDP server configuration
const std::string UDP_SERVER_IP = "127.0.0.1";  // 可根据需要修改
constexpr int UDP_SERVER_PORT = 9870;           // 可根据需要修改

bool check_parameter(const std::vector<uint16_t> arm_cmd) {
  // 检查arm_cmd的大小是否为6
  if (arm_cmd.size() != 6) {
    return false;
  }

  for (size_t i = 0; i < arm_cmd.size(); ++i) {
    // 将uint16_t转换为double进行比较
    double cmd_value = static_cast<double>(arm_cmd[i]);
    if (cmd_value < 0 || cmd_value > 1000) {
      return false;
    }
  }

  return true;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ins_hand_pose");

  program.add_argument("-j", "--joint_angle")
      .default_value(std::vector<uint16_t>{0, 0, 100, 100, 100, 100})
      .help(
          "List of uint16_t values for HandState positions (0~1000). \n"
          "\033[32mfist\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_ctrl -j 1000 1000 1000 1000 1000 1000\033[0m\n"
          "\033[32mpraise\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_ctrl -j 10 0 1000 1000 1000 1000\033[0m\n"
          "\033[32mshake\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_ctrl -j 0 0 800 800 800 800\033[0m\n")
      .nargs(6)
      .scan<'u', uint16_t>();

  program.add_argument("-i", "--interface")
      .help("CAN interface to use (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote").help("UDP server IP address").default_value(std::string(UDP_SERVER_IP));

  program.add_argument("-P", "--port").help("UDP server port").scan<'i', int>().default_value(UDP_SERVER_PORT);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::vector<uint16_t> pose = program.get<std::vector<uint16_t>>("-j");
  std::string can_interface = program.get<std::string>("-i");
  std::string udp_server_ip = program.get<std::string>("-r");
  int udp_server_port = program.get<int>("-P");

  if (!check_parameter(pose)) {
    std::cout << "joint: lower 0, upper 400." << std::endl;
    return 0;
  }

  auto executor = airbot::hardware::AsioExecutor::create(8);

  // auto executor = AsioExecutor::create(8);
  std::unique_ptr<brain_co::BrainCoHand> hand = brain_co::BrainCoHand::create(1u);
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  std::cout << "Initializing arm on " << can_interface << "..." << std::endl;
  if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }
  std::cout << "Initializing BrainCo Hand..." << std::endl;
  if (!hand->init(executor->get_io_context(), can_interface, brain_co::BaudRate::BR_115200)) {
    std::cerr << "Failed to initialize BrainCo Hand." << std::endl;
    return -1;
  }

  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

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

  std::cout << "Enter space(\' \') to shift gesture..." << std::endl;

  HandState hand_cmd;
  // 直接使用 HandState positions (0~1000)
  std::copy_n(pose.begin(), 6, hand_cmd.positions.begin());
  std::fill(hand_cmd.forces.begin(), hand_cmd.forces.end(), 0);
  std::fill(hand_cmd.velocities.begin(), hand_cmd.velocities.end(), 0);

  hand->set_pos(hand_cmd);

  {
    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    std::vector<uint16_t> hand_command(hand_cmd.positions.begin(), hand_cmd.positions.end());
    std::string command_string = boost::json::serialize(commandToJson("command", positions, hand_command));
    try {
      socket.send_to(boost::asio::buffer(command_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send command data: " << e.what() << std::endl;
    }
  }

  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  return 0;
}
