#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/datatypes/hand.hpp"
#include "airbot_hardware/handlers/ro_hand/ro_hand.hpp"
#include "argparse/argparse.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

bool all_target(const std::vector<uint16_t>& current_pos, const std::array<uint16_t, 6>& target_pos, double tol = 100) {
  for (size_t i = 0; i < 6; i++)
    if (std::abs(current_pos[i] - target_pos[i]) > tol) return false;
  return true;
}

// Helper function to create JSON command
boost::json::object commandToJson(const std::string& type, const std::vector<double>& arm_positions,
                                  const std::vector<double>& hand_positions) {
  boost::json::object obj;
  obj["type"] = type;

  boost::json::array arm_array;
  for (const auto& pos : arm_positions) {
    arm_array.push_back(pos);
  }
  obj["arm_positions"] = arm_array;

  boost::json::array hand_array;
  for (const auto& pos : hand_positions) {
    hand_array.push_back(pos);
  }
  obj["hand_positions"] = hand_array;

  return obj;
}

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

// UDP server configuration
const std::string UDP_SERVER_IP = "127.0.0.1";  // 可根据需要修改
const int UDP_SERVER_PORT = 9870;               // 可根据需要修改

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ro_hand_ctrl");

  program.add_argument("-j", "--joint_angle")
      .default_value(std::vector<uint16_t>{0, 0, 0, 0, 0, 0})
      .help(
          "List of uint16_t values for HandState positions (0~1000). \n"
          "\033[32mshake\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_ctrl -j 0 0 0 0 0 0\033[0m\n"
          "\033[32mfist\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_ctrl -j 1000 1000 1000 1000 1000 1000\033[0m\n"
          "\033[32mpraise\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_ctrl -j 0 1000 1000 1000 1000 0\033[0m\n")
      .nargs(6)
      .scan<'u', uint16_t>();

  program.add_argument("-i", "--interface")
      .help("CAN interface to use (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote").help("UDP server IP address").default_value(std::string("192.168.1.2"));

  program.add_argument("-P", "--port").help("UDP server port").scan<'i', int>().default_value(9870);

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
    std::cout << "joint: lower 0, upper 1000." << std::endl;
    return 0;
  }

  auto executor = airbot::hardware::AsioExecutor::create(8);

  std::unique_ptr<ro_hand::RoHand> hand = ro_hand::RoHand::create(2u);
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  std::cout << "Initializing arm on " << can_interface << "..." << std::endl;
  if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }
  std::cout << "Initializing RoHand..." << std::endl;
  if (!hand->init(executor->get_io_context(), can_interface)) {
    std::cerr << "Failed to initialize RoHand." << std::endl;
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
    // 直接使用 HandState 的 positions 作为显示值
    std::vector<double> hand_positions(hand_state.positions.begin(), hand_state.positions.end());
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  std::cout << "Setting hand pose..." << std::endl;

  HandState hand_cmd;
  // 直接将输入值设置为 HandState positions (0~1000)
  // pose 顺序: ThumbFlex, ThumbAux, Index, Middle, Ring, Pinky (HandState顺序)
  for (size_t i = 0; i < 6; ++i) {
    hand_cmd.positions[i] = pose[i];
    hand_cmd.forces[i] = 0;
    hand_cmd.velocities[i] = 1000;
  }
  HandState state_hand;
  auto start_time = std::chrono::steady_clock::now();
  state_hand = hand->get_cached_state();

  while (!all_target(std::vector<uint16_t>(state_hand.positions.begin(), state_hand.positions.end()),
                     hand_cmd.positions) &&
         std::chrono::steady_clock::now() - start_time < 10s) {
    hand->set_pos(hand_cmd);

    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    // 直接使用 HandState positions 作为命令显示
    std::vector<double> hand_command(hand_cmd.positions.begin(), hand_cmd.positions.end());
    std::string command_string = boost::json::serialize(commandToJson("command", positions, hand_command));
    try {
      socket.send_to(boost::asio::buffer(command_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send command data: " << e.what() << std::endl;
    }
    state_hand = hand->get_cached_state();
    for (int i = 0; i < 6; i++) {
      std::cout << state_hand.positions[i] << ", ";
    }
    std::cout << " " << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  return 0;
}
