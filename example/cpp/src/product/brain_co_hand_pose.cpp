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
#include "airbot_hardware/handlers/brain_co/brain_co_hand.hpp"
#include "argparse/argparse.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

// Function to create command JSON
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions,
                                  const std::vector<uint16_t>& hand_positions = {}) {
  boost::json::object json_obj;

  if (!positions.empty()) {
    boost::json::array pos_array;
    for (const auto& pos : positions) pos_array.push_back(pos);

    json_obj[command_type + "/position"] = pos_array;
  }

  // Add hand positions if provided
  if (!hand_positions.empty()) {
    boost::json::array hand_array;
    for (const auto& pos : hand_positions) hand_array.push_back(pos);
    json_obj[command_type + "/hand_position"] = hand_array;
  }

  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

inline boost::json::object commandToJson(const airbot::hardware::HandState& state,
                                         const std::string& command_type = "state") {
  boost::json::object json_obj;

  boost::json::array name_array;
  for (const auto& name : airbot::hardware::HandState::names) {
    switch (name) {
      case airbot::hardware::MotorName::ThumbFlex:
        name_array.push_back("ThumbFlex");
        break;
      case airbot::hardware::MotorName::ThumbAux:
        name_array.push_back("ThumbAux");
        break;
      case airbot::hardware::MotorName::Index:
        name_array.push_back("Index");
        break;
      case airbot::hardware::MotorName::Middle:
        name_array.push_back("Middle");
        break;
      case airbot::hardware::MotorName::Ring:
        name_array.push_back("Ring");
        break;
      case airbot::hardware::MotorName::Pinky:
        name_array.push_back("Pinky");
        break;
      default:
        name_array.push_back("Unknown");
        break;
    }
  }
  json_obj["name"] = name_array;

  // 位置
  boost::json::array pos_array;
  for (const auto& pos : state.positions) pos_array.push_back(pos);
  json_obj[command_type + "/position"] = pos_array;

  // 速度
  boost::json::array vel_array;
  for (const auto& vel : state.velocities) vel_array.push_back(vel);
  json_obj[command_type + "/velocity"] = vel_array;

  // 力
  boost::json::array force_array;
  for (const auto& force : state.forces) force_array.push_back(force);
  json_obj[command_type + "/force"] = force_array;

  // 时间戳
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

inline std::string to_string(const airbot::hardware::HandState& state) {
  std::ostringstream oss;

  // 电机名称
  oss << "name: [";
  for (size_t i = 0; i < airbot::hardware::HandState::names.size(); ++i) {
    switch (airbot::hardware::HandState::names[i]) {
      case airbot::hardware::MotorName::ThumbFlex:
        oss << "ThumbFlex";
        break;
      case airbot::hardware::MotorName::ThumbAux:
        oss << "ThumbAux";
        break;
      case airbot::hardware::MotorName::Index:
        oss << "Index";
        break;
      case airbot::hardware::MotorName::Middle:
        oss << "Middle";
        break;
      case airbot::hardware::MotorName::Ring:
        oss << "Ring";
        break;
      case airbot::hardware::MotorName::Pinky:
        oss << "Pinky";
        break;
      default:
        oss << "Unknown";
        break;
    }
    if (i < airbot::hardware::HandState::names.size() - 1) oss << ",";
  }
  oss << "]\n";

  // 位置信息
  oss << "pos: [";
  for (size_t i = 0; i < state.positions.size(); ++i) {
    oss << state.positions[i];
    if (i < state.positions.size() - 1) oss << ",";
  }
  oss << "]\n";

  // 速度信息
  oss << "vel: [";
  for (size_t i = 0; i < state.velocities.size(); ++i) {
    oss << state.velocities[i];
    if (i < state.velocities.size() - 1) oss << ",";
  }
  oss << "]\n";

  // 力信息
  oss << "force: [";
  for (size_t i = 0; i < state.forces.size(); ++i) {
    oss << state.forces[i];
    if (i < state.forces.size() - 1) oss << ",";
  }
  oss << "]";

  return oss.str();
}

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

const std::string UDP_SERVER_IP = "127.0.0.1";
constexpr int UDP_SERVER_PORT = 9870;

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ins_hand_pose");

  program.add_argument("-p", "--pose")
      .help(
          "Hand pose preset. Supported: shake, praise, six\n"
          "\033[32msix\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_pose -p six\033[0m\n"
          "\033[32mpraise\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_pose -p praise\033[0m\n"
          "\033[32mshake\033[0m\n"
          "\033[34m\texample_airbot_brain_co_hand_pose -p shake\033[0m\n")
      .default_value(std::string("praise"));

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

  std::string pose_str = program.get<std::string>("-p");
  std::vector<uint16_t> pose;
  if (pose_str == "six") {
    pose = {0, 0, 1000, 1000, 1000, 0};
  } else if (pose_str == "praise") {
    pose = {0, 0, 1000, 1000, 1000, 1000};
  } else if (pose_str == "shake") {
    pose = {0, 0, 300, 300, 300, 300};
  } else {
    std::cerr << "Unknown pose: " << pose_str << ". Supported: shake, praise, six" << std::endl;
    return 1;
  }

  std::string can_interface = program.get<std::string>("-i");
  std::string udp_server_ip = program.get<std::string>("-r");
  int udp_server_port = program.get<int>("-P");

  auto executor = AsioExecutor::create(8);
  std::unique_ptr<brain_co::BrainCoHand> hand = brain_co::BrainCoHand::create(1u);
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }

  if (!hand->init(executor->get_io_context(), "can0", brain_co::BaudRate::BR_115200)) {
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
    std::vector<uint16_t> hand_positions;
    std::copy_n(hand_state.positions.begin(), 6, std::back_inserter(hand_positions));
    std::string state_string = boost::json::serialize(commandToJson("state", positions, hand_positions));
    try {
      socket.send_to(boost::asio::buffer(state_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state data: " << e.what() << std::endl;
    }
  }

  std::cout << "Enter space(\' \') to shift gesture..." << std::endl;

  HandState hand_cmd;

  std::copy_n(pose.begin(), 6, hand_cmd.positions.begin());

  hand->set_pos(hand_cmd);

  // 发送命令数据
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
