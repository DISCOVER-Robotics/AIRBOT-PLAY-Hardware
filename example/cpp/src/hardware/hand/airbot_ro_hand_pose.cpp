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

std::atomic<bool> running(true);

void signal_handler(int) { running = false; }

bool all_zero(const std::array<double, 6>& pos, const bool& is_valid) {
  if (is_valid != true) {
    return false;
  } else {
    for (const auto& p : pos)
      if (std::abs(p) > 1e-2) return false;
    return true;
  }
}

void set_nonblocking(bool enable) {
  static struct termios oldt;
  static bool saved = false;

  if (enable && !saved) {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    saved = true;
  } else if (!enable && saved) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, 0);
    saved = false;
  }
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

int get_key() {
  unsigned char c;
  if (read(STDIN_FILENO, &c, 1) == 1) {
    return c;
  }
  return -1;
}

// UDP server configuration
const std::string UDP_SERVER_IP = "127.0.0.1";
const int UDP_SERVER_PORT = 9870;

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ro_hand_pose");

  program.add_argument("-p", "--pose")
      .help(
          "Hand pose preset. Supported: shake, praise, fist\n"
          "\033[32mfist\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_pose -p fist\033[0m\n"
          "\033[32mpraise\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_pose -p praise\033[0m\n"
          "\033[32mshake\033[0m\n"
          "\033[34m\texample_airbot_ro_hand_pose -p shake\033[0m\n")
      .default_value(std::string("shake"));

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

  std::string pose_name = program.get<std::string>("-p");
  std::string can_interface = program.get<std::string>("-i");
  std::string udp_server_ip = program.get<std::string>("-r");
  int udp_server_port = program.get<int>("-P");

  // Define pose presets using HandState (0~1000)
  std::map<std::string, HandState> pose_presets;

  // Shake pose: all fingers extended (open)
  HandState shake_pose;
  shake_pose.positions = {0, 0, 0, 0, 0, 0};  // ThumbFlex, ThumbAux, Index, Middle, Ring, Pinky
  std::fill(shake_pose.forces.begin(), shake_pose.forces.end(), 0);
  std::fill(shake_pose.velocities.begin(), shake_pose.velocities.end(), 0);
  pose_presets["shake"] = shake_pose;

  // Fist pose: all fingers closed
  HandState fist_pose;
  fist_pose.positions = {1000, 1000, 1000, 1000, 1000, 1000};
  std::fill(fist_pose.forces.begin(), fist_pose.forces.end(), 0);
  std::fill(fist_pose.velocities.begin(), fist_pose.velocities.end(), 0);
  pose_presets["fist"] = fist_pose;

  // Praise pose: thumb up, other fingers closed
  HandState praise_pose;
  praise_pose.positions = {12, 53, 1000, 1000, 1000, 1000};  // Based on ctrl example
  std::fill(praise_pose.forces.begin(), praise_pose.forces.end(), 0);
  std::fill(praise_pose.velocities.begin(), praise_pose.velocities.end(), 0);
  pose_presets["praise"] = praise_pose;

  if (pose_presets.find(pose_name) == pose_presets.end()) {
    std::cerr << "Unknown pose: " << pose_name << std::endl;
    std::cerr << "Available poses: shake, fist, praise" << std::endl;
    return 1;
  }
  size_t pose_idx = 0;

  HandState selected_pose = pose_presets[pose_name];

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

  std::signal(SIGINT, signal_handler);
  set_nonblocking(true);

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
  while (!all_zero(arm->state().pos, arm->state().is_valid) && std::chrono::steady_clock::now() - now <= 10s) {
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
  while (running) {
    std::cout << "Please press the space to confirm the switch gesture." << std::endl;
    int ch = get_key();
    if (ch == ' ') {
      pose_idx = (pose_idx + 1) % pose_presets.size();
      if (pose_idx == 0) {
        selected_pose = pose_presets["shake"];
        std::cout << "shake" << std::endl;
      }
      if (pose_idx == 1) {
        selected_pose = pose_presets["fist"];
        std::cout << "fist" << std::endl;
      }
      if (pose_idx == 2) {
        selected_pose = pose_presets["praise"];
        std::cout << "praise" << std::endl;
      }
    }

    hand->set_pos(selected_pose);

    auto state = arm->state();
    std::vector<double> positions(state.pos.begin(), state.pos.end());
    // 直接使用 HandState positions 作为命令显示
    std::vector<double> hand_command(selected_pose.positions.begin(), selected_pose.positions.end());
    std::string command_string = boost::json::serialize(commandToJson("command", positions, hand_command));
    try {
      socket.send_to(boost::asio::buffer(command_string), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send command data: " << e.what() << std::endl;
    }
    std::this_thread::sleep_for(100ms);
  }
  set_nonblocking(false);

  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  return 0;
}
