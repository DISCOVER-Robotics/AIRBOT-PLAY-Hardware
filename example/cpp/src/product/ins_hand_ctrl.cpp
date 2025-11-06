#include <array>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/hand/ins_hand.hpp"
#include "argparse/argparse.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

const std::string UDP_SERVER_IP = "127.0.0.1";
const int UDP_SERVER_PORT = 9870;

bool all_target(const std::vector<uint16_t>& current_pos, const std::array<uint16_t, 6>& target_pos, double tol = 50) {
  for (size_t i = 0; i < 6; i++)
    if (std::abs(current_pos[i] - target_pos[i]) > tol) return false;
  return true;
}

boost::json::object handStateToJson(const std::vector<uint16_t>& positions) {
  boost::json::object json_obj;
  boost::json::array pos_array;
  for (auto p : positions) pos_array.push_back(p);
  json_obj["hand/position"] = pos_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;
  return json_obj;
}

bool check_parameter(const std::array<uint16_t, 6> arm_cmd) {
  for (size_t i = 0; i < arm_cmd.size(); ++i) {
    // 将uint16_t转换为double进行比较
    double cmd_value = static_cast<double>(arm_cmd[i]);
    if (cmd_value < 0.0 || cmd_value > 1000) {
      return false;
    }
  }
  return true;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ins_hand_ctrl");

  program.add_argument("-j", "--joints")
      .default_value(std::vector<uint16_t>{0, 0, 0, 0, 0, 0})
      .help("Target joint positions for the hand (6 values)")
      .nargs(6)
      .scan<'u', uint16_t>();

  program.add_argument("-i", "--interface")
      .help("CAN interface to use (e.g., can0, can1)")
      .default_value(std::string("can0"));

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::array<uint16_t, 6> target_pos{};
  auto joint_values = program.get<std::vector<uint16_t>>("-j");
  std::copy_n(joint_values.begin(), 6, target_pos.begin());

  auto can_interface = program.get<std::string>("-i");

  if (!check_parameter(target_pos)) {
    std::cout << "joint: lower 0, upper 1000." << std::endl;
    return 0;
  }

  auto executor = AsioExecutor::create(1);
  auto hand = InsHand::create<1>();
  hand->init(executor->get_io_context(), can_interface, 250_hz);
  hand->reset();
  hand->reset_error();

  // UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(UDP_SERVER_IP), UDP_SERVER_PORT);
  socket.open(boost::asio::ip::udp::v4());

  auto start_time = std::chrono::steady_clock::now();
  HandState state = hand->state();
  while (!all_target(std::vector<uint16_t>(state.positions.begin(), state.positions.end()), target_pos) &&
         std::chrono::steady_clock::now() - start_time < 10s) {
    hand->get_param("ANGLE_ACT");

    std::vector<uint16_t> pos(state.positions.begin(), state.positions.end());

    std::string msg = boost::json::serialize(handStateToJson(pos));
    try {
      socket.send_to(boost::asio::buffer(msg), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send hand state: " << e.what() << std::endl;
    }

    HandState hand_cmd;
    std::copy_n(target_pos.begin(), 6, hand_cmd.positions.begin());
    std::fill_n(hand_cmd.velocities.begin(), 6, 500);
    // std::fill_n(hand_cmd.forces.begin(), 6, 50);

    hand->set_pos(hand_cmd);
    state = hand->state();
    std::this_thread::sleep_for(100ms);
  }

  // 停止
  hand->uninit();
  socket.close();
  return 0;
}
