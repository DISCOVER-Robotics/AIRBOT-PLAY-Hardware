#include <array>
#include <atomic>
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
#include "airbot_hardware/handlers/hand/ins_hand.hpp"
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

bool all_reached(const std::array<double, 6>& pos, const std::array<double, 6>& target, double tol = 1e-3) {
  for (size_t i = 0; i < pos.size(); ++i)
    if (std::abs(pos[i] - target[i]) > tol) return false;
  return true;
}

boost::json::object stateToJson(const std::vector<double>& arm_pos, const std::vector<double>& hand_pos) {
  boost::json::object json_obj;
  boost::json::array arm_array, hand_array;
  for (auto p : arm_pos) arm_array.push_back(p);
  for (auto p : hand_pos) hand_array.push_back(p);
  json_obj["arm/position"] = arm_array;
  json_obj["hand/position"] = hand_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;
  return json_obj;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_arm_hand_ctrl");

  program.add_argument("-i", "--interface")
      .help("CAN interface to use (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("--amp").help("Sine wave amplitude").default_value(150.0).scan<'g', double>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto can_interface = program.get<std::string>("-i");
  double amp = program.get<double>("--amp");

  if (amp < 0.0 || amp > 300.0) {
    std::cout << "amp: lower 0.0, upper 300.0." << std::endl;
    return 1;
  }

  auto executor = AsioExecutor::create(8);
  auto hand = InsHand::create<1>();
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  if (!arm->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }
  if (!hand->init(executor->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize hand on " << can_interface << std::endl;
    return 1;
  }

  hand->reset();
  hand->reset_error();
  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  // UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address("127.0.0.1"), 9870);
  socket.open(boost::asio::ip::udp::v4());

  std::signal(SIGINT, signal_handler);

  // Arm move to target
  auto now = std::chrono::steady_clock::now();
  std::cout << "Arm moving to target..." << std::endl;
  now = std::chrono::steady_clock::now();
  std::array<double, 6> arm_target = {0.0, 0.0, M_PI / 2, 0.0, 0.0, 0.0};
  while (!all_reached(arm->state().pos, arm_target) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt(arm_target, {M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8});
    std::this_thread::sleep_for(1ms);
  }

  std::cout << "Hand waving (sin wave)..." << std::endl;
  uint16_t base_pos = 300;

  HandState cmd;
  std::fill_n(cmd.velocities.begin(), 6, 100);
  std::fill_n(cmd.forces.begin(), 6, 50);

  auto start_time = std::chrono::steady_clock::now();

  while (running) {
    auto now_time = std::chrono::steady_clock::now();
    double t_sec = std::chrono::duration<double>(now_time - start_time).count();

    cmd.positions = {200, 200, base_pos, base_pos, base_pos, base_pos};
    cmd.velocities = {1000, 1000, 1000, 1000, 1000, 1000};

    double omega = 2 * M_PI / 5.0;  // ω = 2π/5 rad/s → T = 5s
    for (size_t i = 0; i < 4; ++i) {
      double phase = omega * t_sec + i * M_PI / 3;
      cmd.positions[5 - i] = static_cast<uint16_t>(base_pos + amp * std::sin(phase));
    }

    hand->set_pos(cmd);

    auto arm_state = arm->state();
    hand->get_param("ANGLE_ACT");
    auto hand_state = hand->state();

    std::vector<double> arm_pos(arm_state.pos.begin(), arm_state.pos.end());
    std::vector<double> hand_pos(hand_state.positions.begin(), hand_state.positions.end());

    std::string msg = boost::json::serialize(stateToJson(arm_pos, hand_pos));
    try {
      socket.send_to(boost::asio::buffer(msg), server_endpoint);
    } catch (std::exception& e) {
      std::cerr << "Failed to send state: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(50ms);
  }

  std::cout << "Arm homing to zero..." << std::endl;
  while (!all_zero(arm->state().pos, arm->state().is_valid)) {
    arm->pvt({0, 0, 0, 0, 0, 0}, {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(4ms);
  }

  cmd.positions.fill(0);
  hand->set_pos(cmd);

  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  return 0;
}
