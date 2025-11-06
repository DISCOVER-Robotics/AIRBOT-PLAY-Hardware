#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/datatypes/hand.hpp"
#include "airbot_hardware/handlers/ro_hand/ro_hand.hpp"
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
  for (size_t i = 0; i < pos.size(); ++i)
    if (std::abs(pos[i] - target[i]) > tol) return false;
  return true;
}

// Helper function to create JSON command
boost::json::object commandToJson(const std::string& type, const std::vector<double>& arm_positions,
                                  const std::vector<double>& hand_positions) {
  boost::json::object obj;
  obj["type"] = type;

  boost::json::array arm_array;
  for (const auto& pos : arm_positions) arm_array.push_back(pos);
  obj["arm_positions"] = arm_array;

  boost::json::array hand_array;
  for (const auto& pos : hand_positions) hand_array.push_back(pos);
  obj["hand_positions"] = hand_array;

  return obj;
}

// HandState ranges (0~1000)
constexpr std::pair<uint16_t, uint16_t> hand_pos_range = {0, 1000};

// Helper to calculate mid point and amplitude for HandState
struct HandStateRange {
  static uint16_t mid() { return 200; }
  static uint16_t amplitude(double ratio) { return static_cast<uint16_t>(300.0 * ratio); }
};

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ro_hand_greeting_mainthread");
  program.add_argument("-i", "--interface").help("CAN interface (e.g., can0)").default_value(std::string("can0"));
  program.add_argument("-r", "--remote").help("UDP server IP").default_value(std::string("192.168.1.2"));
  program.add_argument("-P", "--port").help("UDP port").scan<'i', int>().default_value(9870);
  program.add_argument("--amp").help("Sine amplitude ratio 0.0~1.0").default_value(0.5).scan<'g', double>();

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
  double amp = std::clamp(program.get<double>("--amp"), 0.0, 1.0);

  // Only arm uses io_context thread
  boost::asio::io_context arm_io;
  auto arm_executor = AsioExecutor::create(8);
  auto hand_executor = AsioExecutor::create(2);
  std::thread arm_thread;

  // Create arm & hand
  auto hand = ro_hand::RoHand::create(2u);
  auto arm = Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM, MotorType::DM,
                            EEFType::NA, MotorType::NA>();

  if (!arm->init(arm_executor->get_io_context(), can_interface, 250)) {
    std::cerr << "Failed to initialize arm on " << can_interface << std::endl;
    return 1;
  }

  if (!hand->init(hand_executor->get_io_context(), can_interface)) {
    std::cerr << "Failed to initialize RoHand." << std::endl;
    return 1;
  }

  HandState cmd{};
  std::fill(cmd.positions.begin(), cmd.positions.end(), 0);
  std::fill(cmd.forces.begin(), cmd.forces.end(), 0);
  std::fill(cmd.velocities.begin(), cmd.velocities.end(), 0);

  arm->enable();
  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  std::signal(SIGINT, signal_handler);

  // UDP socket setup
  boost::asio::io_context udp_ctx;
  boost::asio::ip::udp::socket socket(udp_ctx);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(udp_server_ip), udp_server_port);
  socket.open(boost::asio::ip::udp::v4());
  std::cout << "UDP socket opened -> " << udp_server_ip << ":" << udp_server_port << std::endl;

  // Arm homing
  std::cout << "Arm homing..." << std::endl;
  auto now = std::chrono::steady_clock::now();
  while (!all_zero(arm->state().pos) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0, 0, 0, 0, 0, 0}, {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(10ms);
  }

  // Control loop periods
  const std::chrono::milliseconds arm_period(20);
  const std::chrono::milliseconds hand_period(50);
  const double arm_freq = 0.5;
  const double arm_amp = 1.0;

  // Arm timer
  auto arm_timer = std::make_shared<boost::asio::steady_timer>(arm_io, arm_period);
  auto arm_start_time = std::chrono::steady_clock::now();

  std::function<void(const boost::system::error_code&)> arm_tick;
  arm_tick = [&](const boost::system::error_code& ec) {
    if (ec || !running.load()) return;
    auto t = std::chrono::duration<double>(std::chrono::steady_clock::now() - arm_start_time).count();
    double desired = arm_amp * std::sin(2.0 * M_PI * arm_freq * t);
    auto target = arm->state().pos;
    target[0] = desired;
    std::array<double, 6> speed{};
    speed.fill(M_PI / 6);
    arm->pvt(target, speed);
    arm_timer->expires_after(arm_period);
    arm_timer->async_wait(arm_tick);
  };

  // Run arm async
  arm_thread = std::thread([&] { arm_io.run(); });
  boost::asio::post(arm_io, [&] { arm_timer->async_wait(arm_tick); });

  std::cout << "Running... Press Ctrl+C to stop." << std::endl;

  // --- Hand control in main thread ---
  auto hand_start_time = std::chrono::steady_clock::now();
  std::array<double, 4> phase_offsets = {0.0, M_PI / 4, M_PI / 2, 3 * M_PI / 4};
  auto last_hand_time = std::chrono::steady_clock::now();

  // === 延迟测量相关变量 ===
  std::unordered_map<int, std::chrono::steady_clock::time_point> send_time_map;
  std::array<uint16_t, 6> last_positions{};
  for (auto& p : last_positions) p = 0;

  while (running) {
    auto now = std::chrono::steady_clock::now();

    // 手部命令发送
    if (now - last_hand_time >= hand_period) {
      last_hand_time = now;
      auto t = std::chrono::duration<double>(now - hand_start_time).count();

      HandState local_cmd{};
      for (size_t i = 2; i < 6; ++i) {
        double phase = 2 * M_PI * t + phase_offsets[i - 2];
        double val = HandStateRange::mid() + HandStateRange::amplitude(amp) * std::sin(phase);
        local_cmd.positions[i] = static_cast<uint16_t>(std::clamp(val, 0.0, 1000.0));
      }
      local_cmd.forces.fill(0);

      hand->set_pos(local_cmd);

      // 记录发送时间（使用指令第一个位置的值作为 key）
      int key = static_cast<int>(local_cmd.positions[2]);
      send_time_map[key] = now;
    }

    // 获取反馈并计算延迟
    HandState hs = hand->get_cached_state();

    // 检查是否达到了发送的目标之一
    int key = static_cast<int>(hs.positions[2]);
    if (send_time_map.count(key)) {
      auto latency = std::chrono::duration<double, std::milli>(now - send_time_map[key]).count();
      std::cout << "[HAND_FEEDBACK_LATENCY] " << latency << " ms (target=" << key << ")" << std::endl;
      send_time_map.erase(key);
    }

    // 发送 UDP telemetry
    auto state = arm->state();
    std::vector<double> pos(state.pos.begin(), state.pos.end());
    std::vector<double> hand_pos(hs.positions.begin(), hs.positions.end());
    auto json = boost::json::serialize(commandToJson("state", pos, hand_pos));
    socket.send_to(boost::asio::buffer(json), server_endpoint);

    std::this_thread::sleep_for(10ms);
  }

  std::cout << "Stopping..." << std::endl;

  boost::asio::post(arm_io, [&] {
    arm_timer->cancel();
    arm_io.stop();
  });

  if (arm_thread.joinable()) arm_thread.join();

  std::fill(cmd.positions.begin(), cmd.positions.end(), 0);
  hand->set_pos(cmd);

  arm->disable();
  arm->uninit();
  hand->uninit();
  socket.close();
  std::cout << "Exited cleanly." << std::endl;

  return 0;
}
