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
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions_teach_arm,
                                  const std::vector<double>& positions_arm,
                                  const std::vector<double>& velocity_teach_arm,
                                  const std::vector<double>& velocity_arm, const std::vector<double>& effort_teach_arm,
                                  const std::vector<double>& effort_arm) {
  boost::json::object json_obj;

  // Convert position vector to JSON array
  boost::json::array pos_array_teach;
  boost::json::array pos_array_arm;
  boost::json::array vel_array_teach;
  boost::json::array vel_array_arm;
  boost::json::array eff_array_teach;
  boost::json::array eff_array_arm;
  for (const auto& pos : positions_teach_arm) pos_array_teach.push_back(pos);
  for (const auto& pos : positions_arm) pos_array_arm.push_back(pos);
  for (const auto& vel : velocity_teach_arm) vel_array_teach.push_back(vel);
  for (const auto& vel : velocity_arm) vel_array_arm.push_back(vel);
  for (const auto& eff : effort_teach_arm) eff_array_teach.push_back(eff);
  for (const auto& eff : effort_arm) eff_array_arm.push_back(eff);

  json_obj[command_type + "/teach" + "/position"] = pos_array_teach;
  json_obj[command_type + "/arm" + "/position"] = pos_array_arm;
  json_obj[command_type + "/teach" + "/velocity"] = vel_array_teach;
  json_obj[command_type + "/arm" + "/velocity"] = vel_array_arm;
  json_obj[command_type + "/teach" + "/effort"] = eff_array_teach;
  json_obj[command_type + "/arm" + "/effort"] = eff_array_arm;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

int main() {
  auto executor_teach = airbot::hardware::AsioExecutor::create(7);
  auto teach_arm = std::array{
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 1>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 2>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 3>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 4>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 5>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 6>(),
      airbot::hardware::Motor::create<airbot::hardware::MotorType::EC, 7>(),
  };

  for (auto&& i : teach_arm) {
    i->init(executor_teach->get_io_context(), "can1", 500);
    i->enable();
  }

  auto executor_arm = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<7>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::G2, MotorType::DM>();
  if (!arm->init(executor_arm->get_io_context(), "can0", 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }

  bool running = true;
  std::array<double, 7> arm_vel = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 1};
  std::array<double, 7> arm_eff = {8, 8, 8, 8, 8, 8, 10};

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

  int num = 0;
  std::array<double, 7> teach_arm_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));

      for (auto&& i : teach_arm) {
        num++;
        if (!i->ping()) {
          std::cerr << "Failed to send ping command to left motor:" << num << std::endl;
          break;
        }
      }
      num = 0;

      teach_arm_pos[0] = teach_arm[0]->state().pos;
      teach_arm_pos[1] = teach_arm[1]->state().pos;
      teach_arm_pos[2] = teach_arm[2]->state().pos;
      teach_arm_pos[3] = teach_arm[3]->state().pos;
      teach_arm_pos[4] = teach_arm[4]->state().pos;
      teach_arm_pos[5] = teach_arm[5]->state().pos;
      teach_arm_pos[6] = teach_arm[6]->state().pos;
      teach_arm_pos[6] = 0.0 + (teach_arm_pos[6] - 0.01) / (-2.7 - 0.01) * (0.07 - 0.0);
      teach_arm_pos[6] = std::clamp(teach_arm_pos[6], 0.0, 0.07);
      std::vector<double> positions_teach(teach_arm_pos.begin(), teach_arm_pos.end());
      std::vector<double> velocity_teach(teach_arm_pos.begin(), teach_arm_pos.end());
      std::vector<double> effort_teach(teach_arm_pos.begin(), teach_arm_pos.end());

      auto state = arm->state();
      std::vector<double> positions_arm(state.pos.begin(), state.pos.end());
      std::vector<double> velocity_arm(state.pos.begin(), state.pos.end());
      std::vector<double> effort_arm(state.pos.begin(), state.pos.end());
      std::string state_string = boost::json::serialize(commandToJson(
          "state", positions_teach, positions_arm, velocity_teach, velocity_arm, effort_teach, effort_arm));

      try {
        socket.send_to(boost::asio::buffer(state_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send state data: " << e.what() << std::endl;
      }
    }
  });

  arm->enable();

  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  while (true) {
    arm->pvt(teach_arm_pos, arm_vel, arm_eff);
  }

  running = false;
  t.join();

  socket.close();
  arm->disable();
  arm->uninit();

  for (auto&& i : teach_arm) {
    i->disable();
    i->uninit();
  }

  return 0;
}
