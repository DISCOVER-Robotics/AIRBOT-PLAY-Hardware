#include <argparse/argparse.hpp>
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

std::atomic<bool> running{true};

// Function to create command JSON
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions_teach_arm,
                                  const std::vector<double>& positions_arm, const std::vector<double>& velocity_arm,
                                  const std::vector<double>& effort_arm) {
  boost::json::object json_obj;

  // Convert position vector to JSON array
  boost::json::array pos_array_teach;
  boost::json::array pos_array_arm;
  boost::json::array vel_array_arm;
  boost::json::array eff_array_arm;
  for (const auto& pos : positions_teach_arm) pos_array_teach.push_back(pos);
  for (const auto& pos : positions_arm) pos_array_arm.push_back(pos);
  for (const auto& vel : velocity_arm) vel_array_arm.push_back(vel);
  for (const auto& eff : effort_arm) eff_array_arm.push_back(eff);

  json_obj[command_type + "/teach" + "/position"] = pos_array_teach;
  json_obj[command_type + "/arm" + "/position"] = pos_array_arm;
  json_obj[command_type + "/arm" + "/velocity"] = vel_array_arm;
  json_obj[command_type + "/arm" + "/effort"] = eff_array_arm;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

void signalHandler(int signal) {
  if (signal == SIGINT) {
    running = false;
  }
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, signalHandler);
  argparse::ArgumentParser program("1f1");

  program.add_argument("-i_arm", "--can-interface-arm")
      .help("play CAN interface name (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-i_teacharm", "--can-interface-teacharm")
      .help("Replay CAN interface name (e.g., can0, can1)")
      .default_value(std::string("can1"));

  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));

  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string can_interface_arm = program.get<std::string>("-i_arm");
  std::string can_interface_teacharm = program.get<std::string>("-i_teacharm");
  std::string remote_ip = program.get<std::string>("--remote-ip");
  int port = program.get<int>("--port");

  auto executor_teach = airbot::hardware::AsioExecutor::create(7);
  auto teach_arm = airbot::hardware::Arm<7>::create<MotorType::EC, MotorType::EC, MotorType::EC, MotorType::EC,
                                                    MotorType::EC, MotorType::EC, EEFType::E2, MotorType::EC>();

  if (!teach_arm->init(executor_teach->get_io_context(), can_interface_teacharm, 250_hz)) {
    std::cerr << "Failed to initialize teach arm" << std::endl;
    return 1;
  }
  teach_arm->enable();
  teach_arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

  auto executor_arm = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<7>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::G2, MotorType::DM>();
  if (!arm->init(executor_arm->get_io_context(), can_interface_arm, 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }

  std::array<double, 7> arm_vel = {3, 3, 3, 3, 3, 3, 20};
  std::array<double, 7> arm_eff = {8, 8, 8, 8, 8, 8, 10};

  // Setup UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(remote_ip), port);
  try {
    socket.open(boost::asio::ip::udp::v4());
    std::cout << "UDP socket opened, sending data to " << remote_ip << ":" << port << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    teach_arm->disable();
    teach_arm->uninit();
    arm->disable();
    arm->uninit();
    return 1;
  }

  std::array<double, 7> teach_arm_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));

      teach_arm->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      auto teach_state = teach_arm->state();

      teach_state.pos[6] = teach_state.pos[6] * 1.5;
      teach_state.pos[6] = std::clamp(teach_state.pos[6], 0.0, 0.072);
      teach_arm_pos = teach_state.pos;

      std::vector<double> positions_teach(teach_state.pos.begin(), teach_state.pos.end());

      auto state = arm->state();
      std::vector<double> positions_arm(state.pos.begin(), state.pos.end());
      std::vector<double> velocity_arm(state.vel.begin(), state.vel.end());
      std::vector<double> effort_arm(state.eff.begin(), state.eff.end());
      std::string state_string =
          boost::json::serialize(commandToJson("state", positions_teach, positions_arm, velocity_arm, effort_arm));

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

  while (running) {
    arm->pvt(teach_arm_pos, arm_vel, arm_eff);
  }

  t.join();

  socket.close();
  arm->disable();
  arm->uninit();

  teach_arm->disable();
  teach_arm->uninit();

  return 0;
}
