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

// Function to create command JSON
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions_teach_arm) {
  boost::json::object json_obj;

  // Convert position vector to JSON array
  boost::json::array pos_array_teach;
  for (const auto& pos : positions_teach_arm) pos_array_teach.push_back(pos);

  json_obj[command_type + "/teach" + "/position"] = pos_array_teach;

  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("replay");

  program.add_argument("-i", "--can-interface")
      .help("Replay CAN interface name (e.g., can0, can1)")
      .default_value(std::string("can0"));

  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));

  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  std::string can_interface = program.get<std::string>("-i");
  std::string remote_ip = program.get<std::string>("--remote-ip");
  int port = program.get<int>("--port");

  auto executor_teach = airbot::hardware::AsioExecutor::create(7);
  auto teach_arm = airbot::hardware::Arm<7>::create<MotorType::EC, MotorType::EC, MotorType::EC, MotorType::EC,
                                                    MotorType::EC, MotorType::EC, EEFType::E2, MotorType::EC>();

  if (!teach_arm->init(executor_teach->get_io_context(), can_interface, 250_hz)) {
    std::cerr << "Failed to initialize teach arm" << std::endl;
    return 1;
  }
  teach_arm->enable();
  teach_arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

  bool running = true;

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
    return 1;
  }

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));

      teach_arm->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      auto teach_state = teach_arm->state();

      std::vector<double> positions_teach(teach_state.pos.begin(), teach_state.pos.end());

      std::string state_string = boost::json::serialize(commandToJson("state", positions_teach));

      try {
        socket.send_to(boost::asio::buffer(state_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send state data: " << e.what() << std::endl;
      }
    }
  });

  while (true) {
    std::cout << "replay pos: ";
    for (const auto& value : teach_arm->state().pos) {
      std::cout << value << " ";
    }
    std::cout << std::endl;
  }

  running = false;
  t.join();

  socket.close();

  teach_arm->disable();
  teach_arm->uninit();

  return 0;
}
