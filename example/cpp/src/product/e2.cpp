#include <argparse/argparse.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <utility>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/eef.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

// Function to create state JSON
boost::json::object stateToJson(const std::vector<double>& positions, const std::vector<double>& velocity,
                                const std::vector<double>& effort) {
  boost::json::object json_obj;

  // Convert vectors to JSON arrays
  boost::json::array pos_array;
  boost::json::array vel_array;
  boost::json::array eff_array;
  for (const auto& pos : positions) pos_array.push_back(pos);
  for (const auto& vel : velocity) vel_array.push_back(vel);
  for (const auto& eff : effort) eff_array.push_back(eff);

  json_obj["position"] = pos_array;
  json_obj["velocity"] = vel_array;
  json_obj["effort"] = eff_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_e2", "1.0");

  program.add_argument("-i", "--can-interface")
      .help("CAN interface name (e.g., can0, can1)")
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

  std::string can_interface = program.get<std::string>("--can-interface");
  std::string remote_ip = program.get<std::string>("--remote-ip");
  int port = program.get<int>("--port");

  std::cout << "Using CAN interface: " << can_interface << std::endl;
  std::cout << "Sending data to Plotjuggler at " << remote_ip << ":" << port << std::endl;

  auto executor = AsioExecutor::create(1);
  auto eef = EEF<1>::create<EEFType::E2, MotorType::OD>();

  eef->init(executor->get_io_context(), can_interface.c_str(), 250_hz);
  eef->enable();
  eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(remote_ip), port);

  try {
    socket.open(boost::asio::ip::udp::v4());
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    eef->disable();
    eef->uninit();
    return 1;
  }

  auto ts = std::thread([&]() {
    auto last_print_time = std::chrono::steady_clock::now();

    while (true) {
      auto now = std::chrono::steady_clock::now();

      if (now - last_print_time > 100ms) {
        auto state = eef->state();
        double width = state.pos[0];
        std::cout << "E2 Parallel Width: " << width << std::endl;
        last_print_time = now;
      }

      auto state = eef->state();
      std::vector<double> pos(state.pos.begin(), state.pos.end());
      std::vector<double> vel(state.vel.begin(), state.vel.end());
      std::vector<double> eff(state.eff.begin(), state.eff.end());

      std::string state_string = boost::json::serialize(stateToJson(pos, vel, eff));

      try {
        socket.send_to(boost::asio::buffer(state_string), server_endpoint);
      } catch (std::exception& e) {
        std::cerr << "Failed to send state data: " << e.what() << std::endl;
      }

      std::this_thread::sleep_for(10ms);
    }
  });

  ts.detach();

  auto start = std::chrono::steady_clock::now();
  bool running = true;

  while (running && std::chrono::steady_clock::now() - start < 120s) {
    eef->mit({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(1ms);
  }

  eef->disable();
  eef->uninit();

  return 0;
}
