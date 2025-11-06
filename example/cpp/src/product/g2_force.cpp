#include <argparse/argparse.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <utility>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

static bool running = true;  // 全局标志，用于控制线程和循环

void signal_handler(int signal) {
  if (signal == SIGINT) {
    std::cout << "\nCaught CTRL+C, exiting..." << std::endl;
    running = false;
  }
}

boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions,
                                  const std::vector<double>& velocity, const std::vector<double>& effort) {
  boost::json::object json_obj;

  boost::json::array pos_array;
  boost::json::array vel_array;
  boost::json::array eff_array;
  for (const auto& pos : positions) pos_array.push_back(pos);
  for (const auto& vel : velocity) vel_array.push_back(vel);
  for (const auto& eff : effort) eff_array.push_back(eff);

  json_obj[command_type + "/position"] = pos_array;
  json_obj[command_type + "/velocity"] = vel_array;
  json_obj[command_type + "/effort"] = eff_array;
  json_obj["timestamp"] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now().time_since_epoch())
                                                  .count()) /
                          1e6;

  return json_obj;
}

int main(int argc, char* argv[]) {
  std::signal(SIGINT, signal_handler);

  argparse::ArgumentParser program("g2_pos_ctrl");

  program.add_argument("-i").help("can port").default_value(std::string("can0"));
  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));
  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);
  program.add_argument("-f").help("eef eff").scan<'g', double>().default_value(5.0);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto executor = airbot::hardware::AsioExecutor::create(1);
  auto eef = EEF<1>::create<EEFType::G2, MotorType::DM>();

  std::string can = program.get<std::string>("-i");
  std::string UDP_SERVER_IP = program.get<std::string>("-r");
  int UDP_SERVER_PORT = program.get<int>("-p");
  double eff = program.get<double>("-f");

  if (!eef->init(executor->get_io_context(), can, 250_hz)) {
    std::cerr << "Failed to initialize eef" << std::endl;
    return 1;
  }

  // Setup UDP socket
  boost::asio::io_context io_context;
  boost::asio::ip::udp::socket socket(io_context);
  boost::asio::ip::udp::endpoint server_endpoint(boost::asio::ip::make_address(UDP_SERVER_IP), UDP_SERVER_PORT);
  try {
    socket.open(boost::asio::ip::udp::v4());
    std::cout << "UDP socket opened, sending data to " << UDP_SERVER_IP << ":" << UDP_SERVER_PORT << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to open UDP socket: " << e.what() << std::endl;
    eef->disable();
    eef->uninit();
    return 1;
  }

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
      if (eef->state().is_valid) {
        auto state = eef->state();
        std::vector<double> pos(state.pos.begin(), state.pos.end());
        std::vector<double> vel(state.vel.begin(), state.vel.end());
        std::vector<double> eff(state.eff.begin(), state.eff.end());
        std::string state_string = boost::json::serialize(commandToJson("state", pos, vel, eff));

        try {
          socket.send_to(boost::asio::buffer(state_string), server_endpoint);
        } catch (std::exception& e) {
          std::cerr << "Failed to send state data: " << e.what() << std::endl;
        }
      }
    }
  });

  eef->enable();
  eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::MIT));

  while (running) {
    eef->mit({0.0, 0.0, eff, 0.0, 0.0});
    std::this_thread::sleep_for(4ms);
  }

  t.join();
  socket.close();
  eef->disable();
  eef->uninit();
  return 0;
}
