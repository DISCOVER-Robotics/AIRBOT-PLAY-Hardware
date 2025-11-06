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
#include "airbot_hardware/handlers/eef.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

// // UDP server configuration
// const std::string UDP_SERVER_IP = "127.0.0.1";  // Change this to your server IP
// const int UDP_SERVER_PORT = 987;                // Change this to your server port

// Function to create command JSON
boost::json::object commandToJson(const std::string& command_type, const std::vector<double>& positions,
                                  const std::vector<double>& velocity, const std::vector<double>& effort) {
  boost::json::object json_obj;

  // Convert position vector to JSON array
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

bool is_arm_arrive(const std::array<double, 6> arm_target, const std::array<double, 6> arm_pose) {
  if (std::abs(arm_target[0] - arm_pose[0]) <= 0.02 && std::abs(arm_target[1] - arm_pose[1]) <= 0.02 &&
      std::abs(arm_target[2] - arm_pose[2]) <= 0.02 && std::abs(arm_target[3] - arm_pose[3]) <= 0.02 &&
      std::abs(arm_target[4] - arm_pose[4]) <= 0.02 && std::abs(arm_target[5] - arm_pose[5]) <= 0.02) {
    return 1;
  } else {
    return 0;
  }
}

bool is_eef_arrive(const double eef_target, const double eef_pose) {
  if (std::abs(eef_target - eef_pose) <= 0.001) {
    return 1;
  } else {
    return 0;
  }
}

void print_joints_THRESHOLDS() {
  std::cout << "关节角输入阈值如下：" << std::endl;
  std::cout << "joint1: lower -3.1416, upper 2.0944." << std::endl;
  std::cout << "joint2: lower -2.9671, upper 0.17453." << std::endl;
  std::cout << "joint3: lower -0.087266, upper 3.1416." << std::endl;
  std::cout << "joint4: lower -3.0107, upper 3.0107." << std::endl;
  std::cout << "joint5: lower -1.7628, upper 1.7628." << std::endl;
  std::cout << "joint6: lower -3.0107, upper 3.0107." << std::endl;
  std::cout << "eef g2 joint: lower 0.0, upper 0.072" << std::endl;
}

bool check_parameter(const std::array<double, 6> arm_cmd, const double eef_pos) {
  const std::array<double, 6> MIN_THRESHOLDS = {-3.1416, -2.9671, -0.087266, -3.0107, -1.7628, -3.0107};
  const std::array<double, 6> MAX_THRESHOLDS = {2.0944, 0.17453, 3.1416, 3.0107, 1.7628, 3.0107};

  for (size_t i = 0; i < arm_cmd.size(); ++i) {
    if (arm_cmd[i] < MIN_THRESHOLDS[i] || arm_cmd[i] > MAX_THRESHOLDS[i]) {
      return false;
    }
  }
  if (eef_pos < 0.0 || eef_pos > 0.072) {
    return false;
  }
  return true;
}

std::array<double, 6> vectorToArray(const std::vector<double>& vec, const double eef_pose) {
  if (vec.size() != 6) {
    throw std::invalid_argument("关节角输入错误");
  }
  std::array<double, 6> arr;
  for (size_t i = 0; i < 6; ++i) {
    arr[i] = vec[i];
  }
  // arr[6] = eef_pose;

  return arr;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("play_pos_ctrl");

  program.add_argument("-i").help("can port").default_value(std::string("can0"));

  program.add_argument("-r", "--remote-ip").help("Plotjuggler IP address").default_value(std::string("127.0.0.1"));

  program.add_argument("-p", "--port").help("UDP port for Plotjuggler").scan<'i', int>().default_value(9870);

  program.add_argument("-j")
      .help("target joints of arm")
      .nargs(6)
      .default_value(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      .scan<'g', double>();

  program.add_argument("-g").help("eef pos").scan<'g', double>().default_value(0.0);

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();

  auto executor_eef = AsioExecutor::create(1);
  auto eef = EEF<1>::create<EEFType::G2, MotorType::DM>();

  std::string can = program.get<std::string>("-i");

  bool is_eef_init = true;

  if (!arm->init(executor->get_io_context(), can, 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }
  if (!eef->init(executor_eef->get_io_context(), can, 250_hz)) {
    // std::cerr << "Failed to initialize eef" << std::endl;
    std::cout << "WARNING: Failed to initialize eef." << std::endl;
    is_eef_init = false;
  }

  bool running = true;

  double eef_pos = program.get<double>("-g");
  std::vector<double> input_pos = program.get<std::vector<double>>("-j");
  std::array<double, 6> arm_cmd = vectorToArray(input_pos, eef_pos);

  if (!check_parameter(arm_cmd, eef_pos)) {
    print_joints_THRESHOLDS();
    return 0;
  }

  std::string UDP_SERVER_IP = program.get<std::string>("-r");
  int UDP_SERVER_PORT = program.get<int>("-p");

  std::array<double, 6> arm_vel = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  std::array<double, 6> arm_eff = {8, 8, 8, 8, 8, 8};

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

  auto t = std::thread([&]() {
    while (running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
      if (arm->state().is_valid) {
        auto state = arm->state();
        std::vector<double> pos(state.pos.begin(), state.pos.end());
        std::vector<double> vel(state.vel.begin(), state.vel.end());
        std::vector<double> eff(state.eff.begin(), state.eff.end());
        if (eef->state().is_valid) {
          pos.push_back(eef->state().pos[0]);
          vel.push_back(eef->state().vel[0]);
          eff.push_back(eef->state().eff[0]);
        } else {
          pos.push_back(0.0);
          vel.push_back(0.0);
          eff.push_back(0.0);
        }
        std::string state_string = boost::json::serialize(commandToJson("state", pos, vel, eff));

        try {
          socket.send_to(boost::asio::buffer(state_string), server_endpoint);
        } catch (std::exception& e) {
          std::cerr << "Failed to send state data: " << e.what() << std::endl;
        }
      }
    }
  });

  arm->enable();

  arm->set_param("arm.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  // arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  eef->enable();
  eef->set_param("control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  while (true) {
    if (is_eef_init) {
      if (!arm->state().is_valid || !eef->state().is_valid) {
        arm->pvt(arm_cmd, arm_vel, arm_eff);
        eef->pvt({eef_pos, 5.0, 0.0, 50.0, 1.0, 1.0});
      } else {
        if (!is_eef_arrive(eef_pos, eef->state().pos[0]) || !is_arm_arrive(arm_cmd, arm->state().pos)) {
          arm->pvt(arm_cmd, arm_vel, arm_eff);
          eef->pvt({eef_pos, 5.0, 0.0, 50.0, 1.0, 1.0});
        } else {
          break;
        }
      }
      std::this_thread::sleep_for(4ms);
    } else {
      if (!arm->state().is_valid) {
        arm->pvt(arm_cmd, arm_vel, arm_eff);
      } else {
        if (!is_arm_arrive(arm_cmd, arm->state().pos)) {
          arm->pvt(arm_cmd, arm_vel, arm_eff);
        } else {
          break;
        }
      }
      std::this_thread::sleep_for(4ms);
    }
  }

  running = false;
  t.join();

  socket.close();
  arm->disable();
  arm->uninit();
  eef->disable();
  eef->uninit();
  return 0;
}
