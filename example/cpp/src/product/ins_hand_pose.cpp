#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/hand/ins_hand.hpp"
#include "argparse/argparse.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

std::atomic<bool> running(true);

void signal_handler(int) { running = false; }

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

int get_key() {
  unsigned char c;
  if (read(STDIN_FILENO, &c, 1) == 1) {
    return c;
  }
  return -1;
}

bool all_zero(const std::array<double, 6>& pos) {
  for (const auto& p : pos)
    if (std::abs(p) > 1e-3) return false;
  return true;
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("airbot_ins_hand_pose");

  program.add_argument("-s", "--select").help("Initial pose: shake / praise / six").default_value(std::string("shake"));

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

  std::string pose = program.get<std::string>("-s");
  std::string can_interface = program.get<std::string>("-i");

  std::array<uint16_t, 6> shake_pose = {200, 200, 200, 200, 200, 200};
  std::array<uint16_t, 6> praise_pose = {0, 0, 1000, 1000, 1000, 1000};
  std::array<uint16_t, 6> six_pose = {0, 0, 1000, 1000, 1000, 0};

  std::vector<std::pair<std::string, std::array<uint16_t, 6>>> poses = {
      {"shake", shake_pose}, {"praise", praise_pose}, {"six", six_pose}};

  size_t pose_idx = 0;
  for (size_t i = 0; i < poses.size(); i++) {
    if (poses[i].first == pose) {
      pose_idx = i;
      break;
    }
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

  std::signal(SIGINT, signal_handler);
  set_nonblocking(true);

  auto now = std::chrono::steady_clock::now();
  std::cout << "Arm homing to zero..." << std::endl;
  while (!all_zero(arm->state().pos) && std::chrono::steady_clock::now() - now <= 10s) {
    arm->pvt({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10});
    std::this_thread::sleep_for(1ms);
  }

  std::cout << "Enter space(\' \') to shift gesture..." << std::endl;

  while (running) {
    int ch = get_key();
    if (ch == ' ') {
      pose_idx = (pose_idx + 1) % poses.size();
      std::cout << "Switched to pose: " << poses[pose_idx].first << std::endl;
    }

    HandState hand_cmd;
    std::copy_n(poses[pose_idx].second.begin(), 6, hand_cmd.positions.begin());
    std::fill_n(hand_cmd.velocities.begin(), 6, 500);
    std::fill_n(hand_cmd.forces.begin(), 6, 50);

    hand->pvt(hand_cmd);

    std::this_thread::sleep_for(100ms);
  }

  set_nonblocking(false);

  std::cout << "Exiting..." << std::endl;
  arm->disable();
  arm->uninit();
  hand->uninit();
  return 0;
}
