#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/eef.hpp"
#include "airbot_hardware/utils.hpp"
#include "argparse/argparse.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("arm_params");

  program.add_argument("-i").help("CAN interface to use (e.g., can0, can1)").default_value(std::string("can0"));
  program.add_argument("--eef").help("End effector type (e.g., E2, G2, NA)").default_value(std::string("NA"));

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  auto can_iface = program.get<std::string>("-i");

  bool use_e2 = false;
  bool use_g2 = false;

  auto eef_type = program.get<std::string>("--eef");
  if (eef_type == "G2" || eef_type == "g2") use_g2 = true;
  if (eef_type == "E2" || eef_type == "e2") use_e2 = true;

  auto executor = airbot::hardware::AsioExecutor::create(8);
  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();

  std::unique_ptr<EEF<1>> eef;

  if (use_e2) {
    eef = airbot::hardware::EEF<01>::create<EEFType::E2, MotorType::OD>();
    if (!eef->init(executor->get_io_context(), can_iface, 250)) {
      std::cerr << "Failed to initialize E2 on " << can_iface << std::endl;
      use_e2 = false;
    }
  }
  if (use_g2) {
    eef = airbot::hardware::EEF<1>::create<EEFType::G2, MotorType::DM>();
    if (!eef->init(executor->get_io_context(), can_iface, 250)) {
      std::cerr << "Failed to initialize G2 on " << can_iface << std::endl;
      use_g2 = false;
    }
  }

  if (!arm->init(executor->get_io_context(), can_iface, 250)) {
    std::cerr << "Failed to initialize arm on " << can_iface << std::endl;
    return 1;
  }
  arm->enable();

  auto arm_params = arm->params();

  std::vector<std::pair<std::string, ParamValue>> sorted_arm_params(arm_params.begin(), arm_params.end());
  std::sort(sorted_arm_params.begin(), sorted_arm_params.end(), [](auto& a, auto& b) { return a.first < b.first; });

  std::cout << "======== Arm Parameters ========" << std::endl;
  for (const auto& [key, value] : sorted_arm_params) {
    std::cout << key << " : " << value.format() << std::endl;
  }

  if (use_e2 || use_g2) {
    auto eef_params = eef->params();
    std::cout << "======== EEF Parameters ========" << std::endl;
    for (const auto& [key, value] : eef_params) {
      std::cout << key << " : " << value.format() << std::endl;
    }
    eef->disable();
    eef->uninit();
  }

  arm->disable();
  arm->uninit();

  return 0;
}
