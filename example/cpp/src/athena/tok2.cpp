#include <airbot_hardware/executors/executor.hpp>
#include <airbot_hardware/handlers/chassis.hpp>
#include <airbot_hardware/handlers/motor.hpp>
#include <iostream>

using namespace airbot::hardware;

int main() {
  auto executor = AsioExecutor::create(1);

  auto chassis = Chassis::create<ChassisType::STAT>();
  if (!chassis) {
    std::cerr << "Failed to create chassis instance." << std::endl;
    return 1;
  }
  if (!chassis->init(executor->get_io_context(), "192.168.11.1", 1000_hz)) {
    std::cerr << "Failed to initialize chassis." << std::endl;
    return 1;
  }
  if (!chassis->enable()) {
    std::cerr << "Failed to enable chassis." << std::endl;
    return 1;
  }
  auto motors = std::array{
      Motor::create<MotorType::OD, 1>(),  Motor::create<MotorType::OD, 2>(),  Motor::create<MotorType::OD, 3>(),
      Motor::create<MotorType::ODM, 4>(), Motor::create<MotorType::ODM, 5>(), Motor::create<MotorType::ODM, 6>(),
  };
  if (std::any_of(motors.begin(), motors.end(), [](const auto& motor) { return motor == nullptr; })) {
    std::cerr << "Failed to create one or more motor instances." << std::endl;
    return 1;
  }
  for (auto&& motor : motors) {
    if (!motor->init(executor->get_io_context(), "can0", 1000_hz)) {
      std::cerr << "Failed to initialize motor." << std::endl;
      return 1;
    }
    if (!motor->enable()) {
      std::cerr << "Failed to enable motor." << std::endl;
      return 1;
    }
  }

  return 0;
}
