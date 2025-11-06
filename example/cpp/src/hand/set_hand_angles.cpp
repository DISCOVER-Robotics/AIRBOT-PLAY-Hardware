#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/arm.hpp"
#include "airbot_hardware/handlers/hand/ins_hand.hpp"

using namespace airbot::hardware;
using namespace std::chrono_literals;

int main() {
  auto executor = AsioExecutor::create(3);
  auto hand = InsHand::create<1>();

  auto arm = airbot::hardware::Arm<6>::create<MotorType::OD, MotorType::OD, MotorType::OD, MotorType::DM, MotorType::DM,
                                              MotorType::DM, EEFType::NA, MotorType::NA>();
  if (!arm->init(executor->get_io_context(), "can0", 250_hz)) {
    std::cerr << "Failed to initialize arm" << std::endl;
    return 1;
  }

  hand->init(executor->get_io_context(), "can0", 250_hz);
  hand->reset();
  hand->reset_error();

  InsHandCmd hand_cmd;
  hand_cmd.force = {500, 500, 500, 500, 500, 500};  // 固定力
  hand_cmd.vel = {500, 600, 700, 800, 100, 100};    // 固定速度

  std::array<int16_t, 6> target_pos = {1000, 1000, 1000, 1000, 1000, 1000};
  std::array<int16_t, 6> pos_output = target_pos;

  hand_cmd.pos = pos_output;
  hand->pvt(hand_cmd);

  arm->enable();

  arm->set_param("motor1.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor2.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor3.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor4.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor5.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("motor6.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));
  arm->set_param("eef.control_mode", static_cast<uint32_t>(MotorControlMode::PVT));

  while (true) {
    hand->get_param("ANGLE_ACT");
    hand->get_param("POS_ACT");
    hand->get_param("FORCE_ACT");
    auto state_hand = hand->state();
    auto current_angle = state_hand.angle_act;

    static std::array<bool, 5> forward_fingers = {true, false, true, false, true};
    static std::array<int, 5> step_fingers = {200, 300, 400, 500, 250};

    for (size_t i = 0; i < 5; i++) {
      int16_t error;
      if (forward_fingers[i]) {
        target_pos[i] += step_fingers[i];
        if (target_pos[i] > 1000) {
          target_pos[i] = 1000;
          forward_fingers[i] = false;
        }
      } else {
        target_pos[i] -= step_fingers[i];
        if (target_pos[i] < 0) {
          target_pos[i] = 0;
          forward_fingers[i] = true;
        }
      }

      error = target_pos[i] - current_angle[i];
      pos_output[i] = current_angle[i] + error;
    }

    hand_cmd.pos = pos_output;
    hand->pvt(hand_cmd);  // hand pvt command

    static bool to_minus = true;

    auto arm_state = arm->state();
    double q3 = arm_state.pos[2];

    if (to_minus && q3 <= 0.95) {
      to_minus = false;
    } else if (!to_minus && q3 >= 0.05) {
      to_minus = true;
    }

    std::array<double, 6> arm_target_pos = {0, 0, 1, 0, 0, 0};
    std::array<double, 6> arm_target_vel = {M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10};

    arm->pvt(arm_target_pos, arm_target_vel);

    std::cout << "Hand ANGLE:";
    for (auto p : pos_output) {
      std::cout << std::setw(6) << static_cast<int>(p);
    }
    std::cout << "    Act ANGLE:";
    for (auto a : current_angle) {
      std::cout << std::setw(6) << a;
    }
    std::cout << "    Arm J3 act: " << q3 << "  target: " << arm_target_pos[2] << std::endl;

    std::this_thread::sleep_for(20ms);
  }

  arm->disable();
  arm->uninit();

  hand->uninit();
  return 0;
}
