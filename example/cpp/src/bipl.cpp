#include <chrono>
#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/gpio.hpp"
#include "airbot_hardware/utils.hpp"

using namespace airbot::hardware;

int main() {
  auto executor = AsioExecutor::create(8);
  auto base_board = GPIO::create<GPIOType::PLAY_BASE_BOARD>();
  auto end_board = GPIO::create<GPIOType::PLAY_END_BOARD>();
  if (!base_board->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize base board" << std::endl;
    return 1;
  }
  if (!end_board->init(executor->get_io_context(), "can0", 250)) {
    std::cerr << "Failed to initialize end board" << std::endl;
    return 1;
  }
  base_board->add_callback("button", "button", [](const ParamValue& params) {
    std::cout << "base button pressed: " << params.format() << std::endl;
  });
  end_board->add_callback("button", "button", [](const ParamValue& params) {
    std::cout << "end button pressed: " << params.format() << std::endl;
  });

  for (const auto& [name, value] : base_board->params()) {
    std::cout << name << ": " << value.format() << std::endl;
  }
  base_board->set_param("craft_flag", ParamValue(20, 4, 0x0F));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  for (const auto& [name, value] : base_board->params()) {
    std::cout << name << ": " << value.format() << std::endl;
  }

  std::cout << "start" << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(0, 1, 0x00));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(4, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(0, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(1, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(2, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(3, 1, 0x01));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(4, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(3, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(4, 1, 0x03));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  base_board->set_param("manufacture_flag", ParamValue(2, 1, 0x00));
  std::cout << base_board->params()["manufacture_flag"].format() << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << base_board->params()["product_sn"].format() << std::endl;
  base_board->set_param("product_sn", ParamValue("PZ27C02404000211"));
  // base_board->get_param("product_sn");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << base_board->params()["product_sn"].format() << std::endl;

  std::cout << base_board->params()["gravity_comp_param"].format() << std::endl;
  base_board->set_param("gravity_comp_param", ParamValue(0x12, 0.48));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << base_board->params()["gravity_comp_param"].format() << std::endl;

  auto effects = std::array{
      LightEffect::RED_CONSTANT,     LightEffect::ORANGE_CONSTANT,  LightEffect::YELLOW_CONSTANT,
      LightEffect::GREEN_CONSTANT,   LightEffect::CYAN_CONSTANT,    LightEffect::BLUE_CONSTANT,
      LightEffect::PURPLE_CONSTANT,  LightEffect::WHITE_CONSTANT,   LightEffect::RED_BREATHING,
      LightEffect::ORANGE_BREATHING, LightEffect::YELLOW_BREATHING, LightEffect::GREEN_BREATHING,
      LightEffect::CYAN_BREATHING,   LightEffect::BLUE_BREATHING,   LightEffect::PURPLE_BREATHING,
      LightEffect::WHITE_BREATHING,  LightEffect::RED_FLASHING,     LightEffect::ORANGE_FLASHING,
      LightEffect::YELLOW_FLASHING,  LightEffect::GREEN_FLASHING,   LightEffect::CYAN_FLASHING,
      LightEffect::BLUE_FLASHING,    LightEffect::PURPLE_FLASHING,  LightEffect::WHITE_FLASHING,
      LightEffect::RED_WAVE,         LightEffect::ORANGE_WAVE,      LightEffect::YELLOW_WAVE,
      LightEffect::GREEN_WAVE,       LightEffect::CYAN_WAVE,        LightEffect::BLUE_WAVE,
      LightEffect::PURPLE_WAVE,      LightEffect::WHITE_WAVE,       LightEffect::RAINBOW_WAVE,
  };

  for (auto effect : effects) {
    base_board->set_param("light_effect", ParamValue(static_cast<uint8_t>(effect)));
    std::cout << "hi" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  base_board->set_param("light_effect", ParamValue(static_cast<uint8_t>(LightEffect::WHITE_CONSTANT)));
  auto start_time = std::chrono::steady_clock::now();

  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
