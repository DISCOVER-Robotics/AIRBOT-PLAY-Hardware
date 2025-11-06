#include <iostream>
#include <thread>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/handlers/gpio.hpp"
#include "airbot_hardware/utils.hpp"

using namespace airbot::hardware;

int main() {
  auto executor = AsioExecutor::create(8);
  auto gpio = GPIO::create<GPIOType::PLAY_BASE_BOARD>();
  gpio->init(executor->get_io_context(), "can0", 100);
  gpio->add_callback("button", "button",
                     [](const ParamValue& params) { std::cout << "button pressed: " << params.format() << std::endl; });

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
    gpio->set_param("light_effect", ParamValue(static_cast<uint8_t>(effect)));
    std::cout << "hi" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  gpio->set_param("light_effect", ParamValue(static_cast<uint8_t>(LightEffect::WHITE_CONSTANT)));
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
