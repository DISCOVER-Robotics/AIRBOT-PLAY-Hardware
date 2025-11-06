#ifndef AIRBOT_HARDWARE_HANDLERS_GPIO_HPP
#define AIRBOT_HARDWARE_HANDLERS_GPIO_HPP

#include <memory>
#include <string>
#include <string_view>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

using GPIOParams = std::unordered_map<std::string_view, ParamValue>;
using ParamsCallback = std::function<void(const ParamValue&)>;

enum class AB_API GPIOType {
  NONE = 0x00,
  PLAY_BASE_BOARD = 0x10,
  PLAY_END_BOARD = 0x11,
  REPLAY_BASE_BOARD = 0x12,
};

#ifdef MANUFACTURE
enum class ManufactureFlag {
  UNSET = 0x00,
  TESTED = 0x01,
  PASSED = 0x03,
};
#endif

enum class AB_API ButtonState {
  IDLE = 0x00,
  PRESSED = 0x01,
  LONG_PRESSED = 0x02,
  DOUBLE_CLICKED = 0x03,
};

enum class AB_API LightEffect {
  NONE = 0x00,
  RED_CONSTANT = 0x01,
  ORANGE_CONSTANT = 0x02,
  YELLOW_CONSTANT = 0x03,
  GREEN_CONSTANT = 0x04,
  CYAN_CONSTANT = 0x05,
  BLUE_CONSTANT = 0x06,
  PURPLE_CONSTANT = 0x07,
  WHITE_CONSTANT = 0x0F,
  RED_BREATHING = 0x11,
  ORANGE_BREATHING = 0x12,
  YELLOW_BREATHING = 0x13,
  GREEN_BREATHING = 0x14,
  CYAN_BREATHING = 0x15,
  BLUE_BREATHING = 0x16,
  PURPLE_BREATHING = 0x17,
  WHITE_BREATHING = 0x1F,
  RED_FLASHING = 0x21,
  ORANGE_FLASHING = 0x22,
  YELLOW_FLASHING = 0x23,
  GREEN_FLASHING = 0x24,
  CYAN_FLASHING = 0x25,
  BLUE_FLASHING = 0x26,
  PURPLE_FLASHING = 0x27,
  WHITE_FLASHING = 0x2F,
  RED_WAVE = 0x31,
  ORANGE_WAVE = 0x32,
  YELLOW_WAVE = 0x33,
  GREEN_WAVE = 0x34,
  CYAN_WAVE = 0x35,
  BLUE_WAVE = 0x36,
  PURPLE_WAVE = 0x37,
  WHITE_WAVE = 0x3F,
  RED_MOVEON = 0x41,
  ORANGE_MOVEON = 0x42,
  YELLOW_MOVEON = 0x43,
  GREEN_MOVEON = 0x44,
  CYAN_MOVEON = 0x45,
  BLUE_MOVEON = 0x46,
  PURPLE_MOVEON = 0x47,
  WHITE_MOVEON = 0x4F,
  RED_MOVEOFF = 0x51,
  ORANGE_MOVEOFF = 0x52,
  YELLOW_MOVEOFF = 0x53,
  GREEN_MOVEOFF = 0x54,
  CYAN_MOVEOFF = 0x55,
  BLUE_MOVEOFF = 0x56,
  PURPLE_MOVEOFF = 0x57,
  WHITE_MOVEOFF = 0x5F,
  RAINBOW_WAVE = 0xFF,
};

struct AB_API GPIOEvent {
  GPIOType gpio_id = GPIOType::NONE;
  uint32_t gpio_value = UINT32_MAX;

  GPIOEvent(GPIOType gpio_id, uint32_t gpio_value) : gpio_id(gpio_id), gpio_value(gpio_value) {}
};

/**
 * @brief GPIO class that provides basic functionality for GPIO control.
 * This class serves as an abstract base class for different GPIO types.
 */
class AB_API GPIO {
 public:
  template <GPIOType gpio_type>
  [[nodiscard]] static std::unique_ptr<GPIO> create() noexcept;

  virtual ~GPIO() = default;

  /**
   * @brief Initialize the GPIO.
   *
   * @param io_context The IO context to use for asynchronous operations.
   * @param interface The interface to use (e.g., "can0").
   * @param spin_freq The frequency at which the GPIO should spin (in Hz). This spin frequency imposes a minimum delay
   *                  between adjacent writes.
   * @return true if the GPIO was initialized successfully, false otherwise.
   */
  [[nodiscard]] virtual bool init(ExecutorPtr io_context, const std::string& interface,
                                  uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize the GPIO.
   *
   * @return true if the GPIO was uninitialized successfully, false otherwise.
   */
  [[nodiscard]] virtual bool uninit() noexcept = 0;

  /**
   * @brief Get a parameter value from the GPIO.
   *
   * @param name The name of the parameter to get.
   * @return The value of the parameter.
   */
  virtual bool get_param(std::string_view name) noexcept = 0;

  /**
   * @brief Set a parameter value to the GPIO.
   *
   * @param name The name of the parameter to set.
   * @param value The value of the parameter to set.
   * @return true if the parameter was set successfully, false otherwise.
   */
  virtual bool set_param(std::string_view name, ParamValue value) noexcept = 0;

  /**
   * @brief Persist a parameter value to the GPIO.
   *
   * @param name The name of the parameter to persist.
   * @param value The value of the parameter to persist.
   * @return true if the parameter was persisted successfully, false otherwise.
   */
  virtual bool persist_param(std::string_view name, ParamValue value) noexcept = 0;

  /**
   * @brief Add a callback to the GPIO.
   *
   * @param name The name of the parameter to add the callback to.
   * @param param_name The name of the parameter to add the callback to.
   * @param callback The callback to add.
   * @return true if the callback was added successfully, false otherwise.
   */
  virtual bool add_callback(std::string_view name, std::string_view param_name, ParamsCallback callback) noexcept = 0;

  /**
   * @brief Remove a callback from the GPIO.
   *
   * @param name The name of the parameter to remove the callback from.
   * @param param_name The name of the parameter to remove the callback from.
   * @return true if the callback was removed successfully, false otherwise.
   */
  virtual bool remove_callback(std::string_view name, std::string_view param_name) noexcept = 0;

  /**
   * @brief Get the parameters of the GPIO.
   *
   * @return The parameters of the GPIO.
   */
  virtual GPIOParams params() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_HANDLERS_GPIO_HPP
