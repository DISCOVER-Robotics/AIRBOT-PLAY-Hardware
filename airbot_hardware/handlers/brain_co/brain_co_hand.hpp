#pragma once

#ifndef AB_API
#define AB_API __attribute__((visibility("default")))
#endif

#include <array>
#include <cstdint>
#include <future>
#include <memory>

#include "../datatypes/hand.hpp"
#include "./datatypes.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot::hardware::brain_co {

/**
 * @brief Abstract base class for BrainCo robotic hand.
 *
 * This class defines the interface for initialization, control, parameter setting,
 * and state querying of a BrainCo hand device.
 * Derived classes should implement all pure virtual methods for specific hardware.
 */
class AB_API BrainCoHand {
 public:
  /**
   * @brief Factory method to create a BrainCoHand instance.
   * @param id Device ID.
   * @return Unique pointer to BrainCoHand instance, or nullptr on failure.
   */
  static std::unique_ptr<BrainCoHand> create(const uint8_t id) noexcept;

  virtual ~BrainCoHand() = default;

  /**
   * @brief Initialize the BrainCo hand.
   * @param io_context Executor for IO operations.
   * @param interface Communication interface (e.g., "can0", "ttyUSB0").
   * @param baud_rate Baud rate.
   * @return True if initialization succeeds, false otherwise.
   */
  virtual bool init(ExecutorPtr io_context, const std::string& interface, BaudRate baud_rate) = 0;

  /**
   * @brief Uninitialize and release resources.
   * @return True if successful, false otherwise.
   */
  virtual bool uninit() = 0;

  /**
   * @brief Stop the hand's control.
   *
   * This function disables the control of the hand.
   * If the hand is currently running, it will be stopped and further control commands will be ignored
   * until the hand is reset. The internal running state will be set to false.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool stop() = 0;

  /**
   * @brief Reactivate the hand's control.
   *
   * This function re-enables the control of the hand.
   * If the hand is not running (i.e., after calling stop()), it will be reactivated and able to accept control commands
   * again. The internal running state will be set to true.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool reset() = 0;

  /**
   * @brief Enable or disable the auto query feature.
   *
   * When enabled (true), the device will automatically and periodically query its state in the background.
   * When disabled (false), state queries must be triggered manually by the user.
   *
   * @param is_enable Set to true to enable auto query, false to disable.
   * @return True if the operation succeeds, false otherwise.
   */
  virtual bool enable_auto_query(bool is_enable) noexcept = 0;

  // set

  /**
   * @brief Set device ID.
   * @param device_id New device ID.
   * @return True if successful, false otherwise.
   */
  virtual bool set_device_id(const uint8_t device_id) const = 0;

  /**
   * @brief Set baud rate.
   * @param baud_rate Baud rate.
   * @return True if successful, false otherwise.
   */
  virtual bool set_baud_rate(const BaudRate baud_rate) const = 0;

  /**
   * @brief Set force grade.
   * @param force_grade Force grade.
   * @return True if successful, false otherwise.
   */
  virtual bool set_force_grade(const ForceGrade force_grade) const = 0;

  /**
   * @brief Store parameters(baudrate or device id) and restart device.
   * @return True if successful, false otherwise.
   */
  virtual bool store_and_restart() const = 0;

  /**
   * @brief Set LED mode and color.
   * @param led_mode LED mode.
   * @param led_color LED color.
   * @return True if successful, false otherwise.
   */
  virtual bool set_led_message(const LedMode led_mode, const LedColor led_color) const = 0;

  /**
   * @brief Set turbo mode.
   * More details see https://www.brainco-hz.com/docs/revolimb-hand-dev/revo1/modbus_foundation.html
   * @param is_turbo Enable or disable turbo mode.
   * @return True if successful, false otherwise.
   */
  virtual bool set_turbo_mode(const bool is_turbo) const = 0;

  /**
   * @brief Set turbo parameters.
   * More details see https://www.brainco-hz.com/docs/revolimb-hand-dev/revo1/modbus_foundation.html
   * @param stop_time Stop time.
   * @param run_time Run time.
   * @return True if successful, false otherwise.
   */
  virtual bool set_turbo_param(const uint16_t stop_time, const uint16_t run_time) const = 0;

  /**
   * @brief Set auto calibration mode.
   * More details see https://www.brainco-hz.com/docs/revolimb-hand-dev/revo1/modbus_foundation.html
   * @param is_auto_calibration Enable or disable auto calibration.
   * @return True if successful, false otherwise.
   */
  virtual bool set_auto_calibration(const bool is_auto_calibration) const = 0;

  /**
   * @brief Perform manual calibration.
   * More details see https://www.brainco-hz.com/docs/revolimb-hand-dev/revo1/modbus_foundation.html
   * @return True if successful, false otherwise.
   */
  virtual bool ensure_manual_calibration() const = 0;

  /**
   * @brief Set target position.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_pos(const HandState& cmd) const = 0;

  /**
   * @brief Set target velocity.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_vel(const HandState& cmd) const = 0;

  /**
   * @brief Set target current.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_current(const HandState& cmd) const = 0;

  // get

  /**
   * @brief Get cached finger state (non-blocking).
   *
   * This function returns the most recently cached finger state immediately
   * without blocking. Note that the returned value may not reflect the
   * latest hardware state in real-time, since message updates are asynchronous
   * and may be delayed.
   *
   * @return Cached finger state (possibly outdated).
   */
  virtual HandState get_cached_state() const noexcept = 0;

  /**
   * @brief Get the latest finger state (blocking).
   *
   * This function queries the hardware/device directly and blocks until
   * the current finger state is retrieved. The returned value always
   * reflects the latest state at the time of the call.
   *
   * @return Latest finger state.
   */
  virtual HandState get_latest_state() const noexcept = 0;

  /**
   * @brief Get motor states.
   * @return Array of 6 motor states.
   */
  virtual std::array<MotorState, 6> get_motor_state() const noexcept = 0;

  /**
   * @brief Get device ID.
   * @return Current device ID.
   */
  virtual uint16_t get_device_id() const noexcept = 0;

  /**
   * @brief Get baud rate.
   * @return Current baud rate.
   */
  virtual BaudRate get_baud_rate() const noexcept = 0;

  /**
   * @brief Get force grade.
   * @return Current force grade.
   */
  virtual ForceGrade get_force_grade() const noexcept = 0;

  /**
   * @brief Get LED mode and color.
   * @return Pair of current LED mode and color.
   */
  virtual std::pair<LedMode, LedColor> get_led_message() const noexcept = 0;

  /**
   * @brief Get voltage.
   * @return Current voltage.
   */
  virtual uint16_t get_voltage() const noexcept = 0;

  /**
   * @brief Get firmware version.
   * @return Firmware version string.
   */
  virtual std::string get_firmware_version() const noexcept = 0;

  /**
   * @brief Get serial number.
   * @return Serial number string.
   */
  virtual std::string get_serial_number() const noexcept = 0;
};

}  // namespace airbot::hardware::brain_co
