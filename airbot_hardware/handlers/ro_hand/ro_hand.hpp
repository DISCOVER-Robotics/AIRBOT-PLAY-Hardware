#pragma once

#ifndef AB_API
#define AB_API __attribute__((visibility("default")))
#endif

#include <array>
#include <cstdint>
#include <memory>

// #include "./datatypes.hpp"
#include "../datatypes/hand.hpp"
#include "airbot_hardware/executors/executor.hpp"

namespace airbot::hardware::ro_hand {

class AB_API RoHand {
 public:
  /**
   * @brief Factory method to create a RoHand instance.
   * @param id Device ID.
   * @return Unique pointer to RoHand instance, or nullptr on failure.
   */
  static std::unique_ptr<RoHand> create(const uint8_t id) noexcept;

  virtual ~RoHand() = default;

  /**
   * @brief Initialize the RoHand.
   * @param io_context Executor for IO operations.
   * @param interface Communication interface (e.g., "can0", "ttyUSB0").
   * @return True if initialization succeeds, false otherwise.
   */
  virtual bool init(ExecutorPtr io_context, const std::string& interface) = 0;

  /**
   * @brief Uninitialize and release resources.
   * @return True if successful, false otherwise.
   */
  virtual bool uninit() = 0;

  /**
   * @brief Stop the hand. if the hand is running, it will be stopped. running will be set to false.
   * @return True if successful, false otherwise.
   */
  virtual bool stop() = 0;

  /**
   * @brief Reset the hand. if the hand is not running, it will be reset. running will be set to true.
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

  /**
   * @brief Set target position.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_pos(const HandState& cmd) const = 0;

  /**
   * @brief Set target position.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_force(const HandState& cmd) const = 0;

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
};

}  // namespace airbot::hardware::ro_hand
