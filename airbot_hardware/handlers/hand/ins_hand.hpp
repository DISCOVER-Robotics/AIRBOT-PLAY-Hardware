#pragma once

#ifndef AB_API
#define AB_API __attribute__((visibility("default")))
#endif

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "../datatypes/hand.hpp"
#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

/**
 * @brief Abstract base class for controlling an InsHand (robotic hand).
 *
 * This class defines the interface for initializing, controlling,
 * and querying the state of an InsHand device.
 * Derived classes should provide the concrete implementation for specific hardware.
 */
class AB_API InsHand {
 public:
  InsHand() = default;
  virtual ~InsHand() = default;

  /**
   * @brief Factory method to create an InsHand instance with the specified hand ID.
   *
   * @tparam hand_id The ID of the hand to create (e.g., 1 for left hand, 2 for right hand).
   * @return A unique_ptr to the created InsHand instance, or nullptr if creation fails.
   */
  template <uint8_t hand_id>
  [[nodiscard]] static std::unique_ptr<InsHand> create() noexcept;

  /**
   * @brief Initialize the InsHand device.
   *
   * @param io_context The executor or IO context for asynchronous operations.
   * @param interface The communication interface (e.g., "can0", "ttyUSB0").
   * @param spin_freq The frequency (Hz) at which the internal update loop runs.
   * @return True if initialization succeeds, false otherwise.
   */
  virtual bool init(ExecutorPtr io_context, const std::string& interface, uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize the InsHand device and release resources.
   * @return True if successful, false otherwise.
   */
  virtual bool uninit() noexcept = 0;

  /**
   * @brief Reset the InsHand device to its default state.
   * @return True if reset succeeds, false otherwise.
   */
  virtual bool reset() const noexcept = 0;

  /**
   * @brief Save current configuration/parameters of the InsHand to non-volatile storage.
   * @return True if save succeeds, false otherwise.
   */
  virtual bool save() const noexcept = 0;

  /**
   * @brief Send a PVT (Position–Velocity–Torque/Force) command to the InsHand.
   *
   * @param cmd The command structure containing position, velocity, and force targets.
   * @return True if the command is successfully sent, false otherwise.
   */
  virtual bool pvt(const HandState& cmd) const noexcept = 0;

  /**
   * @brief Set target position.
   * @param cmd Target state.
   * @return True if successful, false otherwise.
   */
  virtual bool set_pos(const HandState& cmd) const = 0;

  /**
   * @brief Set a parameter on the InsHand device.
   *
   * @param name The parameter name.
   *              Supported options:
   *              - "baudrate"       : Communication baudrate of the hand.
   *                                    Valid values:
   *                                      (int) 115200 bps
   *                                      (int) 57600 bps
   *                                      (int) 19200 bps
   *              - "hand_id"        : The unique identifier of the hand.
   *              - "current_limit"  : Maximum allowable motor current.
   *              - "default_speed"  : Default movement speed for the fingers.
   *              - "default_force"  : Default force applied by the fingers.
   *
   * @param value The parameter value as a ParamValue variant.
   * @return True if the parameter is set successfully, false otherwise.
   */
  virtual bool set_param(std::string_view name, ParamValue value) noexcept = 0;

  /**
   * @brief Get a parameter value from the InsHand device.
   *
   * @param name The parameter name.
   *              Supported options (mapped to device registers):
   *              - "HAND_ID"     : The unique identifier of the hand.
   *              - "REDU_RATIO"  : Gear reduction ratio of the actuator.
   *                                    Valid values:
   *                                      0 -> 115200 bps
   *                                      1 ->  57600 bps
   *                                      2 ->  19200 bps
   *              - "VLTAGE"      : Supply voltage of the hand.
   *              - "POS_ACT"     : Actual position feedback.
   *              - "ANGLE_ACT"   : Actual joint angle feedback.
   *              - "FORCE_ACT"   : Actual gripping force feedback.
   *              - "CURRENT"     : Motor current measurement.
   *              - "ERROR_CODE"  : Current error code of the device.
   *              - "STATUS"      : Current status word of the device.
   *              - "TEMP"        : Temperature measurement of the device.
   *
   * @return True if the parameter is retrieved successfully, false otherwise.
   *
   * @note The retrieved value may update the internal HandState object.
   *       Make sure to call this before accessing `state()`.
   */
  virtual bool get_param(std::string_view name) const noexcept = 0;

  /**
   * @brief Perform force calibration for gesture recognition.
   *
   * This is typically used to normalize sensor readings for different gestures.
   * @return True if calibration succeeds, false otherwise.
   */
  virtual bool gesture_force_calibration() const noexcept = 0;

  /**
   * @brief Reset error status of the InsHand device.
   * @return True if error status is cleared successfully, false otherwise.
   */
  virtual bool reset_error() const noexcept = 0;

  /**
   * @brief Get the current state of the InsHand.
   *
   * @return A HandState object containing the latest measured joint positions,
   *         velocities, and forces.
   * @note Text values in HandState are only updated after calling
   *       `get_param()`. Without invoking `get_param`, the fields
   *       may contain stale or default data.
   */
  virtual HandState state() const noexcept = 0;
};
}  // namespace hardware
}  // namespace airbot
