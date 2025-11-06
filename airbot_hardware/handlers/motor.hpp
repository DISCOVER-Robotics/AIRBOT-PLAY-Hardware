#ifndef AIRBOT_HARDWARE_HANDLERS_MOTOR_MOTOR_HPP
#define AIRBOT_HARDWARE_HANDLERS_MOTOR_MOTOR_HPP

#include <memory>
#include <string>
#include <string_view>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {
/**
 * @brief Motor class that provides basic functionality for motor control.
 * This class serves as an abstract base class for different motor types.
 */
class AB_API Motor {
 public:
  /**
   * @brief Factory method to create a Motor instance.
   *
   * @tparam motor_type The type of motor to create (e.g., `MotorType::OD`).
   * @tparam motor_id The ID of the motor to create (e.g., 1, 2, 3, etc.).
   * @return A unique_ptr to the created Motor instance, or nullptr if creation fails.
   *
   * @note The supported combinations of motor_type and motor_id are:
   *
   *       - None: `MotorType::NA` with motor_id 1-7
   *       - OD Motor: `MotorType::OD` with motor_id 1-7
   *       - DM Motor: `MotorType::DM` with motor_id 1-7
   *       - ODM Motor: `MotorType::ODM` with motor_id 1-7
   *       - Encoder: `MotorType::EC` with motor_id 1-7
   */
  template <MotorType motor_type, uint16_t motor_id>
  [[nodiscard]] static std::unique_ptr<Motor> create() noexcept;

  /**
   * @brief Virtual destructor for Motor.
   * It will clean up the resources used by the motor.
   * @note `uninit()` is called and waited under the hood. Explicitly calling `uninit()` is not required.
   */
  virtual ~Motor() = default;

  /**
   * @brief Initialize the motor with the specified IO context and communication interface.
   * @param io_context The IO context to use for asynchronous operations.
   * @param interface The communication interface to use (e.g., "can0").
   * @param spin_freq The frequency at which the motor should spin (in Hz). This spin frequency imposes a minimum delay
   *                  between adjacent writes.
   * @return true if the motor was initialized successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool init(ExecutorPtr io_context, const std::string& interface, uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize the motor and clean up resources.
   * @return true if the motor was uninitialized successfully, false otherwise.
   * @note After calling this method, the motor should not be used anymore.
   *       If you want to use the motor again, you need to call `init()` again.
   *       This method will also stop the communication handler and delete all read callbacks.
   *       It is safe to call this method multiple times, but it will only have an effect the first time.
   */
  [[nodiscard]]
  virtual bool uninit() noexcept = 0;

  /**
   * @brief Send ping frame to obtain latest motor state.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref CommHandler::write_strong
   * for details.
   * @note This method is only implemented for DM motors and encoder for now. When ping is not unsupported, `true` is
   * always returned.
   */
  [[nodiscard]]
  virtual bool ping() noexcept = 0;

  /**
   * @brief Enable the motor.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref CommHandler::write_strong
   * for details.
   * @note The semantics of this method depend on the motor type:
   *
   *       - For OD motors, it is a no-op and returns `true`. OD motors will release the brake automatically when a
   *         motion command is sent.
   *       - For DM motors, it will explicitly send a CAN frame to enable the motor. After being enabled, the motor will
   *         enter the last state it was in before being disabled.
   */
  [[nodiscard]]
  virtual bool enable() noexcept = 0;

  /**
   * @brief Disable the motor.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref CommHandler::write_strong
   * for details.
   * @note The semantics of this method depend on the motor type:
   *
   *       - For OD motors, it is a no-op and returns `true`. OD motors will lock the brake automatically when no
   *         motion command is sent for 500ms.
   *       - For DM motors, it will explicitly send a CAN frame to disable the motor. After being disabled, the motor
   *         will stop all motion and enter a safe state (with large resistance to motion).
   */
  [[nodiscard]]
  virtual bool disable() noexcept = 0;

  /**
   * @brief Set the current position as zero.
   * @return true if the set-zero frame is sent successfully, false otherwise.
   * @note This method will send a CAN frame to the motor to set the current position as zero.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref CommHandler::write_strong
   * for details.
   * @note This method is only implemented for DM motors for now. For other motors, `false` is returned
   */
  virtual bool set_zero() noexcept = 0;

  /**
   * @brief Reset the error state of the motor.
   * @return true if the reset-error frame is sent successful, false otherwise.
   * @note This method will send a CAN frame to the motor to reset the error state.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref CommHandler::write_strong
   * for details.
   * @note This method is only implemented for DM motors for now. For other motors, `false` is returned.
   */
  virtual bool reset_error() noexcept = 0;

  /**
   * @brief Send a Cyclic Synchronous Position (CSP) command to the motor.
   * @param cmd The motor command containing position, velocity, and other parameters.
   * Effective command entries are:
   * ```
   * struct MotorCommand {
   *   float pos;                   // Target position in radians
   *   float vel;                   // Maximum velocity in radians per second
   *   float eff;                   // Not used in CSP
   *   float mit_kp;                // Not used in CSP
   *   float mit_kd;                // Not used in CSP
   *   float current_threshold;     // Not used in CSP
   * };
   * ```
   * @return true if the command was sent successfully, false otherwise.
   * @note The command frame is sent with weak write semantics. See @ref CommHandler::write_weak for details.
   */
  virtual bool csp(const MotorCommand& cmd) noexcept = 0;

  /**
   * @brief Send a Cyclic Synchronous Velocity (CSV) command to the motor.
   * @param cmd The motor command containing position, velocity, and other parameters.
   * Effective command entries are:
   * ```
   * struct MotorCommand {
   *   float pos;                    // Not used in CSV
   *   float vel;                    // Target velocity in radians per second
   *   float eff;                    // Not used in CSV
   *   float mit_kp;                 // Not used in CSV
   *   float mit_kd;                 // Not used in CSV
   *   float current_threshold;      // Not used in CSV
   * };
   * ```
   * @return true if the command was sent successfully, false otherwise.
   * @note The command frame is sent with weak write semantics. See @ref CommHandler::write_weak for details.
   */
  virtual bool csv(const MotorCommand& cmd) noexcept = 0;

  /**
   * @brief Send a MIT command to the motor.
   * @param cmd The motor command containing position, velocity, and other parameters.
   * Effective command entries are:
   * ```
   * struct MotorCommand {
   *   float pos;                   // Target position in radians
   *   float vel;                   // Target velocity in radians per second
   *   float eff;                   // Feed forward torque in Nm
   *   float mit_kp;                // Proportional gain
   *   float mit_kd;                // Derivative gain
   *   float current_threshold;     // Not used in MIT
   * };
   * ```
   * @return true if the command was sent successfully, false otherwise.
   * @note The command frame is sent with weak write semantics. See @ref CommHandler::write_weak for details.
   */
  virtual bool mit(const MotorCommand& cmd) noexcept = 0;

  /**
   * @brief Send a position command with velocity and current threshold to the motor.
   * @param cmd The motor command containing position, velocity, and other parameters.
   * Effective command entries are:
   * ```
   * struct MotorCommand {
   *   float pos;                   // Position in radians
   *   float vel;                   // Maximum velocity in radians per second
   *   float eff;                   // Not used in PVT
   *   float mit_kp;                // Not used in PVT
   *   float mit_kd;                // Not used in PVT
   *   float current_threshold;     // Maximum current threshold in Amperes
   * };
   * ```
   * @return true if the command was sent successfully, false otherwise.
   * @note The command frame is sent with weak write semantics. See @ref CommHandler::write_weak for details.
   */
  virtual bool pvt(const MotorCommand& cmd) noexcept = 0;

  /**
   * @brief Send a request to get a parameter value from the motor.
   * @param name The name of the parameter to get.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref CommHandler::write_strong for details.
   * @note This method is only responsible for sending a frame to fetch parameters. It does not wait for the response.
   *       To obtain the actual parameter value, use @ref Motor::params instead.
   */
  virtual bool get_param(std::string_view name) noexcept = 0;

  /**
   * @brief Send a request to set a parameter value on the motor.
   * @param name The name of the parameter to set.
   * @param value The value to set the parameter to.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref CommHandler::write_strong for details.
   */
  virtual bool set_param(std::string_view name, ParamValue value) noexcept = 0;

  /**
   * @brief Send a request to persist a parameter value on the motor.
   * @param name The name of the parameter to persist.
   * @param value The value to persist the parameter to.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref CommHandler::write_strong for details.
   */
  virtual bool persist_param(std::string_view name, ParamValue value) noexcept = 0;

  /**
   * @brief Update the motor state and parameters based on the received CAN frame.
   * @param result The result of parsing the received CAN frame, containing the frame type, motor state, and parameters.
   * @return true if the motor state or parameters were updated successfully, false otherwise.
   * @note This method is called by the CommHandler when a new CAN frame is received.
   *       It is responsible for updating the internal state of the motor based on the received frame.
   * @todo Move this to the MotorImpl class.
   */
  virtual bool update(ParseResult&& result) noexcept = 0;

  /**
   * @brief Get the current state of the motor.
   * @return The current state of the motor as a MotorState object.
   * @note This method is thread-safe and can be called from multiple threads.
   *       It returns a copy of the current motor state, which is a snapshot of the state at the time of the call.
   *       The state includes information such as position, velocity, torque, motor temperature, MOSFET temperature,
   *       and error code.
   */
  virtual MotorState state() const noexcept = 0;

  /**
   * @brief Get the current state of the motor.
   * @return The current state of the motor as a MotorState object.
   * @note This method is deprecated and will be removed in future versions.
   *       Use `state()` instead, which is the preferred method to get the current motor state.
   * @deprecated Use `state()` instead.
   */
  [[deprecated]]
  virtual MotorState get_state() const noexcept {
    return state();
  }

  /**
   * @brief Get the current parameters of the motor.
   * @return The current parameters of the motor as a MotorParams object.
   * @note This method is thread-safe and can be called from multiple threads.
   *       It returns a copy of the current motor parameters, which are a snapshot of the parameters at the time of the
   *       call.
   */
  virtual MotorParams params() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_HANDLERS_MOTOR_MOTOR_HPP
