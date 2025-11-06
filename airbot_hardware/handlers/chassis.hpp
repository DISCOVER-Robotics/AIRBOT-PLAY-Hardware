#ifndef AIRBOT_HARDWARE_HANDLERS_CHASSIS_HPP
#define AIRBOT_HARDWARE_HANDLERS_CHASSIS_HPP

#include <any>
#include <future>
#include <memory>
#include <string>
#include <string_view>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

enum class AB_API ChassisType {
  STAT = 0x00,
};

using ChassisParams = std::unordered_map<std::string_view, std::string_view>;

struct AB_API NavTarget {
  double x = 0.0f;      // X coordinate in meters
  double y = 0.0f;      // Y coordinate in meters
  double theta = 0.0f;  // Orientation in radians
};

struct AB_API VelTarget {
  double vx = 0.0f;     // Linear velocity in m/s
  double vy = 0.0f;     // Linear velocity in m/s
  double omega = 0.0f;  // Angular velocity in rad/s
};

struct AB_API ChassisState {
  bool is_valid = true;      /** Indicates if the motor state is valid. If false, all other fields are invalid.
                              * This is useful to check if the state was received correctly or if there was an error.
                              */
  std::time_t timestamp = 0; /** Timestamp of the state in milliseconds since epoch (UNIX time) */
  double x = 0.0f;           /** X position in meters */
  double y = 0.0f;           /** Y position in meters */
  double yaw = 0.0;          /** Yaw orientation in radians */
  double vx = 0.0;           /** Linear velocity in m/s */
  double vy = 0.0;           /** Linear velocity in m/s (if applicable) */
  double omega = 0.0;        /** Linear velocity in m/s (if applicable) */

  // TODO: add interface for IMU and Laser data (`getLaserScan()` and `getImuRawData()`)

  std::string format() const noexcept {
    return "ChassisState(is_valid=" + std::to_string(is_valid) + ", x=" + std::to_string(x) +
           ", y=" + std::to_string(y) + ", yaw=" + std::to_string(yaw) + ", vx=" + std::to_string(vx) +
           ", vy=" + std::to_string(vy) + ", omega=" + std::to_string(omega) + ")";
  }
};

/**
 * @brief Chassis class that provides basic functionality for chassis control.
 * This class serves as an abstract base class for different chassis types.
 */
class AB_API Chassis {
 public:
  /**
   * @brief Factory method to create a Chassis instance.
   *
   * @tparam chassis_type The type of chassis to create
   * @return A unique_ptr to the created Chassis instance, or nullptr if creation fails.
   *
   * @note The supported chassis_type for now: ChassisType::STAT
   */
  template <ChassisType chassis_type>
  [[nodiscard]] static std::unique_ptr<Chassis> create() noexcept;

  /**
   * @brief Virtual destructor for Chassis.
   * It will clean up the resources used by the chasses.
   * @note `uninit()` is called and waited under the hood. Explicitly calling `uninit()` is not required.
   */
  virtual ~Chassis() noexcept = default;

  /**
   * @brief Initialize with the specisfied IO context and communication interface.
   * @param io_context The IO context to use for asynchronous operations.
   * @param interface The communication interface to use (e.g., "192.168.11.1").
   * @param spin_freq The frequency at which this instance should spin (in Hz). This spin frequency imposes a minimum
   * delay between adjacent writes.
   * @return true if the chassis was initialized successfully, false otherwise.
   * @note For external chassis (e.g. ChassisType::STAT), `io_context` and `spin_freq` are ignored.
   */
  [[nodiscard]]
  virtual bool init(ExecutorPtr io_context, const std::string& interface, uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize and clean up resources.
   * @return true if uninitialized successfully, false otherwise.
   * @note After calling this method, this instance should not be used anymore.
   *       If you want to use this instance again, you need to call `init()` again.
   *       This method will also stop the communication handler and delete all read callbacks.
   *       It is safe to call this method multiple times, but it will only have an effect the first time.
   */
  [[nodiscard]]
  virtual bool uninit() noexcept = 0;

  /**
   * @brief Send ping frame to obtain latest state.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref SocketcanComm::write_strong
   * for details.
   * @note This method is a no-op for external chassis (e.g. ChassisType::STAT).
   */
  [[nodiscard]]
  virtual bool ping() noexcept = 0;

  /**
   * @brief Enable the hardware.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref SocketcanComm::write_strong
   * for details.
   * @note This method is a no-op for external chassis (e.g. ChassisType::STAT).
   */
  [[nodiscard]]
  virtual bool enable() noexcept = 0;

  /**
   * @brief Disable the hardware.
   * @return false if the enabling frame (if required) is not sent successfully, true otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref SocketcanComm::write_strong
   * for details.
   * @note This method is a no-op for external chassis (e.g. ChassisType::STAT).
   */
  [[nodiscard]]
  virtual bool disable() noexcept = 0;

  /**
   * @brief Set the current pose as zero.
   * @return true if the set-zero frame is sent successfully, false otherwise.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref SocketcanComm::write_strong
   * for details.
   */
  [[nodiscard]]
  virtual bool set_zero() noexcept = 0;

  /**
   * @brief Reset the error state of the motor.
   * @return true if the reset-error frame is sent successful, false otherwise.
   * @note This method will send a CAN frame to the motor to reset the error state.
   * @note The enabling frame (if required) is sent with strong write semantics. See @ref SocketcanComm::write_strong
   * for details.
   * @note This method is not implemented for external chassis (e.g. ChassisType::STAT).
   */
  [[nodiscard]]
  virtual bool reset_error() noexcept = 0;

  /**
   * @brief Send a navigation command to the chassis.
   * @param cmd The navigation target containing x, y coordinates and orientation theta.
   * @return A future that will be resolved to true if the command was sent successfully,
   *         or false if the command could not be sent.
   * @note This method is asynchronous and returns immediately. The actual sending of the command is
   *       handled in the background. The future will be resolved when the command is sent or
   *       if an error occurs.
   */
  [[nodiscard]]
  virtual std::future<bool> navigate(const NavTarget& cmd) noexcept = 0;

  /**
   * @brief Send a velocity command to the chassis.
   * @param cmd The velocity target containing linear velocities vx, vy and angular velocity omega.
   * @return true if the command was sent successfully, false otherwise.
   * @note This method is asynchronous and returns immediately. The actual sending of the command is
   *      handled in the background.
   */
  virtual bool cmd_vel(const VelTarget& cmd) noexcept = 0;

  /**
   * @brief Save the currently built map to the specified path.
   * @param map_path The path where the map should be saved.
   * @return true if the map was saved successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool save_map(const std::string& map_path) noexcept = 0;

  /**
   * @brief Load a map from the specified path.
   * @param map_path The path from which the map should be loaded.
   * @return true if the map was loaded successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool load_map(const std::string& map_path) noexcept = 0;

  /**
   * @brief Clear the currently built map.
   * @return true if the map was cleared successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool clear_map() noexcept = 0;

  /**
   * @brief Send a request to get a parameter value from the motor.
   * @param name The name of the parameter to get.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref SocketcanComm::write_strong for details.
   */
  virtual bool get_param(std::string_view name) noexcept = 0;

  /**
   * @brief Send a request to set a parameter value on the motor.
   * @param name The name of the parameter to set.
   * @param value The value to set the parameter to.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref SocketcanComm::write_strong for details.
   */
  virtual bool set_param(std::string_view name, std::string_view value) noexcept = 0;

  /**
   * @brief Send a request to persist a parameter value on the motor.
   * @param name The name of the parameter to persist.
   * @param value The value to persist the parameter to.
   * @return true if the request was sent successfully, false otherwise.
   * @note The request frame is sent with strong write semantics. See @ref SocketcanComm::write_strong for details.
   */
  virtual bool persist_param(std::string_view name, std::string_view value) noexcept = 0;

  /**
   * @brief Update the motor state and parameters based on the received CAN frame.
   * @param result The result of parsing the received CAN frame, containing the frame type, motor state, and parameters.
   * @return true if the motor state or parameters were updated successfully, false otherwise.
   * @note This method is called by the SocketcanComm when a new CAN frame is received.
   *       It is responsible for updating the internal state of the motor based on the received frame.
   */
  virtual bool update(ParseResult&& result) noexcept = 0;

  /**
   * @brief Get the current state of the hardware.
   * @return The current state of the hardware as a ChassisState object.
   */
  virtual ChassisState state() const noexcept = 0;

  /**
   * @brief Get the current parameters of the motor.
   * @return The current parameters of the motor as a MotorParams object.
   * @note This method is thread-safe and can be called from multiple threads.
   *       It returns a copy of the current motor parameters, which are a snapshot of the parameters at the time of the
   *       call.
   */
  virtual ChassisParams params() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_HANDLERS_CHASSIS_HPP
