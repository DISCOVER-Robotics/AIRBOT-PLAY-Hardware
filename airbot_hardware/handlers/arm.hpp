#ifndef AIRBOT_HARDWARE_HANDLERS_ARM_HPP
#define AIRBOT_HARDWARE_HANDLERS_ARM_HPP

#include "airbot_hardware/handlers/eef.hpp"
#include "airbot_hardware/handlers/motor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

/**
 * @brief Arm state data structure.
 *
 * This struct is used to store the state of the arm.
 * It is used to store the position, velocity, and effort of the arm.
 *
 * @tparam dof The number of degrees of freedom of the arm.
 */
template <uint16_t dof>
struct AB_API ArmState {
  bool is_valid;
  std::array<double, dof> pos;
  std::array<double, dof> vel;
  std::array<double, dof> eff;
};

// TODO add automatic arm type identification

/**
 * @brief Arm parameters data structure.
 */
using ArmParams = std::unordered_map<std::string, ParamValue>;

/**
 * @brief Arm class that provides basic functionality for arm control.
 * This class serves as an abstract base class for different arm types.
 *
 * @tparam dof The number of degrees of freedom of the arm.
 */
template <uint16_t dof>
class AB_API Arm {
 private:
  template <typename T>
  static constexpr std::array<T, dof> make_filled_array(T value) {
    std::array<T, dof> arr{};
    for (size_t i = 0; i < dof; ++i) arr[i] = value;
    return arr;
  }

 public:
  static constexpr std::array<double, dof> DEFAULT_MAX_VEL = make_filled_array(30.0);
  static constexpr std::array<double, dof> DEFAULT_MAX_EFF = make_filled_array(100.0);

  /**
   * @brief Factory method to create an Arm instance.
   *
   * The functionality of the arm is implemented by the combination of Motor and EEF.
   *
   * @tparam m1 The type of the first motor.
   * @tparam m2 The type of the second motor.
   * @tparam m3 The type of the third motor.
   * @tparam m4 The type of the fourth motor.
   * @tparam m5 The type of the fifth motor.
   * @tparam m6 The type of the sixth motor.
   *            * @tparam eef_type The type of the end-effector.
   * @tparam eef_motor The motor type of the end-effector.
   */
  template <MotorType m1, MotorType m2, MotorType m3, MotorType m4, MotorType m5, MotorType m6, EEFType eef_type,
            MotorType eef_motor>
  static std::unique_ptr<Arm<dof>> create() noexcept;

  virtual ~Arm() = default;

  /**
   * @brief Initialize the arm.
   *
   * @param io_context The executor that manages workers to actually perform IO operations.
   * @param interface The interface to use for the arm.
   * @param spin_freq The maximum frequency for this arm to try to read / write frames.
   *                  This spin frequency imposes a minimum delay between adjacent writes.
   *                  Currently only writing rate is limited by this spin frequency.
   * @return True if the arm is initialized successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool init(ExecutorPtr io_context, const std::string& interface, uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize the arm.
   *
   * @return True if the arm is uninitialized successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool uninit() noexcept = 0;

  /**
   * @brief Ping the arm.
   *
   * This method sequentially pings all the motors and the end-effector.
   *
   * @return True if the all components are pinged successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool ping() const noexcept = 0;

  /**
   * @brief Enable the arm.
   *
   * This method sequentially enables all the motors and the end-effector.
   *
   * @return True if all components are enabled successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool enable() const noexcept = 0;

  /**
   * @brief Disable the arm.
   *
   * This method sequentially disables all the motors and the end-effector.
   *
   * @return True if all components are disabled successfully, false otherwise
   */
  [[nodiscard]]
  virtual bool disable() const noexcept = 0;

  /**
   * @brief Set zero point reference of the arm.
   *
   * This method sequentially set zero point reference for all the motors and the end-effector.
   *
   * @return True if all components successfully set zero point reference, false otherwise
   */
  [[nodiscard]]
  virtual bool set_zero() const noexcept = 0;

  /**
   * @brief Reset error of the components of the arm.
   *
   * This method sequentially set zero point reference for all the motors and the end-effector.
   *
   * @return True if all components successfully set zero point reference, false otherwise
   */
  [[nodiscard]]
  virtual bool reset_error() const noexcept = 0;

  /**
   * @brief Send one Cyclic Synchronous Velocity (CSV) control frame to the arm.
   *
   * To maintain continous control of the arm, this method should be called repeatedly.
   *
   * @param vel The target velocities of each motor (angular, in rad/s) and
   *            the end effector (linear, in m/s for parallel 2-finger gripper)
   */
  virtual bool csv(const std::array<double, dof>& vel) const noexcept = 0;

  /**
   * @brief Send one position control frame to the arm, with velocity and effort limits.
   *
   * This method send one control frame to the motors and the end effector, with target
   * angular position and the maximum angular velocity and maximum effort for each component.
   * To maintain continous control of the arm, this method should be called repeatedly.
   *
   * @param pos The target position of each motor (angular, in rad) and
                the end effector (linear, in m for parallel 2-finger gripper)
   * @param max_vel The maximum velocity of each motor (angular, in rad/s) and
                    the end effector (linear, in m/s for parallel 2-finger gripper)
   * @param max_eff The maximum efforts of each motor and the end effector.
   * @todo TODO: add unit for max_eff
   */
  virtual bool pvt(const std::array<double, dof>& pos, const std::array<double, dof>& max_vel = DEFAULT_MAX_VEL,
                   const std::array<double, dof>& max_eff = DEFAULT_MAX_EFF) const noexcept = 0;
  /**
   * @brief Send one MIT control frame to the arm, with velocity and effort limits.
   *
   * To maintain continous control of the arm, this method should be called repeatedly.
   *
   * @param pos The target position of each motor (angular, in rad) and
                the end effector (linear, in m for parallel 2-finger gripper)
   * @param vel The target velocity of each motor (angular, in rad/s) and
                the end effector (linear, in m/s for parallel 2-finger gripper)
   * @param eff The feed-forward effort of each motor and the end effector
   * @param mit_kp the proportional gain in MIT control of each motor and the end effector
   * @param mit_kd the derivative gain in MIT control of each motor and the end effector
   * @todo TODO: add unit for eff
   */
  virtual bool mit(const std::array<double, dof>& pos, const std::array<double, dof>& vel,
                   const std::array<double, dof>& eff, const std::array<double, dof>& mit_kp,
                   const std::array<double, dof>& mit_kd) const noexcept = 0;

  /**
   * @brief Send a request to get a parameter value from the arm.
   *
   * To get the param of all motors and eef, use "arm.<param_name>".
   * To get the param of a single motor of effector, use "motor<N>.<param_name>".
   *
   * Available parameter names:
   * - `control_mode`: the current control mode of the motor (PVT, CSV or MIT)
   *
   * @param name The name of the parameter to get.
   *
   * @return true if the request was sent successfully, false otherwise.
   *
   * @note This method only send a request to fetch parameters. It does not wait for the response.
   *       To obtain the actual parameter value, use @ref Arm::params instead.
   */
  virtual bool get_param(std::string_view name) const noexcept = 0;

  /**
   * @brief Send a request to set a parameter value on the arm.
   *
   * To set the param of all motors and eef, use "arm.<param_name>".
   * To set the param of a single motor of effector, use "motor<N>.<param_name>".
   *
   * Available parameter names:
   * - `control_mode`: the current control mode of the motor (PVT, CSV or MIT)
   *
   * @param name The name of the parameter to set.
   *
   * @return true if the request was sent successfully, false otherwise.
   *
   * @note This method only send a request to fetch parameters. It does not wait for the response.
   * @note The response frame for set_param would contain parameters that were set. It is
   *       unnecessary to call @ref Arm::get_param after set_param.
   */
  virtual bool set_param(std::string_view name, ParamValue value) const noexcept = 0;

  /**
   * @brief Send a request to persist a parameter value on the arm.
   *
   * To persist the param of all motors and eef, use "arm.<param_name>".
   * To persist the param of a single motor of effector, use "motor<N>.<param_name>".
   *
   * Available parameter names:
   * - `control_mode`: the current control mode of the motor (PVT, CSV or MIT)
   *
   * @param name The name of the parameter to persist.
   *
   * @return true if the request was sent successfully, false otherwise.
   *
   * @note This method only send a request to fetch parameters. It does not wait for the response.
   */
  virtual bool persist_param(std::string_view name, ParamValue value) const noexcept = 0;

  /**
   * @brief Get the current state of the arm.
   *
   * @return The current state of the arm.
   *
   * @note This method is thread-safe and can be called from multiple threads.
   *       It returns a copy of the current arm state, which is a snapshot of the state at the time of the call.
   * @todo Move this to the ArmImpl class.
   */
  virtual ArmState<dof> state() const noexcept = 0;

  /**
   * @brief Get the current parameters of the arm.
   *
   * @return The current parameters of the arm.
   *
   * @note This method is thread-safe and can be called from multiple threads.
   *       It returns a copy of the current arm parameters, which is a snapshot of the parameters at the time of the
   *       call.
   */
  virtual ArmParams params() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_HANDLERS_ARM_HPP
