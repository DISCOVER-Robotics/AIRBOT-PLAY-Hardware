#ifndef AIRBOT_HARDWARE_HANDLERS_EEF_HPP
#define AIRBOT_HARDWARE_HANDLERS_EEF_HPP

#include <any>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

/**
 * @brief End effector type.
 */
enum class AB_API EEFType {
  NA = 0x00,            // Not applicable, used when no EEF is present
  G2 = 0x01,            // AIRBOT G2
  E2 = 0x02,            // AIRBOT E2
  INS_RH56DFX = 0x03,   // Inspire hand RH56DFX
  INS_RH56DFTP = 0x04,  // Inspire hand RH56DFTP
};

/**
 * @brief End effector command.
 */
template <uint16_t dof>
struct AB_API EEFCommand {
  std::array<double, dof> pos;
  std::array<double, dof> vel;
  std::array<double, dof> eff;
  std::array<double, dof> mit_kp;
  std::array<double, dof> mit_kd;
  std::array<double, dof> current_threshold;
};

/**
 * @brief End effector state.
 */
template <uint16_t dof>
struct AB_API EEFState {
  bool is_valid;
  std::array<double, dof> pos;
  std::array<double, dof> vel;
  std::array<double, dof> eff;

  std::string format() const noexcept {
    std::stringstream ss;
    ss << "pos: " << pos[0] << ", vel: " << vel[0] << ", eff: " << eff[0];
    return ss.str();
  }
};

/**
 * @brief End effector parameters.
 */
using EEFParams = std::unordered_map<std::string_view, ParamValue>;

/**
 * @brief End effector class.
 *
 * Please note that EEF class is only for the composition of actuators and the
 * mapping of the states and commands from the whole end effector to each single
 * actuator.
 *
 * The actual handling of states and commands is done by the Motor class.
 *
 * @tparam dof The number of degrees of freedom of the end effector.
 */
template <uint16_t dof>
class AB_API EEF {
 public:
  /**
   * @brief Factory method to create an EEF instance.
   *
   * @tparam eef_type The type of the end effector.
   * @tparam motor_type The type of the motor that drives the end effector, e.g. MotorType::G2, MotorType::E2, etc.
   *
   * Currently, supported combinations are:
   * - EEFType::G2, MotorType::DM
   * - EEFType::G2, MotorType::ODM
   *
   * @todo TODO: add support for EEFType::E2 x MotorType::OD
   *
   * @return A unique pointer to the created EEF instance.
   */
  template <EEFType eef_type, MotorType motor_type>
  static std::unique_ptr<EEF> create() noexcept;

  virtual ~EEF() = default;

  /**
   * @brief Initialize the EEF.
   *
   * @param io_context The executor to use for the EEF.
   * @param interface The interface to use for the EEF.
   * @param spin_freq The spin frequency of the EEF.
   *
   * @return True if the EEF is initialized successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool init(ExecutorPtr io_context, const std::string& interface, uint16_t spin_freq) noexcept = 0;

  /**
   * @brief Uninitialize the EEF.
   *
   * This method would sequentially uninitialize each motor of the EEF.
   *
   * @return True if all motors of the EEF are uninitialized successfully, false otherwise.
   */
  virtual bool uninit() const noexcept = 0;

  /**
   * @brief Ping the EEF.
   *
   * This method would sequentially ping each motor of the EEF.
   *
   * @return True if all motors of the EEF are pinged successfully, false otherwise.
   */
  virtual bool ping() const noexcept = 0;

  /**
   * @brief Enable the EEF.
   *
   * This method would sequentially enable each motor of the EEF.
   *
   * @return True if all motors of the EEF are enabled successfully, false otherwise.
   */
  virtual bool enable() const noexcept = 0;

  /**
   * @brief Disable the EEF.
   *
   * This method would sequentially disable each motor of the EEF.
   *
   * @return True if all motors of the EEF are disabled successfully, false otherwise.
   */
  virtual bool disable() const noexcept = 0;

  /**
   * @brief Set the zero point reference of the EEF.
   *
   * This method would sequentially set the zero point reference of each motor of the EEF.
   *
   * @return True if all motors of the EEF are set zero point reference successfully, false otherwise.
   */
  virtual bool set_zero() const noexcept = 0;

  /**
   * @brief Reset the error of the EEF.
   *
   * This method would sequentially reset the error of each motor of the EEF.
   *
   * @return True if all motors of the EEF are reset error successfully, false otherwise.
   */
  virtual bool reset_error() const noexcept = 0;

  /**
   * @brief Send a PVT command to the EEF.
   *
   * This method would sequentially send commands to each motor of the EEF.
   *
   * @param cmd The command to send to the EEF.
   *
   * @return True if all motors of the EEF are sent PVT command successfully, false otherwise.
   */
  virtual bool pvt(const EEFCommand<dof>& cmd) const noexcept = 0;

  /**
   * @brief Send a MIT command to the EEF.
   *
   * This method would sequentially send commands to each motor of the EEF.
   *
   * @param cmd The command to send to the EEF.
   *
   * @return True if all motors of the EEF are sent MIT command successfully, false otherwise.
   */
  virtual bool mit(const EEFCommand<dof>& cmd) const noexcept = 0;

  /**
   * @brief Get a parameter value from the EEF.
   *
   * @param name The name of the parameter to get.
   *
   * @return The value of the parameter.
   */
  virtual bool get_param(std::string_view name) const noexcept = 0;

  /**
   * @brief Set a parameter value to the EEF.
   *
   * @param name The name of the parameter to set.
   * @param value The value of the parameter to set.
   *
   * @return True if the parameter is set successfully, false otherwise.
   */
  virtual bool set_param(std::string_view name, ParamValue value) const noexcept = 0;

  /**
   * @brief Persist a parameter value to the EEF.
   *
   * @param name The name of the parameter to persist.
   * @param value The value of the parameter to persist.
   *
   * @return True if the parameter is persist successfully, false otherwise.
   */
  virtual bool persist_param(std::string_view name, ParamValue value) const noexcept = 0;

  /**
   * @brief Get the state of the EEF.
   *
   * @return The state of the EEF.
   */
  virtual EEFState<dof> state() const noexcept = 0;

  /**
   * @brief Get the parameters of the EEF.
   *
   * @return The parameters of the EEF.
   */
  virtual EEFParams params() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_HANDLERS_EEF_HPP
