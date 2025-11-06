#ifndef AIRBOT_HARDWARE_PARSERS_HPP
#define AIRBOT_HARDWARE_PARSERS_HPP

#include <linux/can.h>

#include <vector>

#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

enum class AB_API ParserType {
  NONE = 0x00,
  OD_MOTOR = 0x01,
  DM_MOTOR = 0x02,
  ODM_MOTOR = 0x03,
  ENCODER = 0x04,
  SIM_MOTOR = 0xFE,
  NULL_MOTOR = 0xFF,
  PLAY_BOARD = 0x30,
  REPLAY_BOARD = 0x31,
  PLAY_LED = 0x10,
  PLAY_BUTTON = 0x11,
  PLAY_ARM_PARAM = 0x20,
};

/**
 * @brief FrameParser is an abstract base class for parsing and generating CAN frames for different motor types.
 * It provides a factory method to create specific motor parsers based on the motor type and ID.
 * @tparam frame_type The type of CAN frame to use (e.g., CanFrame).
 * @note The parser would not hold any runtime state of the motor (except for the necessary parameters for generating
 *       the frames, e.g., maximum current threshold, position limits, etc.), but it is responsible for generating and
 *       parsing CAN frames for the motor. The necessary parameters are also initialized with default values, which
 *       could be but are not advised to be changed by the user after the parser is created.
 */
template <typename frame_type>
class AB_API FrameParser {
 public:
  using Frame = frame_type;
  using FramePtr = std::shared_ptr<frame_type>;
  /**
   * @brief Factory method to create a Parser instance.
   *
   * @tparam motor_type The type of motor to create (e.g., MotorType::OD, MotorType::DM, MotorType::ODM).
   * @tparam motor_id The ID of the motor to create (e.g., 1, 2, 3, etc.).
   * @return A unique_ptr to the created FrameParser instance, or nullptr if creation fails.
   *
   * @note The supported combinations of motor_type and motor_id are:
   *
   *       - MotorType::OD with motor_id 1, 2, 3, or 7
   *
   *       - MotorType::DM with motor_id 4, 5, 6, or 7
   *
   *       - MotorType::ODM with motor_id 4, 5, 6, or 7
   */
  template <ParserType parser_type, uint16_t parser_id>
  [[nodiscard]] static std::unique_ptr<FrameParser> create() noexcept;

  /**
   * @brief generate a motor command frame.
   * @param type The type of frame to generate: `FrameType::MITCmdReq`, `FrameType::CSPReq`, `FrameType::CSVReq`, or
   * `FrameType::PVTReq`.
   * @param cmd The motor command to generate the frame for.
   * @return A shared pointer to the generated CAN frame.
   */
  [[nodiscard]]
  virtual std::vector<FramePtr> generate(const FrameType& /* type */, const MotorCommand& /* cmd */) const noexcept = 0;

  /**
   * @brief Generate a CAN frame for getting or setting a specific parameter.
   * @param type The type of frame to generate: `FrameType::GetParamReq`, `FrameType::SetParamReq`, or
   * `FrameType::PersistParamReq`.
   * @param name The name of the parameter to generate the frame for.
   * @param value The value of the parameter to generate the frame for (default is an empty ParamValue).
   * @return A shared pointer to the generated CAN frame.
   */
  [[nodiscard]]
  virtual std::vector<FramePtr> generate(const FrameType& /* type */, std::string_view /* name */,
                                         ParamValue /* value */ = {}) const noexcept = 0;

  /**
   * @brief Generate a CAN frame for a specific frame type.
   * @param type The type of frame to generate: `FrameType::PingReq`, `FrameType::EnableReq`, `FrameType::DisableReq`,
   * `FrameType::SetZeroReq`, or `FrameType::ResetErrReq`.
   * @return A shared pointer to the generated CAN frame.
   * @note This method is used for generating frames that do not require additional parameters or commands.
   */
  [[nodiscard]]
  virtual std::vector<FramePtr> generate(const FrameType& /* type */) const noexcept = 0;

  /**
   * @brief Parse a CAN frame and extract the motor state and parameters.
   * @param frame The CAN frame to parse.
   * @return A ParseResult containing the frame type, motor state, and parameters.
   */
  [[nodiscard]]
  virtual ParseResult parse(const Frame& /* frame */) const noexcept = 0;

  /**
   * @brief Get the definition of the parameters supported by this motor parser.
   * @return A vector of ParamBuilder objects representing the parameters supported by this motor parser.
   */
  virtual std::vector<ParamBuilder> params() const noexcept = 0;

  virtual ParamBuilder param(std::string_view name) const noexcept {
    auto params = this->params();
    auto it = std::find_if(params.begin(), params.end(), [name](const ParamBuilder& p) { return p.name == name; });
    if (it == params.end()) return ParamBuilder{};
    return *it;
  }
  virtual ParamBuilder param(uint8_t param_id) const noexcept {
    auto params = this->params();
    auto it =
        std::find_if(params.begin(), params.end(), [param_id](const ParamBuilder& p) { return p.id == param_id; });
    if (it == params.end()) return ParamBuilder{};
    return *it;
  }
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_PARSERS_HPP
