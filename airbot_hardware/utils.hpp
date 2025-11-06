#ifndef AIRBOT_HARDWARE_UTILS_HPP
#define AIRBOT_HARDWARE_UTILS_HPP
#define AB_API __attribute__((visibility("default")))

#include <linux/can.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

using CanFrame = struct can_frame;
using CanFramePtr = std::shared_ptr<struct can_frame>;
using CanFilter = struct can_filter;

namespace airbot {
namespace hardware {
/**
 * @brief Parameter data types used in the system.
 *
 * `LE` and `BE` indicate little-endian and big-endian byte order, respectively.
 */
enum class AB_API ParamType {
  UINT8_LE = 0x00,
  INT8_LE = 0x01,
  UINT16_LE = 0x02,
  INT16_LE = 0x03,
  UINT32_LE = 0x04,
  INT32_LE = 0x05,
  // UINT64_LE = 0x06,
  // INT64_LE = 0x07,
  FLOAT32_LE = 0x08,
  // FLOAT64_LE = 0x09,

  UINT8_BE = 0x10,
  INT8_BE = 0x11,
  UINT16_BE = 0x12,
  INT16_BE = 0x13,
  UINT32_BE = 0x14,
  INT32_BE = 0x15,
  // UINT64_BE = 0x16,
  // INT64_BE = 0x17,
  FLOAT32_BE = 0x18,
  // FLOAT64_BE = 0x19,

  UINT8x4 = 0x20,
  // UINT8x5 = 0x21,
  STRING = 0x30,
  INT16x6 = 0x40,
  UINT16x6 = 0x41,
  FLOAT_LE_MULT = 0x42,
  FLAGS_FLOAT = 0xFC,
  FLAGS = 0xFD,
  UNTOUCHED = 0xFE,  // will not use this type for deserialization
  INVALID = 0xFF,    // will not use this type for deserialization
};

/**
 * @brief Represents a value of a parameter with its type.
 *
 * This struct can hold various types of data, including integers, floats, and arrays.
 * It provides constructors for initializing the value based on the type.
 */

struct AB_API ParamValue {
  ParamType t;
  union {
    uint8_t u8;
    int8_t i8;
    uint16_t u16;
    int16_t i16;
    uint32_t u32;
    int32_t i32;
    float f32;
    char arr[4];
  } data;

  std::shared_ptr<std::string> str;
  std::shared_ptr<std::array<int16_t, 6>> i16_array6;
  std::shared_ptr<std::array<uint16_t, 6>> ui16_array6;
  std::shared_ptr<std::vector<float>> f32s;
  std::shared_ptr<std::tuple<uint8_t, uint8_t, uint32_t>> flag_set;  // start, length, value (LE)
  std::shared_ptr<std::tuple<uint8_t, float>> flag_set_f;            // start, length, value (float)

  ParamValue() : t(ParamType::INVALID) { data.u32 = 0; }
  ParamValue(uint8_t v) : t(ParamType::UINT8_LE) { data.u8 = v; }
  ParamValue(int8_t v) : t(ParamType::INT8_LE) { data.i8 = v; }
  ParamValue(uint16_t v) : t(ParamType::UINT16_LE) { data.u16 = v; }
  ParamValue(int16_t v) : t(ParamType::INT16_LE) { data.i16 = v; }
  ParamValue(uint32_t v) : t(ParamType::UINT32_LE) { data.u32 = v; }
  ParamValue(int32_t v) : t(ParamType::INT32_LE) { data.i32 = v; }
  ParamValue(float v) : t(ParamType::FLOAT32_LE) { data.f32 = v; }
  ParamValue(uint8_t start, uint8_t length, uint32_t value) : t(ParamType::FLAGS) {
    flag_set = std::make_shared<std::tuple<uint8_t, uint8_t, uint32_t>>(start, length, value);
  }
  ParamValue(uint8_t start, float value) : t(ParamType::FLAGS_FLOAT) {
    flag_set_f = std::make_shared<std::tuple<uint8_t, float>>(start, value);
  }
  ParamValue(const std::array<uint8_t, 4> &v) : t(ParamType::UINT8x4) {
    data.arr[0] = v[0];
    data.arr[1] = v[1];
    data.arr[2] = v[2];
    data.arr[3] = v[3];
  }
  ParamValue(const std::string &s) : t(ParamType::STRING) { str = std::make_shared<std::string>(s); }

  ParamValue(const std::array<int16_t, 6> &v) : t(ParamType::INT16x6) {
    i16_array6 = std::make_shared<std::array<int16_t, 6>>(v);
  }

  ParamValue(const std::array<uint16_t, 6> &v) : t(ParamType::UINT16x6) {
    ui16_array6 = std::make_shared<std::array<uint16_t, 6>>(v);
  }

  ParamValue(const std::vector<float> &v) : t(ParamType::FLOAT_LE_MULT) {
    f32s = std::make_shared<std::vector<float>>(v);
  }

  // Python binding constructors
  ParamValue(ParamType type, int64_t v);
  ParamValue(ParamType type, float v);
  ParamValue(ParamType type, const std::array<uint8_t, 4> &v);
  ParamValue(ParamType type, const std::string &v);
  ParamValue(ParamType type, uint8_t start, uint8_t length, uint32_t value);
  ParamValue(ParamType type, uint8_t start, float value);

  std::string format() const noexcept {
    switch (t) {
      case ParamType::UINT8_LE:
      case ParamType::UINT8_BE:
        return std::to_string(data.u8);
      case ParamType::INT8_LE:
      case ParamType::INT8_BE:
        return std::to_string(data.i8);
      case ParamType::UINT16_LE:
      case ParamType::UINT16_BE:
        return std::to_string(data.u16);
      case ParamType::INT16_LE:
      case ParamType::INT16_BE:
        return std::to_string(data.i16);
      case ParamType::UINT32_LE:
      case ParamType::UINT32_BE:
        return std::to_string(data.u32);
      case ParamType::INT32_LE:
      case ParamType::INT32_BE:
        return std::to_string(data.i32);
      case ParamType::FLOAT32_LE:
      case ParamType::FLOAT32_BE:
        return std::to_string(data.f32);
      case ParamType::UINT8x4:
        return std::to_string(data.arr[0]) + "." + std::to_string(data.arr[1]) + "." + std::to_string(data.arr[2]) +
               "." + std::to_string(data.arr[3]);
      case ParamType::STRING:
        return str ? *str : "EMPTY";
      case ParamType::INT16x6:
        if (!i16_array6) return "EMPTY";
        {
          std::string s;
          for (size_t i = 0; i < i16_array6->size(); ++i) {
            if (i > 0) s += ",";
            s += std::to_string((*i16_array6)[i]);
          }
          return s;
        }
      case ParamType::UINT16x6:
        if (!ui16_array6) return "EMPTY";
        {
          std::string s;
          for (size_t i = 0; i < ui16_array6->size(); ++i) {
            if (i > 0) s += ",";
            s += std::to_string((*ui16_array6)[i]);
          }
          return s;
        }
      case ParamType::FLOAT_LE_MULT:
        if (!f32s) return "EMPTY";
        {
          std::string s;
          for (size_t i = 0; i < f32s->size(); ++i) {
            if (i > 0) s += ",";
            s += std::to_string((*f32s)[i]);
          }
          return s;
        }
      case ParamType::FLAGS:
      case ParamType::FLAGS_FLOAT:
        if (!flag_set) return "EMPTY";
        {
          std::string s;
          auto &[start, length, val] = *flag_set;
          s += std::to_string(start) + "," + std::to_string(length) + "," + std::to_string(val);
          return s;
        }
      case ParamType::INVALID:
        return "INVALID";
      default:
        return "UNKNOWN";
    }
  }
};

/**
 * @brief Parameter definition for the AIRBOT hardware.
 */
struct AB_API ParamBuilder {
  uint8_t id;                   /** param index */
  bool implemented;             /** is implemented for serialization and deserialization */
  std::string_view name;        /** param name */
  std::string_view permission;  /** read / write permission, one of "rw", "ro", "wo" */
  ParamType datatype;           /** param type, one of ParamType enum values */
  ParamValue default_value;     /** default value for the parameter, used for deserialization */
  std::string_view description; /** description of the parameter */

  /** Function to convert a raw value to a parameter value AFTER deserialized from bytes. */
  std::function<float(float)> from_raw;
  /** Function to convert a parameter value to a raw value BEFORE serialized to bytes. */
  std::function<float(float)> to_raw;

  uint8_t length;

  /**
   * @brief deserialize a 4-byte array to a ParamValue.
   * @param data The 4-byte array to deserialize.
   * @return A ParamValue object representing the deserialized data.
   * @throws std::runtime_error if the ParamType is unsupported for deserialization.
   */
  ParamValue deserialize(const std::array<uint8_t, 4> &data) const;

  /**
   * @brief serialize a ParamValue to a 4-byte array.
   * @param value The ParamValue to serialize.
   * @return A 4-byte array representing the serialized value.
   * @throws std::runtime_error if the ParamType is unsupported for serialization.
   */
  std::array<uint8_t, 4> serialize(const ParamValue &value) const;

  /**
   * @brief Get the length of the parameter in bytes.
   * @return The length of the parameter in bytes.
   * @note This is determined by the ParamType and is used to ensure correct serialization and deserialization.
   */
  inline uint8_t param_len() const noexcept {
    switch (datatype) {
      case ParamType::UINT8_LE:
      case ParamType::UINT8_BE:
      case ParamType::INT8_LE:
      case ParamType::INT8_BE:
        return 1;
      case ParamType::UINT16_LE:
      case ParamType::INT16_LE:
      case ParamType::UINT16_BE:
      case ParamType::INT16_BE:
        return 2;
      case ParamType::UINT32_LE:
      case ParamType::INT32_LE:
      case ParamType::UINT32_BE:
      case ParamType::INT32_BE:
      case ParamType::FLOAT32_LE:
      case ParamType::FLOAT32_BE:
      case ParamType::UINT8x4:
      case ParamType::INT16x6:
      case ParamType::UINT16x6:
      case ParamType::FLOAT_LE_MULT:
      default:
        return 4;
    }
  }
};

/**
 * @brief Enum representing the type of frame being processed.
 */
enum class AB_API FrameType {
  // Requests
  CSPReq = 0x00,          /** Cyclic Synchronous Position Request */
  CSVReq = 0x01,          /** Cyclic Synchronous Velocity Request */
  MITReq = 0x02,          /** MIT mode Request */
  PVTReq = 0x03,          /** Cyclic Synchronous Position Request with Current Threshold */
  GetParamReq = 0x04,     /** Get Parameter Request */
  SetParamReq = 0x05,     /** Set Parameter Request */
  PersistParamReq = 0x06, /** Persist Parameter Request */
  PingReq = 0x07,         /** Ping Request, to obtain motor state without motion command */
  EnableReq = 0x08,       /** Enable Request, to enable the motor */
  DisableReq = 0x09,      /** Disable Request, to disable the motor */
  SetZeroReq = 0x0A,      /** Set Zero Request, to set the motor position reference to zero */
  ResetErrReq = 0x0B,     /** Reset Error Request, to reset the motor error state */
  // Responses
  MotionCmdResp = 0x10,    /** Response to motion command requests (CSP, CSV, MIT, PVT) */
  GetParamResp = 0x11,     /** Response to Get Parameter Request, with param value*/
  SetParamResp = 0x12,     /** Response to Set Parameter Request, with param value*/
  PersistParamResp = 0x13, /** Response to Persist Parameter Request */
  Void = 0xFF,             /** Void response, for invalid or irrelevant responses */
  // For Old DM only
  LEDCmd = 0x20,     /** LED Command Request, to control the color and flashing frequency of LED*/
  LEDCmdResp = 0x21, /** Response to LED Command Request */
};

struct AB_API MotorCommand {
  double pos;               /** Target position in rad, used in CSP, MIT and PVT modes */
  double vel;               /** Target velocity in rad/s in CSV and MIT modes, maximum velocity in CSP and PVT modes */
  double eff;               /** Feed forward torque in Nm, used in MIT modes */
  double mit_kp;            /** MIT Kp gain, used in MIT modes */
  double mit_kd;            /** MIT Kd gain, used in MIT modes */
  double current_threshold; /** Current threshold in Amperes, used in PVT modes */
};

struct AB_API MotorState {
  bool is_valid = true;     /** Indicates if the motor state is valid. If false, all other fields are invalid.
                             * This is useful to check if the state was received correctly or if there was an error.
                             */
  uint16_t joint_id = 0x00; /** Joint ID */
  double pos = 0.0;         /** Position in radians, used in CSP, CSV, MIT and PVT modes */
  double vel = 0.0;         /** Velocity in radians per second */
  double eff = 0.0;         /** Torque in Nm */
  uint8_t motor_temp = 0;   /** Motor temperature in degrees Celsius, represented as uint8_t */
  uint8_t mos_temp = 0;     /** MOSFET temperature in degrees Celsius, represented as uint8_t */
  uint8_t error_id = 0x00;  /** Error code, 0x00 or 0x01 means no error, other values indicate specific errors */

  std::string format() const noexcept {
    return "MotorState(is_valid=" + std::to_string(is_valid) + ", joint_id=" + std::to_string(joint_id) +
           ", pos=" + std::to_string(pos) + ", vel=" + std::to_string(vel) + ", eff=" + std::to_string(eff) +
           ", motor_temp=" + std::to_string(static_cast<int>(motor_temp)) +
           ", mos_temp=" + std::to_string(static_cast<int>(mos_temp)) +
           ", error_id=" + std::to_string(static_cast<int>(error_id)) + ")";
  }
};

/**
 * @brief Enum representing the control modes for the motor.
 *
 * These modes determine how the motor will respond to commands and how it will operate.
 */
enum class AB_API MotorControlMode {
  INVALID = 0x00,
  MIT = 0x01, /** MIT mode:
               * ```
               * cmd.eff   ----------------------------------------------------+
               *                                                               |
               *                                                               |
               *                                                              (+)
               *                                                               |
               *                 +-----------+      +------------+       +-----v-----+      +-----------+
               *                 |           |      |            |       |           |      |           |
               * cmd.pos --(+)-->|           |-(*)->| cmd.mit_kp |--(+)->|           |-(*)->| 1/KT_OUT  |--> iqref
               *                 |           |      |            |       |           |      |           |
               *                 +-----|-----+      +------------+       +-----^-----+      +-----------+
               *                       |                                       |
               *                      (-)                                     (+)
               *                       |                                       |
               * state.pos ------------+                                       |
               *                                                               |
               *                 +-----------+      +------------+             |
               *                 |           |      |            |             |
               * cmd.vel --(+)-->|           |-(*)->| cmd.mit_kd |-------------+
               *                 |           |      |            |                                |
               *                 +-----|-----+      +------------+                                |
               *                       |                                                      +---v---+
               *                      (-)                                                     |   0   |--> idref
               *                       |                                                      +-------+
               * state.vel ------------+
               * ```
               */
  CSP = 0x02, /** Cyclic Synchronous Position mode */
  CSV = 0x03, /** Cyclic Synchronous Velocity mode */
  PVT = 0x04, /** Cyclic Synchronous Position mode with Current Threshold*/
};

using MotorParams = std::unordered_map<std::string_view, ParamValue>;
using ParseResult = std::tuple<FrameType, std::unique_ptr<MotorState>, MotorParams>;

/**
 * @brief Enum representing the type of motor being used.
 */
enum class AB_API MotorType {
  OD = 0x00,  /** For motor #1 - #7 */
  DM = 0x01,  /** For motor #1 - #7 */
  ODM = 0x02, /** For motor #1 - #7 */
  EC = 0x03,  /** For motor #1 - #7 */

  SIM = 0xFE, /** For DISCOVERSE simulated motor */
  NA = 0xFF,  /** For null motor type. Used */
};

/**
 * @brief User-defined literal operator for frequency in Hz.
 * @param freq Frequency value as unsigned long long.
 * @return Frequency as uint16_t.
 */
inline uint16_t AB_API operator""_hz(unsigned long long freq) noexcept { return freq; }

inline std::string format(const CanFrame &frame) noexcept {
  std::string ret;
  char can_id_buf[8];
  std::sprintf(can_id_buf, "0x%03X", frame.can_id);
  ret += "CanFrame(can_id=" + std::string(can_id_buf) + ", can_dlc=" + std::to_string(frame.can_dlc) + ", data=";
  for (int i = 0; i < frame.can_dlc; i++) {
    char buf[4];
    std::sprintf(buf, "%02X", static_cast<unsigned char>(frame.data[i]));
    ret += buf;
    if (i < frame.can_dlc - 1) ret += " ";
  }
  ret += ")";
  return ret;
}

// Type conversion utils

template <typename T>
struct is_size_4 : std::bool_constant<sizeof(T) == 4> {};

// helper variable template (since C++14)
template <typename T>
inline constexpr bool is_size_4_v = is_size_4<T>::value;

// FIXME: this impl has strong requirements on host endianness (little-endian)
template <typename T, typename = std::enable_if_t<is_size_4_v<T> && std::is_trivially_copyable_v<T>>>
inline std::array<std::uint8_t, 4> from_le(T value) {
  // copy the bits into a uint32_t
  std::uint32_t u;
  std::memcpy(&u, &value, sizeof(u));

  return {
      static_cast<std::uint8_t>((u >> 0) & 0xFF),
      static_cast<std::uint8_t>((u >> 8) & 0xFF),
      static_cast<std::uint8_t>((u >> 16) & 0xFF),
      static_cast<std::uint8_t>((u >> 24) & 0xFF),
  };
}

// crc16-check
uint16_t crc16_modbus(const uint8_t *data, std::size_t len);
}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_UTILS_HPP
