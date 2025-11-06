#pragma once

#ifndef AB_API
#define AB_API __attribute__((visibility("default")))
#endif

#include <array>
#include <cstdint>

namespace airbot::hardware::brain_co {

enum class AB_API BaudRate : uint8_t {
  BR_115200 = 0,
  BR_57600 = 1,
  BR_19200 = 2,
};

enum class AB_API ForceGrade : uint8_t {
  Small = 1,
  Normal = 2,
  Full = 3,
};

enum class AB_API LedMode : uint8_t {
  ShutDown = 1,
  Constant = 2,
  Flash1Hz = 3,
  FlashOnce = 4,
  FlashHalf1Hz = 5,
  Flash2Hz = 6,
};

enum class AB_API LedColor : uint8_t {
  Red = 1,
  Green = 2,
  Yellow = 3,
  Blue = 4,
  Purple = 5,
  Cyan = 6,
  White = 7,
};

enum class AB_API MotorState : uint8_t {
  MotorIdle = 0,
  MotorRunning = 1,
  MotorStall = 2,
};

enum class AB_API BrainCoHandType : uint8_t {
  Revo1MR = 0,  // 1代基础版
  // Revo1MT1 = 1, // 1代触觉版
  // Revo2XR = 2, // 2代基础版
  // Revo2XE = 3, // 2代进阶版
  // Revo2XT = 4, // 2代触觉版
};

}  // namespace airbot::hardware::brain_co
