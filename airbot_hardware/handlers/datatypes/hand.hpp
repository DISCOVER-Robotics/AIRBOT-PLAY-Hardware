#pragma once

#ifndef AB_API
#define AB_API __attribute__((visibility("default")))
#endif

#include <array>
#include <cstdint>

namespace airbot::hardware {

enum class AB_API MotorName : uint8_t {
  ThumbFlex = 0,
  ThumbAux = 1,
  Index = 2,
  Middle = 3,
  Ring = 4,
  Pinky = 5,
  Count = 6
};

struct AB_API HandState {
  // Motor order: ThumbFlex, ThumbAux, Index, Middle, Ring, Pinky
  static constexpr std::array<MotorName, 6> names = {MotorName::ThumbFlex, MotorName::ThumbAux, MotorName::Index,
                                                     MotorName::Middle,    MotorName::Ring,     MotorName::Pinky};

  // Joint positions for each finger (unit: 0~1000; 0=open, 1000=closed)
  std::array<uint16_t, 6> positions;

  // Joint velocities for each finger (unit: -1000~1000)
  //   - For BrainCo hand: represents motion speed until turbo
  //   - For other hands: represents velocity setting in position mode
  std::array<int16_t, 6> velocities;

  // Force feedback for each finger (unit: mN)
  //   - Note: RoHand does not support ThumbAux in force mode, so forces[1] is not meaningful
  //   - Note: RoHand not support negative force, so forces[i] >= 0
  std::array<int32_t, 6> forces;
};

constexpr uint16_t QueryOnlyPos = 6;
constexpr uint16_t QueryPosVel = 12;
}  // namespace airbot::hardware
