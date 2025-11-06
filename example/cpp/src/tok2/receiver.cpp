#include <linux/input.h>

#include <airbot_hardware/comm/comm.hpp>
#include <airbot_hardware/executors/executor.hpp>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

template <size_t frame_size>
static std::string format(const std::array<uint8_t, frame_size>& frame, const std::string& prefix = "") {
  std::ostringstream oss;
  auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  oss << prefix << " ";
  oss << "[" << std::setw(3) << std::setfill('0') << timestamp.count() << "] ";
  for (const auto& byte : frame) {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
  }
  return oss.str();
}

static constexpr size_t frame_size = 32;
static constexpr size_t freq = 500;

using Frame = std::array<uint8_t, frame_size>;

#pragma pack(push, 1)
struct TOK2Message {
  float left_arm[7];
  float right_arm[7];
  uint32_t base;

  struct Mapped {
    uint16_t left_arm[7];
    uint16_t right_arm[7];
    uint32_t base;
  };

  inline static uint16_t map_to(float value) {
    return static_cast<uint16_t>((value + M_PI) / (2 * M_PI) * std::numeric_limits<uint16_t>::max());
  }

  inline static float map_from(uint16_t value) {
    return (static_cast<float>(value) / std::numeric_limits<uint16_t>::max()) * 2 * M_PI - M_PI;
  }

  Frame serialize() const {
    Mapped mapped;
    for (size_t i = 0; i < 7; ++i) {
      mapped.left_arm[i] = map_to(left_arm[i]);
      mapped.right_arm[i] = map_to(right_arm[i]);
    }
    mapped.base = base;
    Frame frame;
    std::memcpy(frame.data(), &mapped, sizeof(Mapped));
    return frame;
  }

  std::string format() const {
    std::stringstream oss;
    oss << "Left Arm: ";
    for (size_t i = 0; i < 7; ++i) {
      oss << std::fixed << std::setprecision(2) << left_arm[i] << " ";
    }
    oss << " Right Arm: ";
    for (size_t i = 0; i < 7; ++i) {
      oss << std::fixed << std::setprecision(2) << right_arm[i] << " ";
    }
    oss << " Base: " << base;
    return oss.str();
  }

  static TOK2Message deserialize(const Frame& frame) {
    Mapped mapped;
    if (frame.size() < sizeof(Mapped)) {
      throw std::runtime_error("Frame size is too small for Mapped structure");
    }
    std::memcpy(&mapped, frame.data(), sizeof(Mapped));
    TOK2Message msg;
    for (size_t i = 0; i < 7; ++i) {
      msg.left_arm[i] = map_from(mapped.left_arm[i]);
      msg.right_arm[i] = map_from(mapped.right_arm[i]);
    }
    msg.base = mapped.base;
    return msg;
  }
};
#pragma pack(pop)
static_assert(sizeof(TOK2Message) == 60, "Unexpected TOK2Message size");
static_assert(sizeof(TOK2Message::Mapped) == 32, "Unexpected TOK2Message::Mapped size");

int main() {
  std::string serial_receiver = "/dev/ttyUSB0";

  auto executor = airbot::hardware::AsioExecutor::create(8);

  auto serial_a = airbot::hardware::CommHandler<Frame>::create("tok2_receiver", "serial", serial_receiver, freq,
                                                               executor->get_io_context());

  serial_a->add_read_callback(
      "test", [&](const Frame& frame) { std::cerr << TOK2Message::deserialize(frame).format() << std::endl; });

  serial_a->start();

  auto now = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - now < std::chrono::seconds(120)) {
    std::this_thread::sleep_for(std::chrono::seconds(100));
  }

  serial_a->stop().get();

  return 0;
}
