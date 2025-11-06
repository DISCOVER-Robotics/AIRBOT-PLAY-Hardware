#include <linux/input.h>

#include <airbot_hardware/comm/comm.hpp>
#include <airbot_hardware/executors/executor.hpp>
#include <airbot_hardware/handlers/motor.hpp>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

using namespace airbot::hardware;

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
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);

  std::string left_arm_interface = "can-usb2-lower";
  std::string right_arm_interface = "can-usb2-upper";
  std::string keyboard_path = "/dev/input/by-path/platform-xhci-hcd.2.auto-usb-0:1.3:1.0-event-kbd";  // USB 3.0 port
  std::string serial_sender = "/dev/ttyS3";

  auto executor = AsioExecutor::create(4);

  auto leftarm = std::array{
      Motor::create<MotorType::EC, 1>(), Motor::create<MotorType::EC, 2>(), Motor::create<MotorType::EC, 3>(),
      Motor::create<MotorType::EC, 4>(), Motor::create<MotorType::EC, 5>(), Motor::create<MotorType::EC, 6>(),
      Motor::create<MotorType::EC, 7>(),
  };

  for (auto&& i : leftarm) {
    i->init(executor->get_io_context(), left_arm_interface, 500_hz);
    i->enable();
  }

  auto rightarm = std::array{
      Motor::create<MotorType::EC, 1>(), Motor::create<MotorType::EC, 2>(), Motor::create<MotorType::EC, 3>(),
      Motor::create<MotorType::EC, 4>(), Motor::create<MotorType::EC, 5>(), Motor::create<MotorType::EC, 6>(),
      Motor::create<MotorType::EC, 7>(),
  };

  for (auto&& i : rightarm) {
    i->init(executor->get_io_context(), right_arm_interface, 500_hz);
    i->enable();
  }

  auto serial_a = CommHandler<Frame>::create("tok2_sender", "serial", serial_sender, 1000, executor->get_io_context());
  serial_a->start();

  std::atomic<uint32_t> keyboard_action = 0;

  auto keyboard = CommHandler<struct input_event>::create("tok2_keyboard", "serial", keyboard_path, 1000,
                                                          executor->get_io_context());
  keyboard->start();

  keyboard->add_read_callback("test", [&](const struct input_event& ev) {
    if (ev.type == EV_KEY && ev.code == KEY_1 && ev.value == 1) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current | 0x0800)) break;
      }
    } else if (ev.type == EV_KEY && ev.code == KEY_1 && ev.value == 0) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current & ~0x0800)) break;
      }
    } else if (ev.type == EV_KEY && ev.code == KEY_2 && ev.value == 1) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current | 0x0400)) break;
      }
    } else if (ev.type == EV_KEY && ev.code == KEY_2 && ev.value == 0) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current & ~0x0400)) break;
      }
    } else if (ev.type == EV_KEY && ev.code == KEY_3 && ev.value == 1) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current | 0x0200)) break;
      }
    }
    if (ev.type == EV_KEY && ev.code == KEY_3 && ev.value == 0) {
      while (true) {
        auto current = keyboard_action.load();
        if (std::atomic_compare_exchange_strong(&keyboard_action, &current, current & ~0x0200)) break;
      }
    }
  });

  auto message = TOK2Message{};

  try {
    while (true) {
      for (auto&& i : leftarm) {
        if (!i->ping()) {
          std::cerr << "Failed to send ping command to motor" << std::endl;
          break;
        }
      }
      for (auto&& i : rightarm) {
        if (!i->ping()) {
          std::cerr << "Failed to send ping command to motor" << std::endl;
          break;
        }
      }
      message.left_arm[0] = leftarm[0]->state().pos;
      message.left_arm[1] = leftarm[1]->state().pos;
      message.left_arm[2] = leftarm[2]->state().pos;
      message.left_arm[3] = leftarm[3]->state().pos;
      message.left_arm[4] = leftarm[4]->state().pos;
      message.left_arm[5] = leftarm[5]->state().pos;
      message.left_arm[6] = leftarm[6]->state().pos;

      message.right_arm[0] = rightarm[0]->state().pos;
      message.right_arm[1] = rightarm[1]->state().pos;
      message.right_arm[2] = rightarm[2]->state().pos;
      message.right_arm[3] = rightarm[3]->state().pos;
      message.right_arm[4] = rightarm[4]->state().pos;
      message.right_arm[5] = rightarm[5]->state().pos;
      message.right_arm[6] = rightarm[6]->state().pos;

      message.base = keyboard_action.load();

      auto frame = message.serialize();

      serial_a->write_weak(std::make_shared<Frame>(std::move(frame)));

      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  } catch (const std::exception& e) {
    serial_a->stop().get();
    keyboard->stop().get();
    for (auto&& i : leftarm) {
      i->disable();
      i->uninit();
    }
    for (auto&& i : rightarm) {
      i->disable();
      i->uninit();
    }
    return 0;
  }
  serial_a->stop().get();
  keyboard->stop().get();
  for (auto&& i : leftarm) {
    i->disable();
    i->uninit();
  }
  for (auto&& i : rightarm) {
    i->disable();
    i->uninit();
  }
  return 0;
}
