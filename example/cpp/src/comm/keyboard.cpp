#include <linux/input.h>

#include <airbot_hardware/comm/comm.hpp>
#include <airbot_hardware/executors/executor.hpp>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace airbot::hardware;

static std::string format(const struct input_event& ev, const std::string& prefix = "") {
  std::ostringstream oss;
  auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  oss << prefix << " ";
  oss << "[" << std::setw(3) << std::setfill('0') << timestamp.count() << "] ";
  oss << "type: " << ev.type << ", code: " << ev.code << ", value: " << ev.value;
  return oss.str();
}

int main() {
  std::string keyboard_path = "/dev/input/by-id/usb-Parallels_Virtual_Keyboard_KBD1.1-if01-event-kbd";

  auto executor = AsioExecutor::create(2);

  auto keyboard_interface = CommHandler<struct input_event>::create("test_keyboard", "keyboard", keyboard_path, 1000,
                                                                    executor->get_io_context());

  keyboard_interface->start();

  keyboard_interface->add_read_callback("test", [&](const struct input_event& ev) {
    std::cerr << format(ev, "recv ") << std::endl;
    if (ev.type == EV_KEY && ev.code == KEY_Q && ev.value == 1) {
      std::cerr << "Quit command received, stopping..." << std::endl;
      exit(0);
    }
  });

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    // std::cerr << format(received, "recv ") << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
