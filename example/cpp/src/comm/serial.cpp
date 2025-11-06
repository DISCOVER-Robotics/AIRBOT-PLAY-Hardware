#include <airbot_hardware/comm/comm.hpp>
#include <airbot_hardware/executors/executor.hpp>
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

static constexpr size_t frame_size = 64;  // Define the frame size as needed
static constexpr size_t freq = 250;       // Define the frame size as needed

using Frame = std::array<uint8_t, frame_size>;

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);
  auto executor = AsioExecutor::create(2);

  auto serial_a =
      CommHandler<Frame>::create("test_serial_a", "serial", "/dev/ttyUSB0", freq, executor->get_io_context());
  auto serial_b =
      CommHandler<Frame>::create("test_serial_b", "serial", "/dev/ttyUSB1", freq, executor->get_io_context());

  serial_a->start();
  serial_b->start();

  serial_a->add_read_callback("test_serial", [&](const Frame& frame) {
    std::cout << format(frame, "a_recv ") << std::endl;
    // auto f = std::make_shared<Frame>(frame);
    // f->at(0) += 1;
    // serial_a->write_weak(std::move(f));
  });
  serial_b->add_read_callback("test_serial", [&](const Frame& frame) {
    std::cout << format(frame, "b_recv ") << std::endl;
    // auto f = std::make_shared<Frame>(frame);
    // f->at(0) += 1;
    // serial_b->write_weak(std::move(f));
  });

  std::thread([&]() {
    uint8_t a = 0;
    while (serial_a) {
      a++;
      auto frame = std::make_shared<Frame>();
      frame->fill(a);
      std::cout << format(*frame, "send:: ") << std::endl;
      serial_a->write_weak(std::move(frame));
      std::this_thread::sleep_for(std::chrono::microseconds(1'000'000 / freq));
    }
  }).detach();
  // serial_a->write_weak(std::make_shared<Frame>(Frame{0x00}));

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    // std::cerr << format(received, "recv ") << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  serial_a->stop().get();
  serial_b->stop().get();
  serial_a.reset();
  serial_b.reset();
  std::cout << "Serial communication stopped." << std::endl;

  return 0;
}
