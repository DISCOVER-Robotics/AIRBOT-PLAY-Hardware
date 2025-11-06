#include <cmath>
#include <iostream>
#include <thread>

#include "airbot_hardware/comm/comm.hpp"
#include "airbot_hardware/executors/executor.hpp"

using namespace std::chrono_literals;
using namespace airbot::hardware;

int main() {
  auto executor = airbot::hardware::AsioExecutor::create(1);

  auto tty_comm = CommHandler<char>::create("tty_comm", "tty", "", 1000, executor->get_io_context());
  tty_comm->add_read_callback("test", [](const char& ev) { std::cout << "test: " << ev << std::endl; });
  tty_comm->start();

  auto current_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - current_time < 20s) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  tty_comm->delete_read_callback("test");
  return 0;
}
