#include <airbot_hardware/comm/comm.hpp>
#include <airbot_hardware/executors/executor.hpp>
#include <airbot_hardware_impl/comm/modbus_rtu.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

using namespace airbot::hardware;
using namespace std::chrono_literals;

// Helper function to format frame data for display
static std::string format_frame(const ModbusRtuFrame& frame, size_t length, const std::string& prefix = "") {
  std::ostringstream oss;
  auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
  oss << prefix << " [" << std::setw(6) << std::setfill('0') << timestamp.count() << "] ";
  for (size_t i = 0; i < length; ++i) {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame[i]) << " ";
  }
  return oss.str();
}

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  std::cout.tie(nullptr);

  // Create executor with 2 threads
  auto executor = AsioExecutor::create(2);

  // Create Modbus RTU handler
  // Note: Replace "/dev/ttyUSB0" with your actual serial device
  auto modbus_handler =
      CommHandler<ModbusRtuFrame>::create("modbus_rtu", "modbus_rtu", "/dev/ttyUSB0", 9600, executor->get_io_context());

  if (!modbus_handler) {
    std::cerr << "Failed to create Modbus RTU handler" << std::endl;
    return 1;
  }

  // Add callback to handle received data
  modbus_handler->add_read_callback("modbus_reader", [&](const ModbusRtuFrame& frame) {
    std::cout << format_frame(frame, 64, "RECV") << std::endl;

    // Parse the received frame
    auto register_values = ModbusRtuParser::parse(frame);
    if (!register_values.empty()) {
      std::cout << "Parsed register values:" << std::endl;
      for (const auto& [reg_index, value] : register_values) {
        std::cout << "  Register[" << reg_index << "] = " << value << " (0x" << std::hex << value << std::dec << ")"
                  << std::endl;
      }
    }
  });

  // Start the handler
  if (!modbus_handler->start()) {
    std::cerr << "Failed to start Modbus RTU handler" << std::endl;
    return 1;
  }

  std::cout << "Modbus RTU example started. Reading registers from slave device..." << std::endl;

  // Example 1: Read a single holding register
  std::thread read_thread([&]() {
    std::this_thread::sleep_for(1s);  // Wait for initialization

    while (modbus_handler) {
      // Read 1 register starting from address 0x0000 on slave ID 1
      auto read_frame = std::make_shared<ModbusRtuFrame>(ModbusRtuParser::generate_read_frame(1, 0x0191, 1));

      std::cout << format_frame(*read_frame, 8, "SEND") << std::endl;

      if (!modbus_handler->write_weak(std::move(read_frame))) {
        std::cout << "Failed to send read request" << std::endl;
      }

      std::this_thread::sleep_for(2s);  // Wait 2 seconds between requests
    }
  });

  // // Example 2: Write to a register after some time
  // std::thread write_thread([&]() {
  //   std::this_thread::sleep_for(5s); // Wait 5 seconds before writing

  //   // Write value 0x1234 to register 0x0001 on slave ID 1
  //   auto write_frame = std::make_shared<ModbusRtuFrame>(
  //       ModbusRtuParser::generate_write_frame(1, 0x0001, 0x1234));

  //   std::cout << format_frame(*write_frame, 8, "SEND WRITE") << std::endl;

  //   if (!modbus_handler->write_strong(std::move(write_frame))) {
  //     std::cout << "Failed to send write request" << std::endl;
  //   } else {
  //     std::cout << "Write request sent successfully" << std::endl;
  //   }
  // });

  // // Example 3: Write multiple registers
  // std::thread write_multiple_thread([&]() {
  //   std::this_thread::sleep_for(8s); // Wait 8 seconds before writing multiple

  //   // Write multiple values to registers starting from 0x0010
  //   std::vector<uint16_t> values = {0x1111, 0x2222, 0x3333};
  //   auto write_multi_frame = std::make_shared<ModbusRtuFrame>(
  //       ModbusRtuParser::generate_write_multiple_frame(1, 0x0010, 3, values));

  //   std::cout << format_frame(*write_multi_frame, 15, "SEND MULTI") << std::endl;

  //   if (!modbus_handler->write_strong(std::move(write_multi_frame))) {
  //     std::cout << "Failed to send write multiple request" << std::endl;
  //   } else {
  //     std::cout << "Write multiple request sent successfully" << std::endl;
  //   }
  // });

  // Run for 15 seconds
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 15s) {
    std::this_thread::sleep_for(100ms);
  }

  std::cout << "Stopping Modbus RTU communication..." << std::endl;

  // Clean shutdown
  modbus_handler->stop().get();
  modbus_handler->delete_read_callback("modbus_reader");
  modbus_handler.reset();

  // Wait for threads to finish
  if (read_thread.joinable()) read_thread.join();
  // if (write_thread.joinable()) write_thread.join();
  // if (write_multiple_thread.joinable()) write_multiple_thread.join();

  std::cout << "Modbus RTU example finished." << std::endl;
  return 0;
}
