#ifndef AIRBOT_HARDWARE_EXECUTOR_HPP
#define AIRBOT_HARDWARE_EXECUTOR_HPP

#include <memory>

#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

/**
 * @brief Stub executor that does nothing.
 *
 * This Stub struct is to enable later use of std::shared_ptr<Stub>.
 * A custom type instead of std::any or std::shared_ptr<void> is used
 * because pybind11 does not support those types.
 *
 * @note This is a stub executor that does nothing. It is used to
 *       enable later use of std::shared_ptr<Stub>.
 */
struct AB_API Stub {
  void* a;
};
using ExecutorPtr = std::shared_ptr<Stub>;

/**
 * @brief Executor class that manages multiple worker threads.
 *
 * This executor is designed to try to run in a real-time environment, ensuring that the io_context
 * runs with high priority and memory locking for performance.
 */
class AB_API AsioExecutor {
 protected:
  /**
   * @brief Hidden constructor to prevent direct instantiation.
   */
  AsioExecutor() = default;

 public:
  /**
   * @brief Factory method to create an AsioExecutor with the specified number of threads.
   *
   * Initializes the io_context and starts worker threads. Will try to configure
   * real-time scheduling and memory locking for performance.
   */
  static std::unique_ptr<AsioExecutor> create(uint16_t thread_num) noexcept;

  /**
   * @brief Destructor for AsioExecutor.
   * Stops the io_context and joins all worker threads.
   * This ensures that all resources are cleaned up properly.
   */
  virtual ~AsioExecutor() = default;

  /**
   * @brief Returns the shared pointer to the io_context managed by this executor.
   *
   * @return ExecutorPtr Shared pointer to the io_context.
   */
  [[nodiscard]]
  virtual ExecutorPtr get_io_context() const noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_EXECUTOR_HPP
