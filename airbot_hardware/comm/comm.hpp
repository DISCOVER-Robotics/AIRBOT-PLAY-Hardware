#ifndef AIRBOT_HARDWARE_COMM_COMM_HPP
#define AIRBOT_HARDWARE_COMM_COMM_HPP

#include <future>
#include <memory>
#include <string>

#include "airbot_hardware/executors/executor.hpp"
#include "airbot_hardware/utils.hpp"

namespace airbot {
namespace hardware {

/**
 * @brief Abstract base class for communication (IO) handlers.
 *
 * Communication handlers are designed to provide a unified IO interface
 * for packet/frame based IO. Each implementation of IO handler is able
 * to asynchronously send data frames (write) and call the registered callbacks
 * when data frames are received (read).
 *
 * The design choice of using callback-based IO (not awaitable or other forms)
 * is to be compatible with synchronous programs. The actual callers of the callbacks
 * are the workers managed by airbot::hardware::AsioExecutor, which now provide
 * thread-safety which is **NOT** guaranteed and may be loosened in the future.
 *
 * Notice for developers: to implement a new IO channel under this interface,
 * framing is necessary (unless you would want to use raw bytes).
 *
 * @note The implementation of this class is not thread-safe.
 */
template <typename frame_type>
class AB_API CommHandler {
 public:
  using FramePtr = std::shared_ptr<frame_type>;
  using CallbackType = std::function<void(const frame_type&)>;

  /**
   * @brief Factory method to create a CommHandler instance.
   *
   * Implemented handlers:
   * - `CommHandler<struct can_frame>`: create with type `socketcan`,
   *    use interface to specify the CAN interface, e.g. `can0`.
   * - `CommHandler<struct input_event>`: create with type `keyboard`,
   *    use interface to specify the keyboard path, e.g. `/dev/input/event0`.
   * - `CommHandler<char>`: create with type `tty`, use `/dev/tty` as interface. Interface arg is ignored.
   * - `CommHandler<std::array<uint8_t, 8>>`: create with type `serial`, use interface to specify the serial port path,
   *    e.g. `/dev/ttyUSB0`.
   * - `CommHandler<std::array<uint8_t, 16>>`: create with type `serial`, use interface to specify the serial port path,
   *    e.g. `/dev/ttyUSB0`.
   * - `CommHandler<std::array<uint8_t, 32>>`: create with type `serial`, use interface to specify the serial port path,
   *    e.g. `/dev/ttyUSB0`.
   * - `CommHandler<std::array<uint8_t, 64>>`: create with type `serial`, use interface to specify the serial port path,
   *    e.g. `/dev/ttyUSB0`.
   * - `CommHandler<std::array<uint8_t, 128>>`: create with type `serial`, use interface to specify the serial port
   *    path, e.g. `/dev/ttyUSB0`.
   *
   * @param name The name of the handler instance. Used to identify instance in logs.
   * @param type The type of the handler. See specification above.
   * @param interface The interface to use for the handler. See specification above.
   * @param spin_freq The maximum frequency for this handler to try to read / write frames.
   *                  This spin frequency imposes a minimum delay between adjacent writes.
                      Currently only writing rate is limited by this spin frequency.
   * @param io_context The executor that manages workers to actually perform IO operations.
   * @return A unique_ptr to the created CommHandler instance, or nullptr if creation fails.
   */
  static std::unique_ptr<CommHandler> create(std::string name, std::string type, std::string interface,
                                             uint16_t spin_freq, ExecutorPtr io_context) noexcept;
  /**
   * @brief Virtual destructor for CommHandler. Cancel all pending operations and clean up resources.
   * @note `stop()` is called and waited under the hood. Explicitly calling `stop()` is not required.
   */
  virtual ~CommHandler() = default;

  /**
   * @brief Initialize the communication handler.
   * @param name The name of the handler instance. Used to identify instance in logs.
   * @param interface The interface to use for the handler. See specification above.
   * @param spin_freq The maximum frequency for this handler to try to read / write frames.
   *                  This spin frequency imposes a minimum delay between adjacent writes.
   *                  Currently only writing rate is limited by this spin frequency.
   * @param io_context The executor that manages workers to actually perform IO operations.
   * @return true if the handler initialized successfully, false otherwise.
   * @note This method is called by the factory method `create()` and kept in public API for
   *       the sake of backward compatibility.
   */
  virtual bool init(std::string name, std::string interface, uint16_t spin_freq, ExecutorPtr io_context) noexcept = 0;

  /**
   * @brief Start receiving and processing incoming messages.
   * @return true if the handler started successfully, false otherwise.
   */
  [[nodiscard]]
  virtual bool start() noexcept = 0;

  /**
   * @brief Stop the communication handler. Pending incoming and outgoing operations will be cancelled.
   * @return A future that resolves to true if the handler stopped successfully, false otherwise.
   * @note This method is non-blocking and will return immediately. The actual stop operation
   *       will be performed asynchronously.
   */
  [[nodiscard]]
  virtual std::future<bool> stop() noexcept = 0;

  /**
   * @brief Write a data frame to the communication channel. Alias for `write_weak()`.
   * @param frame The data frame to write. This should be a shared pointer to a frame_type structure.
   *              The ownership of the frame is transferred to the handler.
   * @return See `write_weak()` for return value semantics.
   * @note This method is non-blocking and will return immediately.
   */
  virtual bool write(FramePtr&& frame) noexcept = 0;

  /**
   * @brief Write a data frame to the communication channel with weak semantics.
   *
   * Weak semantics should be used when the application can tolerate occasional
   * failures to write frames, such as in high-frequency control loops.
   *
   * WEAK semantics: This method will attempt to write the frame, but may fail in case of:
   *
   * - Invalid frame: the frame is null or does not contain valid data.
   * - Rate limiting: the last write attempt is too recent (less than 1 / spin_freq seconds ago).
   * - Pending writes: there is already a pending write attempt.
   *
   * If writing attempt succeeds, the frame is swapped in the internal buffer to be
   * asynchronously sent later. If writing attempt fails, the frame is discarded.
   *
   * @param frame The data frame to write. This should be a shared pointer to a frame_type structure.
   *              The ownership of the frame is transferred to the handler.
   * @return true if the frame was successfully written, false if the handler is not ready
   *         to accept new frames (e.g., due to rate limiting).
   * @note This method is non-blocking and will return immediately.
   */
  virtual bool write_weak(FramePtr&& frame) noexcept = 0;

  /**
   * @brief Write a data frame to the communication channel with strong semantics.
   *
   * Strong semantics should be used when the application requires guaranteed delivery.
   *
   * STRONG semantics: This method will attempt to write the frame, and may only fail if:
   *
   * - Invalid frame: the frame is null or does not contain valid data.
   *
   * In case of:
   *
   * - Rate limiting (the last write attempt is too recent, less than 1 / spin_freq seconds ago),
   *   the method will spin-wait until the next allowed time.
   *
   * - Pending writes (a write attempt already pending), the method will spin until
   *   the previous writing frame is swapped out to be sent by workers, and then
   *   swap in the current attempt frame.
   */
  [[nodiscard]]
  virtual bool write_strong(FramePtr&& frame) noexcept = 0;

  /**
   * @brief Add a read callback for incoming data frames.
   * @param name The name of the callback, used to identify it.
   * @param cb The callback function to invoke when a frame is received.
   *           The callback should accept a const reference to a frame_type.
   * @note The invoking of callbacks is protected by a recursive mutex,
   *       so it is safe to call `add_read_callback` and `delete_read_callback`
   *       without external synchronization.
   * @bug The use of recursive mutex would only allow one worker thread to
   *      process incoming frames at a time, which may lead to performance issues.
   *      This is a known issue and will be addressed in the future.
   */
  virtual void add_read_callback(const std::string& name, CallbackType cb) noexcept = 0;

  /**
   * @brief Delete a previously registered read callback.
   * @param name The name of the callback to delete.
   */
  virtual void delete_read_callback(const std::string& name) noexcept = 0;
};

}  // namespace hardware
}  // namespace airbot

#endif  // AIRBOT_HARDWARE_COMM_COMM_HPP
