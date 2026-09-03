#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include "gimbal_driver/serial_protocol.hpp"

namespace gimbal_driver
{

class SerialPort
{
public:
  using ReadCallback = std::function<void(const LegacyReadFrame &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  SerialPort();
  ~SerialPort();

  bool start(const std::string & port_name, int baud_rate, bool verify_crc);
  void stop();
  bool write(const LegacyWriteCommand & command);
  bool isVirtual() const;
  void setReadCallback(ReadCallback callback);
  void setErrorCallback(ErrorCallback callback);

private:
  bool open(const std::string & port_name, int baud_rate, std::string & error_message);
  void startRead();
  void consume(const uint8_t * data, std::size_t size);
  void reportError(const std::string & message);

  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::unique_ptr<boost::asio::io_context::work> work_guard_;
  std::thread io_thread_;
  std::array<uint8_t, 256U> read_buffer_{};
  std::vector<uint8_t> pending_bytes_;
  std::atomic<bool> running_{false};
  bool verify_crc_{true};
  bool virtual_serial_{false};
  std::mutex callback_mutex_;
  std::mutex write_mutex_;
  ReadCallback read_callback_;
  ErrorCallback error_callback_;
};

}  // gimbal_driver
