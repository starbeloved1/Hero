#include "gimbal_driver/serial_port.hpp"

#include <algorithm>
#include <sstream>
#include <thread>
#include <utility>

namespace gimbal_driver
{

using boost::asio::buffer;
using boost::asio::serial_port_base;

SerialPort::SerialPort()
: serial_port_(io_context_)
{
}

SerialPort::~SerialPort()
{
  stop();
}

//主程序入口
bool SerialPort::start(
  const std::string & port_name, int baud_rate, bool verify_crc, bool allow_virtual_serial)
{
  verify_crc_ = verify_crc;
  std::string error_message;
  // 打开串口失败时，只允许离线测试使用虚拟串口继续启动。
  if (!open(port_name, baud_rate, error_message)) {
    virtual_serial_ = allow_virtual_serial;
    if (!virtual_serial_) {
      reportError(error_message);
    }
    return virtual_serial_;
  }

  running_.store(true);
  work_guard_ = std::make_unique<boost::asio::io_context::work>(io_context_);
  startRead();
  io_thread_ = std::thread([this]() {io_context_.run();});
  return true;
}

void SerialPort::stop()
{
  running_.store(false);
  work_guard_.reset();
  boost::system::error_code error;
  serial_port_.cancel(error);
  serial_port_.close(error);
  io_context_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

bool SerialPort::write(const LegacyWriteCommand & command)
{
  if (virtual_serial_) {
    return true;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (!serial_port_.is_open()) {
    reportError("串口未打开，无法发送云台控制命令");
    return false;
  }
  const auto bytes = encodeWriteCommand(command);
  boost::system::error_code error;
  boost::asio::write(serial_port_, buffer(bytes), error);
  if (error) {
    reportError("串口写入云台控制命令失败：" + error.message());
  }
  return !error;
}

void SerialPort::setReadCallback(ReadCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  read_callback_ = std::move(callback);
}

void SerialPort::setErrorCallback(ErrorCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  error_callback_ = std::move(callback);
}

bool SerialPort::open(
  const std::string & port_name, int baud_rate, std::string & error_message)
{
  boost::system::error_code error;
  serial_port_.open(port_name, error);
  if (error) {
    error_message = "无法打开串口 '" + port_name + "'：" + error.message();
    return false;
  }
  serial_port_.set_option(serial_port_base::baud_rate(baud_rate), error);
  if (error) {
    error_message = "无法设置串口波特率：" + error.message();
  }
  serial_port_.set_option(serial_port_base::character_size(8), error);
  if (error && error_message.empty()) {
    error_message = "无法设置串口数据位：" + error.message();
  }
  serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none), error);
  if (error && error_message.empty()) {
    error_message = "无法设置串口校验位：" + error.message();
  }
  serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one), error);
  if (error && error_message.empty()) {
    error_message = "无法设置串口停止位：" + error.message();
  }
  serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none), error);
  if (error) {
    if (error_message.empty()) {
      error_message = "无法设置串口流控：" + error.message();
    }
    serial_port_.close();
    return false;
  }
  return true;
}

void SerialPort::startRead()
{
  serial_port_.async_read_some(
    buffer(read_buffer_), [this](const boost::system::error_code & error, std::size_t size) {
      if (!error && running_.load()) {
        consume(read_buffer_.data(), size);
        startRead();
      } else if (error && running_.load()) {
        reportError("串口读取失败：" + error.message());
      }
    });
}

void SerialPort::reportError(const std::string & message)
{
  ErrorCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = error_callback_;
  }
  if (callback) {
    callback(message);
  }
}

void SerialPort::consume(const uint8_t * data, std::size_t size)
{
  pending_bytes_.insert(pending_bytes_.end(), data, data + size);
  while (pending_bytes_.size() >= kReadFrameSize) {
    const auto start = std::find(pending_bytes_.begin(), pending_bytes_.end(), kFrameStart);
    if (start == pending_bytes_.end()) {
      pending_bytes_.clear();
      return;
    }
    pending_bytes_.erase(pending_bytes_.begin(), start);
    if (pending_bytes_.size() < kReadFrameSize) {
      return;
    }
    std::array<uint8_t, kReadFrameSize> bytes{};
    std::copy_n(pending_bytes_.begin(), kReadFrameSize, bytes.begin());
    const auto frame = decodeReadFrame(bytes, verify_crc_);
    if (frame.has_value()) {
      ReadCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = read_callback_;
      }
      if (callback) {
        callback(*frame);
      }
      pending_bytes_.erase(pending_bytes_.begin(), pending_bytes_.begin() + kReadFrameSize);
    } else {
      pending_bytes_.erase(pending_bytes_.begin());
    }
  }
}

}  // gimbal_driver
