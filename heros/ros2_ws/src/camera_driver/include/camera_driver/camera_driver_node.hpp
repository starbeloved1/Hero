#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace camera_driver
{

class CameraDriverNode : public rclcpp::Node
{
public:
  CameraDriverNode();
  ~CameraDriverNode() override;

private:
  struct Stream;
  struct Frame;

  void captureLoop(Stream & stream);
  bool readFrame(Stream & stream, Frame & frame, std::string & error);
  bool readVideoFrame(Stream & stream, Frame & frame, std::string & error);

  std::atomic<bool> running_{false};
  std::vector<std::unique_ptr<Stream>> streams_;
};

}  // camera_driver
