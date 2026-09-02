#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "camera_router/camera_router_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<camera_router::CameraRouterNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_router_node"), "节点初始化失败：%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
