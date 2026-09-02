#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_driver/gimbal_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<gimbal_driver::GimbalDriverNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("gimbal_driver_node"), "节点初始化失败：%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
