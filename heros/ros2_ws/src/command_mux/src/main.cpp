#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "command_mux/command_mux_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<command_mux::CommandMuxNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("command_mux_node"), "节点初始化失败：%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
