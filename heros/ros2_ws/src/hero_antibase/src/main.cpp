#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hero_antibase/hero_antibase_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hero_antibase::HeroAntiBaseNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("hero_antibase_node"), "节点初始化失败：%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
