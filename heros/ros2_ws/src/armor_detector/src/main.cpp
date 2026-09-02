#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "armor_detector/armor_detector_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<armor_detector::DetectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("armor_detector_node"), "节点初始化失败：%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
