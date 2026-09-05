#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aim_predictor/aim_predictor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<aim_predictor::AimPredictorNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("aim_predictor_node"), "节点初始化失败：%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
