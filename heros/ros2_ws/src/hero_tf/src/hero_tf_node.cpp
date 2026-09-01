#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/qos.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "hero_msgs/msg/gimbal_state.hpp"
#include "hero_tf/transform_utils.hpp"

namespace hero_tf
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

void setRotation(geometry_msgs::msg::TransformStamped & transform, const tf2::Quaternion & quaternion)
{
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
}

}  // namespace

class HeroTfNode : public rclcpp::Node
{
public:
  HeroTfNode(): Node("hero_tf_node")
  {
    //tf发布
    dynamic_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(*this, tf2_ros::DynamicBroadcasterQoS());
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    //节点参数
    declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
    declare_parameter<std::string>("world_frame_id", "world");
    declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
    declare_parameter<std::string>("camera_frame_id", "camera_link");
    declare_parameter<std::string>("camera_optical_frame_id", "camera_optical_frame");
    declare_parameter<double>("camera_trans_x", 0.195);
    declare_parameter<double>("camera_trans_y", 0.0);
    declare_parameter<double>("camera_trans_z", 0.065);
    declare_parameter<double>("camera_yaw_deg", -0.8);
    declare_parameter<double>("camera_pitch_deg", -11.2);
    declare_parameter<double>("camera_roll_deg", 0.0);

    world_frame_id_ = get_parameter("world_frame_id").as_string();
    gimbal_frame_id_ = get_parameter("gimbal_frame_id").as_string();
    camera_frame_id_ = get_parameter("camera_frame_id").as_string();
    camera_optical_frame_id_ = get_parameter("camera_optical_frame_id").as_string();
    if (
      world_frame_id_.empty() || gimbal_frame_id_.empty() || camera_frame_id_.empty() ||
      camera_optical_frame_id_.empty())
    {
      throw std::invalid_argument("TF 坐标系名称不能为空");
    }
    camera_trans_x_ = get_parameter("camera_trans_x").as_double();
    camera_trans_y_ = get_parameter("camera_trans_y").as_double();
    camera_trans_z_ = get_parameter("camera_trans_z").as_double();
    camera_yaw_deg_ = get_parameter("camera_yaw_deg").as_double();
    camera_pitch_deg_ = get_parameter("camera_pitch_deg").as_double();
    camera_roll_deg_ = get_parameter("camera_roll_deg").as_double();

    //订阅gimbal_state
    gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
      get_parameter("gimbal_state_topic").as_string(), highRateQos(),
      [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {publishDynamicTransform(*message);});

    //发布静态tf
    publishStaticTransforms();
    RCLCPP_INFO(
      get_logger(), "TF 节点已启动：%s -> %s -> %s -> %s", world_frame_id_.c_str(),
      gimbal_frame_id_.c_str(), camera_frame_id_.c_str(), camera_optical_frame_id_.c_str());
  }

private:
  //订阅回调，动态发布world->gimbal_link
  void publishDynamicTransform(const hero_msgs::msg::GimbalState & state)
  {
    if (!std::isfinite(state.yaw) || !std::isfinite(state.pitch)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "忽略包含非有限角度的云台状态");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = state.header.stamp;
    transform.header.frame_id = world_frame_id_;
    transform.child_frame_id = gimbal_frame_id_;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, -state.pitch, state.yaw);
    setRotation(transform, quaternion);
    dynamic_broadcaster_->sendTransform(transform);
  }

  //静态发布gimbal_link->camera_link&camera_link->camera_optical
  void publishStaticTransforms()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(2U);
    const auto stamp = now();

    geometry_msgs::msg::TransformStamped camera_transform;
    camera_transform.header.stamp = stamp;
    camera_transform.header.frame_id = gimbal_frame_id_;
    camera_transform.child_frame_id = camera_frame_id_;
    camera_transform.transform.translation.x = camera_trans_x_;
    camera_transform.transform.translation.y = camera_trans_y_;
    camera_transform.transform.translation.z = camera_trans_z_;
    setRotation(
      camera_transform,
      makeCamera2Gimbal(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_));
    transforms.push_back(camera_transform);

    geometry_msgs::msg::TransformStamped optical_transform;
    optical_transform.header.stamp = stamp;
    optical_transform.header.frame_id = camera_frame_id_;
    optical_transform.child_frame_id = camera_optical_frame_id_;
    setRotation(optical_transform, makeFLU2RDF());
    transforms.push_back(optical_transform);

    static_broadcaster_->sendTransform(transforms);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  std::string world_frame_id_;
  std::string gimbal_frame_id_;
  std::string camera_frame_id_;
  std::string camera_optical_frame_id_;
  double camera_trans_x_{0.0};
  double camera_trans_y_{0.0};
  double camera_trans_z_{0.0};
  double camera_yaw_deg_{0.0};
  double camera_pitch_deg_{0.0};
  double camera_roll_deg_{0.0};
};

}  // hero_tf

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<hero_tf::HeroTfNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("hero_tf_node"), "节点初始化失败：%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
