#include "camera_router/camera_router_node.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace camera_router
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

void validateTopic(const std::string & topic_name, const char * parameter_name)
{
  if (topic_name.empty()) {
    throw std::invalid_argument(std::string(parameter_name) + " 不能为空");
  }
}

}  // namespace

CameraRouterNode::CameraRouterNode(): Node("camera_router_node")
{
  //节点参数
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<bool>("base_camera_enabled", true);
  declare_parameter<std::string>("aim8mm_image_topic", "/hero/camera/aim8mm/image_raw");
  declare_parameter<std::string>("aim8mm_camera_info_topic", "/hero/camera/aim8mm/camera_info");
  declare_parameter<std::string>("base_image_topic", "/hero/camera/base/image_raw");
  declare_parameter<std::string>("base_camera_info_topic", "/hero/camera/base/camera_info");
  declare_parameter<std::string>("selected_image_topic", "/hero/camera/selected/image_raw");
  declare_parameter<std::string>("selected_camera_info_topic", "/hero/camera/selected/camera_info");
  declare_parameter<std::string>("selected_frame_id", "camera_optical_frame");

  const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
  const auto aim8mm_image_topic = get_parameter("aim8mm_image_topic").as_string();
  const auto aim8mm_camera_info_topic = get_parameter("aim8mm_camera_info_topic").as_string();
  const auto base_image_topic = get_parameter("base_image_topic").as_string();
  const auto base_camera_info_topic = get_parameter("base_camera_info_topic").as_string();
  const auto selected_image_topic = get_parameter("selected_image_topic").as_string();
  const auto selected_camera_info_topic = get_parameter("selected_camera_info_topic").as_string();
  selected_frame_id_ = get_parameter("selected_frame_id").as_string();
  validateTopic(gimbal_state_topic, "gimbal_state_topic");
  validateTopic(aim8mm_image_topic, "aim8mm_image_topic");
  validateTopic(aim8mm_camera_info_topic, "aim8mm_camera_info_topic");
  validateTopic(base_image_topic, "base_image_topic");
  validateTopic(base_camera_info_topic, "base_camera_info_topic");
  validateTopic(selected_image_topic, "selected_image_topic");
  validateTopic(selected_camera_info_topic, "selected_camera_info_topic");
  validateTopic(selected_frame_id_, "selected_frame_id");
  base_camera_enabled_ = get_parameter("base_camera_enabled").as_bool();
  
  const auto sensor_qos = highRateQos();

  //发布image与camera_info
  selected_image_pub_ = create_publisher<sensor_msgs::msg::Image>(selected_image_topic, sensor_qos);
  selected_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(selected_camera_info_topic, sensor_qos);

  //订阅云台状态以及各相机
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    gimbal_state_topic, highRateQos(),
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {receiveGimbalState(*message);});
  aim8mm_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    aim8mm_image_topic, sensor_qos,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
      forwardImage(CameraProfile::kAim8mm, *message);
    });
  aim8mm_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    aim8mm_camera_info_topic, sensor_qos,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      forwardCameraInfo(CameraProfile::kAim8mm, *message);
    });
  base_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    base_image_topic, sensor_qos,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
      forwardImage(CameraProfile::kBase, *message);
    });
  base_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    base_camera_info_topic, sensor_qos,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      forwardCameraInfo(CameraProfile::kBase, *message);
    });

  RCLCPP_INFO(
    get_logger(), "相机路由节点已启动：默认选中 %s，基地相机%s", profileName(active_profile_),
    base_camera_enabled_ ? "已启用" : "未启用");
}

//云台订阅回调
void CameraRouterNode::receiveGimbalState(const hero_msgs::msg::GimbalState & state)
{
  const auto new_profile = selectProfile(state.mode, base_camera_enabled_);
  if (new_profile == active_profile_) {
    return;
  }

  active_profile_ = new_profile;
  RCLCPP_INFO(
    get_logger(), "云台模式 %u，切换到 %s 相机输入", static_cast<unsigned int>(state.mode),
    profileName(active_profile_));
}

//相机image与info回调
void CameraRouterNode::forwardImage(
  CameraProfile source_profile, const sensor_msgs::msg::Image & image)
{
  if (source_profile != active_profile_) {
    return;
  }
  auto selected_image = image;
  // selected 是供后续算法使用的逻辑相机接口，统一使用当前选中相机坐标系名称
  selected_image.header.frame_id = selected_frame_id_;
  selected_image_pub_->publish(std::move(selected_image));
}

void CameraRouterNode::forwardCameraInfo(
  CameraProfile source_profile, const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (source_profile != active_profile_) {
    return;
  }
  auto selected_camera_info = camera_info;
  // 必须与 selected 图像使用同一 frame_id，时间戳保持物理相机采集时刻不变
  selected_camera_info.header.frame_id = selected_frame_id_;
  selected_camera_info_pub_->publish(std::move(selected_camera_info));
}

}  // camera_router
