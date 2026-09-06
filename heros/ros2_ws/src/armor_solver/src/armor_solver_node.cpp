#include "armor_solver/armor_solver_node.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "hero_msgs/msg/armor_pose.hpp"

namespace armor_solver
{

namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

geometry_msgs::msg::Quaternion quaternionFromRvec(const cv::Vec3d & rvec)
{
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  const double trace = rotation.at<double>(0, 0) + rotation.at<double>(1, 1) + rotation.at<double>(2, 2);
  geometry_msgs::msg::Quaternion quaternion;
  if (trace > 0.0) {
    const auto scale = 2.0 * std::sqrt(trace + 1.0);
    quaternion.w = 0.25 * scale;
    quaternion.x = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
    quaternion.y = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
    quaternion.z = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
    return quaternion;
  }

  // 本工程装甲板姿态通常接近正面；该分支处理接近 180 度旋转的退化情况
  const auto diagonal = std::array<double, 3>{
    rotation.at<double>(0, 0), rotation.at<double>(1, 1), rotation.at<double>(2, 2)};
  const auto axis = diagonal[0] > diagonal[1] && diagonal[0] > diagonal[2] ? 0 :
    (diagonal[1] > diagonal[2] ? 1 : 2);
  if (axis == 0) {
    const auto scale = 2.0 * std::sqrt(1.0 + diagonal[0] - diagonal[1] - diagonal[2]);
    quaternion.x = 0.25 * scale;
    quaternion.y = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.z = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
    quaternion.w = (rotation.at<double>(2, 1) - rotation.at<double>(1, 2)) / scale;
  } else if (axis == 1) {
    const auto scale = 2.0 * std::sqrt(1.0 + diagonal[1] - diagonal[0] - diagonal[2]);
    quaternion.x = (rotation.at<double>(0, 1) + rotation.at<double>(1, 0)) / scale;
    quaternion.y = 0.25 * scale;
    quaternion.z = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
    quaternion.w = (rotation.at<double>(0, 2) - rotation.at<double>(2, 0)) / scale;
  } else {
    const auto scale = 2.0 * std::sqrt(1.0 + diagonal[2] - diagonal[0] - diagonal[1]);
    quaternion.x = (rotation.at<double>(0, 2) + rotation.at<double>(2, 0)) / scale;
    quaternion.y = (rotation.at<double>(1, 2) + rotation.at<double>(2, 1)) / scale;
    quaternion.z = 0.25 * scale;
    quaternion.w = (rotation.at<double>(1, 0) - rotation.at<double>(0, 1)) / scale;
  }
  return quaternion;
}

geometry_msgs::msg::Pose poseFromEstimate(const ArmorPoseEstimate & estimate)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = estimate.tvec[0];
  pose.position.y = estimate.tvec[1];
  pose.position.z = estimate.tvec[2];
  pose.orientation = quaternionFromRvec(estimate.rvec);
  return pose;
}

visualization_msgs::msg::Marker makeMarker(
  const hero_msgs::msg::ArmorPose & armor, int marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header = armor.header;
  marker.ns = "armor_pose";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = armor.pose;
  marker.scale.x = armor.is_large ? 0.23 : 0.135;
  marker.scale.y = 0.056;
  marker.scale.z = 0.01;
  marker.color.r = armor.color == 1U ? 1.0F : 0.0F;
  marker.color.g = 0.2F;
  marker.color.b = armor.color == 0U ? 1.0F : 0.0F;
  marker.color.a = 0.85F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  return marker;
}

}  // namespace

ArmorSolverNode::ArmorSolverNode(): Node("armor_solver_node")
{
  declare_parameter<std::string>("armor_topic", "/hero/detector/armors");
  declare_parameter<std::string>("camera_info_topic", "/hero/camera/selected/camera_info");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("armor_pose_topic", "/hero/solver/armor_poses");
  declare_parameter<std::string>("target_frame_id", "world");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  declare_parameter<bool>("visualization_enabled", true);
  declare_parameter<std::string>("visualization_topic", "/hero/solver/markers");

  target_frame_id_ = get_parameter("target_frame_id").as_string();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  if (tf_lookup_timeout_sec_ < 0.0) {
    throw std::invalid_argument("tf_lookup_timeout_sec 不能小于 0");
  }

  const auto armor_topic = get_parameter("armor_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
  const auto armor_pose_topic = get_parameter("armor_pose_topic").as_string();
  if (armor_topic.empty() || camera_info_topic.empty() ||
      gimbal_state_topic.empty() || armor_pose_topic.empty()) {
    throw std::invalid_argument("装甲板、相机内参、云台状态和位姿话题不能为空");
  }

  const auto qos = highRateQos();
  pose_pub_ = create_publisher<hero_msgs::msg::ArmorPoseArray>(armor_pose_topic, qos);
  armor_sub_ = create_subscription<hero_msgs::msg::ArmorArray>(
    armor_topic, qos,
    [this](const hero_msgs::msg::ArmorArray::ConstSharedPtr message) {receiveArmors(message);});
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, qos,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {receiveCameraInfo(message);});
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
    gimbal_state_topic, qos,
    [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {
      receiveGimbalState(message);
    });
  if (get_parameter("visualization_enabled").as_bool()) {
    const auto visualization_topic = get_parameter("visualization_topic").as_string();
    if (visualization_topic.empty()) {
      throw std::invalid_argument("visualization_topic 不能为空");
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, qos);
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true, qos, tf2_ros::StaticListenerQoS());
  RCLCPP_INFO(
    get_logger(), "位姿解算节点已启动：输入 %s，输出 %s，目标坐标系 %s", armor_topic.c_str(),
    armor_pose_topic.c_str(), target_frame_id_.empty() ? "输入相机坐标系" : target_frame_id_.c_str());
}

void ArmorSolverNode::receiveCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
{
  // mode4 的 selected 来自基地相机，只供 hero_antibase 编码；不能让其
  // CameraInfo 参与装甲板 PnP，也不应对尚未标定的基地内参重复报警
  if (gimbal_mode_.load() == hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
    return;
  }
  CameraIntrinsics intrinsics;
  for (std::size_t index = 0; index < intrinsics.matrix.size(); ++index) {
    intrinsics.matrix[index] = message->k[index];
  }
  intrinsics.distortion.assign(message->d.begin(), message->d.end());
  if (intrinsics.matrix[0] <= 0.0 || intrinsics.matrix[4] <= 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "忽略无效的 CameraInfo 内参");
    return;
  }
  std::lock_guard<std::mutex> lock(intrinsics_mutex_);
  intrinsics_ = std::move(intrinsics);
  has_intrinsics_ = true;
}

void ArmorSolverNode::receiveGimbalState(
  const hero_msgs::msg::GimbalState::ConstSharedPtr & message)
{
  gimbal_mode_.store(message->mode);
}

void ArmorSolverNode::receiveArmors(const hero_msgs::msg::ArmorArray::ConstSharedPtr & message)
{
  if (gimbal_mode_.load() == hero_msgs::msg::GimbalState::MODE_ANTI_BASE) {
    return;
  }
  CameraIntrinsics intrinsics;
  {
    std::lock_guard<std::mutex> lock(intrinsics_mutex_);
    if (!has_intrinsics_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "尚未收到有效 CameraInfo，无法进行 PnP 解算");
      return;
    }
    intrinsics = intrinsics_;
  }

  hero_msgs::msg::ArmorPoseArray output;
  output.header = message->header;
  output.armors.reserve(message->armors.size());
  geometry_msgs::msg::TransformStamped transform;
  bool transform_available = false;
  const bool should_transform = !target_frame_id_.empty() && message->header.frame_id != target_frame_id_;
  if (should_transform) {
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_id_, message->header.frame_id, rclcpp::Time(message->header.stamp),
        tf2::durationFromSec(tf_lookup_timeout_sec_));
      transform_available = true;
      output.header.frame_id = target_frame_id_;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "未找到 %s 到 %s 的同刻 TF，暂以相机坐标系发布：%s",
        message->header.frame_id.c_str(), target_frame_id_.c_str(), error.what());
    }
  }

  for (const auto & armor : message->armors) {
    std::array<cv::Point2f, 4> corners;
    for (std::size_t index = 0; index < corners.size(); ++index) {
      corners[index] = cv::Point2f(armor.corners[index].x, armor.corners[index].y);
    }
    const auto size = armorSizeForId(armor.id);
    const auto estimate = solveArmorPnP(corners, size, intrinsics);
    if (!estimate.success) {
      continue;
    }

    hero_msgs::msg::ArmorPose pose;
    pose.header = message->header;
    pose.pose = poseFromEstimate(estimate);
    for (const auto & corner : armor.corners) {
      pose.image_center.x += corner.x;
      pose.image_center.y += corner.y;
    }
    pose.image_center.x /= static_cast<float>(armor.corners.size());
    pose.image_center.y /= static_cast<float>(armor.corners.size());
    pose.id = armor.id;
    pose.color = armor.color;
    pose.confidence = armor.confidence;
    pose.is_large = size == ArmorSize::kLarge;
    pose.reprojection_error = static_cast<float>(estimate.reprojection_error);
    if (transform_available) {
      geometry_msgs::msg::PoseStamped source_pose;
      source_pose.header = pose.header;
      source_pose.pose = pose.pose;
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(source_pose, transformed_pose, transform);
      pose.header.frame_id = target_frame_id_;
      pose.pose = transformed_pose.pose;
    }
    output.armors.push_back(std::move(pose));
  }

  pose_pub_->publish(output);
  publishMarkers(output);
}

void ArmorSolverNode::publishMarkers(const hero_msgs::msg::ArmorPoseArray & poses)
{
  if (!marker_pub_ || marker_pub_->get_subscription_count() == 0U) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(poses.armors.size() + static_cast<std::size_t>(last_marker_count_));
  int marker_id = 0;
  for (const auto & armor : poses.armors) {
    markers.markers.push_back(makeMarker(armor, marker_id++));
  }
  for (int stale_id = marker_id; stale_id < last_marker_count_; ++stale_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = poses.header;
    marker.ns = "armor_pose";
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(marker));
  }
  last_marker_count_ = marker_id;
  marker_pub_->publish(std::move(markers));
}

}  // armor_solver
