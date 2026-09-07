#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hero_msgs/msg/armor_array.hpp"
#include "hero_msgs/msg/armor_pose_array.hpp"
#include "armor_solver/armor_pnp.hpp"

namespace armor_solver
{

class ArmorSolverNode : public rclcpp::Node
{
public:
  ArmorSolverNode();

private:
  void receiveCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message);
  void receiveArmors(const hero_msgs::msg::ArmorArray::ConstSharedPtr & message);
  void publishMarkers(const hero_msgs::msg::ArmorPoseArray & poses);

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<hero_msgs::msg::ArmorArray>::SharedPtr armor_sub_;
  rclcpp::Publisher<hero_msgs::msg::ArmorPoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex intrinsics_mutex_;
  CameraIntrinsics intrinsics_;
  bool has_intrinsics_{false};
  std::string target_frame_id_;
  double tf_lookup_timeout_sec_{0.05};
  int last_marker_count_{0};
};

}  // armor_solver
