#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <hero_msgs/msg/antitop_debug.hpp>
#include <hero_msgs/msg/armor_pose_array.hpp>
#include <hero_msgs/msg/control_command.hpp>
#include <hero_msgs/msg/gimbal_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_antitop/antitop_aimer.hpp"
#include "aim_antitop/antitop_controller.hpp"
#include "aim_antitop/antitop_tracker.hpp"

namespace aim_antitop {
class AimAntitopNode : public rclcpp::Node {
public:
  AimAntitopNode();

private:
  void receiveGimbalState(
      const hero_msgs::msg::GimbalState::ConstSharedPtr &message);
  void receiveCameraInfo(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &message);
  void receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr &message);
  void receiveArmorPoses(
      const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr &message);
  std::optional<double>
  projectRotationCenterX(const AntitopTrackerState &tracker_state,
                         const builtin_interfaces::msg::Time &stamp);
  void publishDebug(
      const std::optional<AntitopTrackerState> &tracker_state,
      const std::optional<AntitopControllerResult> &controller_result,
      const rclcpp::Time &control_time);
  void
  publishVisualization(const hero_msgs::msg::ArmorPoseArray &message,
                       const std::optional<AntitopTrackerState> &tracker_state,
                       const std::optional<AntitopAimResult> &result,
                       const rclcpp::Time &control_time);
  void publishImageVisualization(
      const hero_msgs::msg::ArmorPoseArray &message,
      const std::optional<AntitopTrackerState> &tracker_state,
      const std::optional<double> &center_image_x_px,
      const std::optional<AntitopControllerResult> &controller_result);

  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr
      gimbal_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<hero_msgs::msg::ArmorPoseArray>::SharedPtr
      armor_pose_sub_;
  rclcpp::Publisher<hero_msgs::msg::ControlCommand>::SharedPtr
      control_candidate_pub_;
  rclcpp::Publisher<hero_msgs::msg::AntitopDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_visualization_pub_;

  std::optional<hero_msgs::msg::GimbalState> latest_gimbal_state_;
  std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;
  std::mutex image_mutex_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> recent_images_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  AntitopTracker tracker_;
  AntitopAimer aimer_;
  AntitopController controller_;
  bool previous_right_clicked_{false};
  bool enabled_{false};
  bool enable_fire_{false};
  double enter_zone_threshold_px_{25.0};
  double exit_zone_threshold_px_{35.0};
  std::string target_frame_id_;
};

} // namespace aim_antitop
