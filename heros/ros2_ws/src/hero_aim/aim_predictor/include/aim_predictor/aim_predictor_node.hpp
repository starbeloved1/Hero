#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <hero_msgs/msg/armor_pose_array.hpp>
#include <hero_msgs/msg/gimbal_state.hpp>
#include <hero_msgs/msg/target_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "aim_predictor/target_tracker.hpp"

namespace aim_predictor
{

// autoaim 预测节点：由装甲板测量维护目标车状态，不输出云台控制命令
class AimPredictorNode : public rclcpp::Node
{
public:
  AimPredictorNode();

private:
  using MeasurementsById = std::map<std::uint8_t, std::vector<ArmorMeasurement>>;

  void receiveGimbalState(const hero_msgs::msg::GimbalState::ConstSharedPtr & message);
  // autoaim 主入口
  void receiveArmorPoses(const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr & message);
  // 工具函数，两者一起查询可见装甲板外法向量（朝向相机）
  [[nodiscard]] MeasurementsById collectMeasurements(
    const hero_msgs::msg::ArmorPoseArray & message);
  [[nodiscard]] std::optional<Eigen::Vector3d> lookupCameraPosition(
    const std_msgs::msg::Header & measurement_header);
  // update
  void updateTrackers(const MeasurementsById & measurements, double stamp_sec);
  void publishStates(const std_msgs::msg::Header & measurement_header);
  void reset();

  rclcpp::Subscription<hero_msgs::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<hero_msgs::msg::ArmorPoseArray>::SharedPtr armor_pose_sub_;
  rclcpp::Publisher<hero_msgs::msg::TargetStateArray>::SharedPtr target_state_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<hero_msgs::msg::GimbalState> latest_gimbal_state_;
  std::map<std::uint8_t, TargetTracker> trackers_;
  std::map<std::uint8_t, int> consecutive_detection_counts_;
  TrackerConfig tracker_config_;
  bool enabled_{false};
  bool previous_right_clicked_{false};
  std::string target_frame_id_;
  std::string camera_frame_id_;
  double tf_lookup_timeout_sec_{0.02};
};

}  // aim_predictor
