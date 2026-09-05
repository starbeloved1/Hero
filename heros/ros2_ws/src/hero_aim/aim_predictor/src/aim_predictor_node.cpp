#include "aim_predictor/aim_predictor_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace aim_predictor {

namespace {

rclcpp::QoS highRateQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

double stampToSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw_rad) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw_rad * 0.5);
  quaternion.w = std::cos(yaw_rad * 0.5);
  return quaternion;
}

std::optional<Eigen::Vector3d>
planeNormal(const geometry_msgs::msg::Quaternion &orientation) {
  const double norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(norm) || norm < 1e-6) {
    return std::nullopt;
  }
  const double x = orientation.x / norm;
  const double y = orientation.y / norm;
  const double z = orientation.z / norm;
  const double w = orientation.w / norm;
  // PnP 装甲板局部 z 轴是板面法向。
  const double normal_x = 2.0 * (x * z + w * y);
  const double normal_y = 2.0 * (y * z - w * x);
  const double normal_z = 1.0 - 2.0 * (x * x + y * y);
  return Eigen::Vector3d(normal_x, normal_y, normal_z);
}

geometry_msgs::msg::Pose
poseFromMeasurement(const ArmorMeasurement &measurement) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = measurement.position_m.x();
  pose.position.y = measurement.position_m.y();
  pose.position.z = measurement.position_m.z();
  pose.orientation = quaternionFromYaw(measurement.yaw_rad);
  return pose;
}

visualization_msgs::msg::Marker
makeCubeMarker(const std_msgs::msg::Header &header,
               const std::string &name_space, int marker_id,
               const geometry_msgs::msg::Pose &pose, float red, float green,
               float blue) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  // 预测板的局部 x 轴是板到车心的内向方向，因此 x 为板厚，y/z 为板宽高。
  marker.scale.x = 0.01;
  marker.scale.y = 0.135;
  marker.scale.z = 0.056;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 0.85F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  return marker;
}

visualization_msgs::msg::Marker
makeInwardArrowMarker(const std_msgs::msg::Header &header,
                      const std::string &name_space, int marker_id,
                      const geometry_msgs::msg::Pose &pose, float red,
                      float green, float blue) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.02;
  marker.scale.z = 0.03;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 0.95F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.points.push_back(pose.position);
  geometry_msgs::msg::Point end = pose.position;
  const double yaw_rad =
      std::atan2(2.0 * (pose.orientation.w * pose.orientation.z +
                        pose.orientation.x * pose.orientation.y),
                 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                              pose.orientation.z * pose.orientation.z));
  end.x += 0.1 * std::cos(yaw_rad);
  end.y += 0.1 * std::sin(yaw_rad);
  marker.points.push_back(end);
  return marker;
}

void appendDeleteMarkers(visualization_msgs::msg::MarkerArray &markers,
                         const std_msgs::msg::Header &header,
                         const std::string &cube_namespace,
                         const std::string &arrow_namespace, int current_count,
                         int previous_count) {
  for (int marker_id = current_count; marker_id < previous_count; ++marker_id) {
    visualization_msgs::msg::Marker cube;
    cube.header = header;
    cube.ns = cube_namespace;
    cube.id = marker_id;
    cube.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(cube));

    visualization_msgs::msg::Marker arrow;
    arrow.header = header;
    arrow.ns = arrow_namespace;
    arrow.id = marker_id;
    arrow.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(arrow));
  }
}

} // namespace

AimPredictorNode::AimPredictorNode() : Node("aim_predictor_node") {
  declare_parameter<bool>("enabled", false);
  declare_parameter<std::string>("armor_pose_topic",
                                 "/hero/solver/armor_poses");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("target_state_topic",
                                 "/hero/aim/autoaim/target_states");
  declare_parameter<std::string>("target_frame_id", "world");
  declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.02);
  declare_parameter<bool>("visualization_enabled", true);
  declare_parameter<std::string>("visualization_topic",
                                 "/hero/aim/autoaim/predictor/markers");
  declare_parameter<double>("init_radius_m", 0.2);
  declare_parameter<int>("min_consecutive_detections", 3);
  declare_parameter<int>("max_lost_frames", 10);
  declare_parameter<double>("low_speed_process_noise_xy", 100.0);
  declare_parameter<double>("low_speed_process_noise_z", 100.0);
  declare_parameter<double>("low_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("middle_speed_process_noise_xy", 100.0);
  declare_parameter<double>("middle_speed_process_noise_z", 100.0);
  declare_parameter<double>("middle_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("high_speed_process_noise_xy", 100.0);
  declare_parameter<double>("high_speed_process_noise_z", 100.0);
  declare_parameter<double>("high_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("middle_speed_angular_velocity_threshold", 2.0);
  declare_parameter<double>("high_speed_angular_velocity_threshold", 4.0);
  declare_parameter<double>("measurement_noise_yaw", 0.004);
  declare_parameter<double>("measurement_noise_pitch", 0.004);
  declare_parameter<double>("measurement_noise_distance_base", 0.1);
  declare_parameter<double>("measurement_noise_armor_yaw_base", 0.09);
  declare_parameter<double>("min_valid_radius_m", 0.05);
  declare_parameter<double>("max_valid_radius_m", 0.5);

  enabled_ = get_parameter("enabled").as_bool();
  target_frame_id_ = get_parameter("target_frame_id").as_string();
  camera_frame_id_ = get_parameter("camera_frame_id").as_string();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  tracker_config_.init_radius_m = get_parameter("init_radius_m").as_double();
  tracker_config_.min_consecutive_detections =
      get_parameter("min_consecutive_detections").as_int();
  tracker_config_.max_lost_frames = get_parameter("max_lost_frames").as_int();
  tracker_config_.low_speed_process_noise_xy =
      get_parameter("low_speed_process_noise_xy").as_double();
  tracker_config_.low_speed_process_noise_z =
      get_parameter("low_speed_process_noise_z").as_double();
  tracker_config_.low_speed_process_noise_yaw =
      get_parameter("low_speed_process_noise_yaw").as_double();
  tracker_config_.middle_speed_process_noise_xy =
      get_parameter("middle_speed_process_noise_xy").as_double();
  tracker_config_.middle_speed_process_noise_z =
      get_parameter("middle_speed_process_noise_z").as_double();
  tracker_config_.middle_speed_process_noise_yaw =
      get_parameter("middle_speed_process_noise_yaw").as_double();
  tracker_config_.high_speed_process_noise_xy =
      get_parameter("high_speed_process_noise_xy").as_double();
  tracker_config_.high_speed_process_noise_z =
      get_parameter("high_speed_process_noise_z").as_double();
  tracker_config_.high_speed_process_noise_yaw =
      get_parameter("high_speed_process_noise_yaw").as_double();
  tracker_config_.middle_speed_angular_velocity_threshold =
      get_parameter("middle_speed_angular_velocity_threshold").as_double();
  tracker_config_.high_speed_angular_velocity_threshold =
      get_parameter("high_speed_angular_velocity_threshold").as_double();
  tracker_config_.measurement_noise_yaw =
      get_parameter("measurement_noise_yaw").as_double();
  tracker_config_.measurement_noise_pitch =
      get_parameter("measurement_noise_pitch").as_double();
  tracker_config_.measurement_noise_distance_base =
      get_parameter("measurement_noise_distance_base").as_double();
  tracker_config_.measurement_noise_armor_yaw_base =
      get_parameter("measurement_noise_armor_yaw_base").as_double();
  tracker_config_.min_valid_radius_m =
      get_parameter("min_valid_radius_m").as_double();
  tracker_config_.max_valid_radius_m =
      get_parameter("max_valid_radius_m").as_double();
  TargetTracker validation_tracker(tracker_config_);

  const auto armor_pose_topic = get_parameter("armor_pose_topic").as_string();
  const auto gimbal_state_topic =
      get_parameter("gimbal_state_topic").as_string();
  const auto target_state_topic =
      get_parameter("target_state_topic").as_string();
  const auto visualization_enabled =
      get_parameter("visualization_enabled").as_bool();
  const auto visualization_topic =
      get_parameter("visualization_topic").as_string();
  if (target_frame_id_.empty() || camera_frame_id_.empty() ||
      armor_pose_topic.empty() || gimbal_state_topic.empty() ||
      target_state_topic.empty()) {
    throw std::invalid_argument("预测器字符串参数不能为空");
  }

  const auto qos = highRateQos();
  if (tf_lookup_timeout_sec_ < 0.0) {
    throw std::invalid_argument("预测器 TF 查询超时不能为负数");
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, this, true, qos, tf2_ros::StaticListenerQoS());
  target_state_pub_ = create_publisher<hero_msgs::msg::TargetStateArray>(
      target_state_topic, qos);
  if (visualization_enabled) {
    if (visualization_topic.empty()) {
      throw std::invalid_argument("预测器可视化话题不能为空");
    }
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        visualization_topic, qos);
  }
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
      gimbal_state_topic, qos,
      [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {
        receiveGimbalState(message);
      });
  armor_pose_sub_ = create_subscription<hero_msgs::msg::ArmorPoseArray>(
      armor_pose_topic, qos,
      [this](const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr message) {
        receiveArmorPoses(message);
      });

  RCLCPP_INFO(get_logger(), "自瞄预测节点已启动：输入 %s，状态输出 %s，当前%s",
              armor_pose_topic.c_str(), target_state_topic.c_str(),
              enabled_ ? "启用" : "禁用");
}

void AimPredictorNode::receiveGimbalState(
    const hero_msgs::msg::GimbalState::ConstSharedPtr &message) {
  // 检测右键上升沿或者离开mode3清空状态
  if (message->right_clicked && !previous_right_clicked_) {
    reset();
  }
  previous_right_clicked_ = message->right_clicked;
  if (message->mode != hero_msgs::msg::GimbalState::MODE_AUTO_AIM) {
    reset();
  }
  latest_gimbal_state_ = *message;
}

void AimPredictorNode::receiveArmorPoses(
    const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr &message) {
  if (!enabled_ || !latest_gimbal_state_.has_value() ||
      latest_gimbal_state_->mode !=
          hero_msgs::msg::GimbalState::MODE_AUTO_AIM) {
    return;
  }
  if (message->header.frame_id != target_frame_id_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "自瞄预测仅接受 %s 坐标系位姿，当前为 %s",
                         target_frame_id_.c_str(),
                         message->header.frame_id.c_str());
    return;
  }
  const auto measurements = collectMeasurements(*message);
  updateTrackers(measurements, stampToSeconds(message->header.stamp));
  publishStates(message->header, measurements);
}

AimPredictorNode::MeasurementsById AimPredictorNode::collectMeasurements(
    const hero_msgs::msg::ArmorPoseArray &message) {
  MeasurementsById output;
  const auto camera_position = lookupCameraPosition(message.header);
  if (!camera_position.has_value()) {
    return output;
  }
  for (const auto &armor : message.armors) {
    const auto &position = armor.pose.position;
    const auto normal = planeNormal(armor.pose.orientation);
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !normal.has_value()) {
      continue;
    }
    const Eigen::Vector3d armor_position(position.x, position.y, position.z);
    Eigen::Vector3d visible_normal = *normal;
    // 平面 PnP 的法向正负没有业务语义。统一为由装甲板指向拍摄相机的可见面法向。
    if (visible_normal.dot(*camera_position - armor_position) < 0.0) {
      visible_normal = -visible_normal;
    }
    const Eigen::Vector2d inward = -visible_normal.head<2>();
    if (inward.norm() < 1e-6) {
      continue;
    }
    const double inward_yaw_rad = std::atan2(inward.y(), inward.x());
    output[armor.id].push_back(
        ArmorMeasurement{armor_position, inward_yaw_rad});
  }
  return output;
}

std::optional<Eigen::Vector3d> AimPredictorNode::lookupCameraPosition(
    const std_msgs::msg::Header &measurement_header) {
  try {
    const auto transform = tf_buffer_->lookupTransform(
        target_frame_id_, camera_frame_id_,
        rclcpp::Time(measurement_header.stamp),
        tf2::durationFromSec(tf_lookup_timeout_sec_));
    const auto &translation = transform.transform.translation;
    return Eigen::Vector3d(translation.x, translation.y, translation.z);
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "预测器未找到同刻相机 TF %s <- %s：%s",
                         target_frame_id_.c_str(), camera_frame_id_.c_str(),
                         error.what());
    return std::nullopt;
  }
}

void AimPredictorNode::updateTrackers(const MeasurementsById &measurements,
                                      double stamp_sec) {
  // 先预测到T0时刻，记一次丢失
  for (auto &[id, tracker] : trackers_) {
    static_cast<void>(id);
    tracker.predict(stamp_sec);
    tracker.markLost();
  }

  for (const auto &[id, target_measurements] : measurements) {
    const int detection_count = ++consecutive_detection_counts_[id];
    auto tracker_it = trackers_.find(id);
    const bool needs_initialize =
        tracker_it == trackers_.end() || !tracker_it->second.active() ||
        tracker_it->second.diverged() ||
        tracker_it->second.lostFrames() > tracker_config_.max_lost_frames;
    if (needs_initialize) {
      if (detection_count < tracker_config_.min_consecutive_detections) {
        continue;
      }
      trackers_.erase(id);
      tracker_it = trackers_.emplace(id, TargetTracker(tracker_config_)).first;
      tracker_it->second.initialize(id, target_measurements.front(), stamp_sec);
      for (std::size_t index = 1; index < target_measurements.size(); ++index) {
        tracker_it->second.update(target_measurements[index]);
      }
    } else {
      for (const auto &measurement : target_measurements) {
        tracker_it->second.update(measurement);
      }
    }
  }

  for (auto it = consecutive_detection_counts_.begin();
       it != consecutive_detection_counts_.end();) {
    if (measurements.find(it->first) == measurements.end()) {
      it = consecutive_detection_counts_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = trackers_.begin(); it != trackers_.end();) {
    if (it->second.diverged() ||
        it->second.lostFrames() > tracker_config_.max_lost_frames) {
      it = trackers_.erase(it);
    } else {
      ++it;
    }
  }
}

void AimPredictorNode::publishStates(
    const std_msgs::msg::Header &measurement_header,
    const MeasurementsById &measurements) {
  hero_msgs::msg::TargetStateArray output;
  output.header.stamp = now();
  output.header.frame_id = target_frame_id_;
  output.measurement_stamp = measurement_header.stamp;
  const double prediction_stamp_sec = stampToSeconds(output.header.stamp);
  for (const auto &[id, tracker] : trackers_) {
    static_cast<void>(id);
    const TargetEstimate estimate = tracker.estimate(prediction_stamp_sec);
    if (!estimate.tracking) {
      continue;
    }
    hero_msgs::msg::TargetState target;
    target.header = output.header;
    target.id = estimate.id;
    target.tracking = estimate.tracking;
    target.converged = estimate.converged;
    target.jumped = estimate.jumped;
    target.center.x = estimate.center_m.x();
    target.center.y = estimate.center_m.y();
    target.center.z = estimate.center_m.z();
    target.velocity.x = estimate.velocity_mps.x();
    target.velocity.y = estimate.velocity_mps.y();
    target.velocity.z = estimate.velocity_mps.z();
    target.yaw = estimate.yaw_rad;
    target.angular_velocity = estimate.angular_velocity_radps;
    target.radius = estimate.radius_m;
    target.radius_offset = estimate.radius_offset_m;
    target.height_offset = estimate.height_offset_m;
    target.predicted_armors.reserve(estimate.armors.size());
    for (const auto &armor : estimate.armors) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = armor.position_m.x();
      pose.position.y = armor.position_m.y();
      pose.position.z = armor.position_m.z();
      pose.orientation = quaternionFromYaw(armor.yaw_rad);
      target.predicted_armors.push_back(std::move(pose));
    }
    output.targets.push_back(std::move(target));
  }
  target_state_pub_->publish(output);
  publishVisualization(output, measurements);
}

void AimPredictorNode::publishVisualization(
    const hero_msgs::msg::TargetStateArray &states,
    const MeasurementsById &measurements) {
  if (!visualization_pub_ ||
      visualization_pub_->get_subscription_count() == 0U) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  int predicted_marker_id = 0;
  for (const auto &target : states.targets) {
    for (const auto &armor : target.predicted_armors) {
      markers.markers.push_back(makeCubeMarker(states.header, "predicted_armor",
                                               predicted_marker_id, armor, 0.2F,
                                               0.8F, 1.0F));
      markers.markers.push_back(
          makeInwardArrowMarker(states.header, "predicted_armor_inward",
                                predicted_marker_id, armor, 1.0F, 0.2F, 0.2F));
      ++predicted_marker_id;
    }
  }
  appendDeleteMarkers(markers, states.header, "predicted_armor",
                      "predicted_armor_inward", predicted_marker_id,
                      last_predicted_marker_count_);
  last_predicted_marker_count_ = predicted_marker_id;

  std_msgs::msg::Header measurement_header = states.header;
  measurement_header.stamp = states.measurement_stamp;
  int measurement_marker_id = 0;
  for (const auto &[target_id, target_measurements] : measurements) {
    static_cast<void>(target_id);
    for (const auto &measurement : target_measurements) {
      const auto pose = poseFromMeasurement(measurement);
      markers.markers.push_back(
          makeCubeMarker(measurement_header, "measurement_armor",
                         measurement_marker_id, pose, 1.0F, 0.9F, 0.1F));
      markers.markers.push_back(makeInwardArrowMarker(
          measurement_header, "measurement_armor_inward", measurement_marker_id,
          pose, 1.0F, 0.45F, 0.0F));
      ++measurement_marker_id;
    }
  }
  appendDeleteMarkers(markers, measurement_header, "measurement_armor",
                      "measurement_armor_inward", measurement_marker_id,
                      last_measurement_marker_count_);
  last_measurement_marker_count_ = measurement_marker_id;
  visualization_pub_->publish(std::move(markers));
}

void AimPredictorNode::reset() {
  trackers_.clear();
  consecutive_detection_counts_.clear();
}

} // namespace aim_predictor
