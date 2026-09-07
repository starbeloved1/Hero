#include "aim_antitop/aim_antitop_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace aim_antitop {

namespace {

constexpr double kPi = 3.14159265358979323846;

rclcpp::QoS highRateQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

double stampToSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secondsToStamp(double seconds) {
  builtin_interfaces::msg::Time stamp;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return stamp;
  }
  const auto nanoseconds = static_cast<std::int64_t>(seconds * 1e9);
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

std::size_t positiveSizeParameter(const rclcpp::Node &node, const char *name) {
  const auto value = node.get_parameter(name).as_int();
  if (value <= 0) {
    throw std::invalid_argument(std::string(name) + " 必须大于 0");
  }
  return static_cast<std::size_t>(value);
}

visualization_msgs::msg::Marker
makeSphereMarker(const std_msgs::msg::Header &header,
                 const std::string &name_space, int id,
                 const Eigen::Vector3d &point_m, double diameter_m, float red,
                 float green, float blue) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point_m.x();
  marker.pose.position.y = point_m.y();
  marker.pose.position.z = point_m.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = diameter_m;
  marker.scale.y = diameter_m;
  marker.scale.z = diameter_m;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 0.9F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  return marker;
}

visualization_msgs::msg::Marker
makeLineMarker(const std_msgs::msg::Header &header,
               const std::string &name_space, int id,
               const Eigen::Vector3d &start_m, const Eigen::Vector3d &end_m,
               double width_m, float red, float green, float blue) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = width_m;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 0.9F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  geometry_msgs::msg::Point start;
  start.x = start_m.x();
  start.y = start_m.y();
  start.z = start_m.z();
  geometry_msgs::msg::Point end;
  end.x = end_m.x();
  end.y = end_m.y();
  end.z = end_m.z();
  marker.points = {start, end};
  return marker;
}

visualization_msgs::msg::Marker
makeOutpostArmorMarker(const std_msgs::msg::Header &header, int id,
                       const Eigen::Vector3d &position_m,
                       double inward_yaw_rad) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "outpost_armor";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = position_m.x();
  marker.pose.position.y = position_m.y();
  marker.pose.position.z = position_m.z();
  marker.pose.orientation.z = std::sin(inward_yaw_rad * 0.5);
  marker.pose.orientation.w = std::cos(inward_yaw_rad * 0.5);
  // 局部 x 轴朝向旋转中心：x 为板厚，y/z 为板宽高
  marker.scale.x = 0.01;
  marker.scale.y = 0.135;
  marker.scale.z = 0.056;
  marker.color.r = 0.2F;
  marker.color.g = 0.7F;
  marker.color.b = 1.0F;
  marker.color.a = 0.8F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  return marker;
}

visualization_msgs::msg::Marker
makeInwardArrowMarker(const std_msgs::msg::Header &header, int id,
                      const Eigen::Vector3d &position_m,
                      double inward_yaw_rad) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "outpost_armor_inward";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.015;
  marker.scale.y = 0.03;
  marker.scale.z = 0.04;
  marker.color.r = 1.0F;
  marker.color.g = 0.2F;
  marker.color.b = 0.6F;
  marker.color.a = 0.9F;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  geometry_msgs::msg::Point start;
  start.x = position_m.x();
  start.y = position_m.y();
  start.z = position_m.z();
  geometry_msgs::msg::Point end = start;
  end.x += 0.12 * std::cos(inward_yaw_rad);
  end.y += 0.12 * std::sin(inward_yaw_rad);
  marker.points = {start, end};
  return marker;
}

void appendDeleteMarker(visualization_msgs::msg::MarkerArray &markers,
                        const std_msgs::msg::Header &header,
                        const std::string &name_space, int id) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  markers.markers.push_back(std::move(marker));
}

} // namespace

AimAntitopNode::AimAntitopNode()
    : Node("aim_antitop_node"), tracker_(AntitopTrackerConfig{}),
      aimer_(AntitopAimerConfig{}), controller_(AntitopControllerConfig{}) {
  declare_parameter<bool>("enabled", false);
  declare_parameter<bool>("enable_fire", false);
  declare_parameter<std::string>("armor_pose_topic",
                                 "/hero/solver/armor_poses");
  declare_parameter<std::string>("gimbal_state_topic", "/hero/gimbal/state");
  declare_parameter<std::string>("camera_info_topic",
                                 "/hero/camera/aim8mm/camera_info");
  declare_parameter<std::string>("control_candidate_topic",
                                 "/hero/aim/antitop/controller");
  declare_parameter<std::string>("debug_topic", "/hero/aim/antitop/debug");
  declare_parameter<bool>("visualization_enabled", true);
  declare_parameter<std::string>("visualization_topic",
                                 "/hero/aim/antitop/markers");
  declare_parameter<std::string>("target_frame_id", "world");
  declare_parameter<int>("outpost_armor_id", 7);
  declare_parameter<int>("center_window_size", 300);
  declare_parameter<int>("minimum_center_samples", 10);
  declare_parameter<int>("calibration_start_center_samples", 40);
  declare_parameter<int>("calibration_min_samples", 240);
  declare_parameter<int>("calibration_max_samples", 360);
  declare_parameter<int>("minimum_layer_samples", 12);
  declare_parameter<double>("z_histogram_bin_size_m", 0.01);
  declare_parameter<double>("z_calibration_match_threshold_m", 0.05);
  declare_parameter<double>("z_layer_gap_min_m", 0.08);
  declare_parameter<double>("z_layer_gap_max_m", 0.12);
  declare_parameter<double>("max_pixel_jump_px", 20.0);
  declare_parameter<int>("direction_window_size", 10);
  declare_parameter<double>("direction_threshold_px", 0.0002);
  declare_parameter<int>("minimum_center_samples_for_zone", 80);
  declare_parameter<double>("enter_zone_threshold_px", 25.0);
  declare_parameter<double>("exit_zone_threshold_px", 35.0);
  declare_parameter<int>("recent_z_window_size", 20);
  declare_parameter<double>("z_runtime_match_threshold_m", 0.05);
  declare_parameter<double>("minimum_period_sec", 0.5);
  declare_parameter<double>("maximum_period_sec", 1.0);
  declare_parameter<int>("maximum_period_history_size", 20);
  declare_parameter<double>("system_delay_sec", 0.05);
  declare_parameter<double>("clockwise_time_bias_sec", 0.0);
  declare_parameter<double>("counterclockwise_time_bias_sec", 0.0);
  // 公共弹道参数没有 C++ 兜底值；必须由 aim_core/config/ballistics.yaml 提供
  declare_parameter("bullet_speed_mps",
                    rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("drag_coefficient",
                    rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("gravity_mps2", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("air_density_kgpm3",
                    rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_mass_kg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("bullet_radius_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("muzzle_offset_m", rclcpp::ParameterType::PARAMETER_DOUBLE);
  declare_parameter("ballistic_iteration_count",
                    rclcpp::ParameterType::PARAMETER_INTEGER);

  enabled_ = get_parameter("enabled").as_bool();
  enable_fire_ = get_parameter("enable_fire").as_bool();
  target_frame_id_ = get_parameter("target_frame_id").as_string();
  const auto outpost_armor_id = get_parameter("outpost_armor_id").as_int();
  const auto ballistic_iteration_count =
      get_parameter("ballistic_iteration_count").as_int();
  if (outpost_armor_id <= 0 || outpost_armor_id > 255 ||
      ballistic_iteration_count <= 0) {
    throw std::invalid_argument("反前哨整数参数无效");
  }
  AntitopTrackerConfig tracker_config;
  tracker_config.outpost_armor_id = static_cast<std::uint8_t>(outpost_armor_id);
  tracker_config.center_window_size =
      positiveSizeParameter(*this, "center_window_size");
  tracker_config.minimum_center_samples =
      positiveSizeParameter(*this, "minimum_center_samples");
  tracker_config.calibration_start_center_samples =
      positiveSizeParameter(*this, "calibration_start_center_samples");
  tracker_config.calibration_min_samples =
      positiveSizeParameter(*this, "calibration_min_samples");
  tracker_config.calibration_max_samples =
      positiveSizeParameter(*this, "calibration_max_samples");
  tracker_config.minimum_layer_samples =
      positiveSizeParameter(*this, "minimum_layer_samples");
  tracker_config.z_histogram_bin_size_m =
      get_parameter("z_histogram_bin_size_m").as_double();
  tracker_config.z_calibration_match_threshold_m =
      get_parameter("z_calibration_match_threshold_m").as_double();
  tracker_config.z_layer_gap_min_m =
      get_parameter("z_layer_gap_min_m").as_double();
  tracker_config.z_layer_gap_max_m =
      get_parameter("z_layer_gap_max_m").as_double();
  tracker_config.max_pixel_jump_px =
      get_parameter("max_pixel_jump_px").as_double();
  tracker_config.direction_window_size =
      positiveSizeParameter(*this, "direction_window_size");
  tracker_config.direction_threshold_px =
      get_parameter("direction_threshold_px").as_double();
  tracker_ = AntitopTracker(tracker_config);

  AntitopControllerConfig controller_config;
  controller_config.minimum_center_samples_for_zone =
      positiveSizeParameter(*this, "minimum_center_samples_for_zone");
  controller_config.enter_zone_threshold_px =
      get_parameter("enter_zone_threshold_px").as_double();
  controller_config.exit_zone_threshold_px =
      get_parameter("exit_zone_threshold_px").as_double();
  controller_config.recent_z_window_size =
      positiveSizeParameter(*this, "recent_z_window_size");
  controller_config.z_runtime_match_threshold_m =
      get_parameter("z_runtime_match_threshold_m").as_double();
  controller_config.minimum_period_sec =
      get_parameter("minimum_period_sec").as_double();
  controller_config.maximum_period_sec =
      get_parameter("maximum_period_sec").as_double();
  controller_config.maximum_period_history_size =
      positiveSizeParameter(*this, "maximum_period_history_size");
  controller_config.system_delay_sec =
      get_parameter("system_delay_sec").as_double();
  controller_config.clockwise_time_bias_sec =
      get_parameter("clockwise_time_bias_sec").as_double();
  controller_config.counterclockwise_time_bias_sec =
      get_parameter("counterclockwise_time_bias_sec").as_double();
  controller_ = AntitopController(controller_config);

  AntitopAimerConfig aimer_config;
  aimer_config.ballistics.bullet_speed_mps =
      get_parameter("bullet_speed_mps").as_double();
  aimer_config.ballistics.drag_coefficient =
      get_parameter("drag_coefficient").as_double();
  aimer_config.ballistics.gravity_mps2 =
      get_parameter("gravity_mps2").as_double();
  aimer_config.ballistics.air_density_kgpm3 =
      get_parameter("air_density_kgpm3").as_double();
  aimer_config.ballistics.bullet_mass_kg =
      get_parameter("bullet_mass_kg").as_double();
  aimer_config.ballistics.bullet_radius_m =
      get_parameter("bullet_radius_m").as_double();
  aimer_config.ballistics.muzzle_offset_m =
      get_parameter("muzzle_offset_m").as_double();
  aimer_config.ballistics.iteration_count = ballistic_iteration_count;
  aimer_ = AntitopAimer(aimer_config);

  const auto armor_pose_topic = get_parameter("armor_pose_topic").as_string();
  const auto gimbal_state_topic =
      get_parameter("gimbal_state_topic").as_string();
  const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
  const auto control_candidate_topic =
      get_parameter("control_candidate_topic").as_string();
  const auto debug_topic = get_parameter("debug_topic").as_string();
  const auto visualization_enabled =
      get_parameter("visualization_enabled").as_bool();
  const auto visualization_topic =
      get_parameter("visualization_topic").as_string();
  if (target_frame_id_.empty() || armor_pose_topic.empty() ||
      gimbal_state_topic.empty() || camera_info_topic.empty() ||
      control_candidate_topic.empty() || debug_topic.empty()) {
    throw std::invalid_argument("反前哨字符串参数不能为空");
  }

  const auto qos = highRateQos();
  control_candidate_pub_ = create_publisher<hero_msgs::msg::ControlCommand>(
      control_candidate_topic, qos);
  debug_pub_ = create_publisher<hero_msgs::msg::AntitopDebug>(debug_topic, qos);
  if (visualization_enabled) {
    if (visualization_topic.empty()) {
      throw std::invalid_argument("反前哨可视化话题不能为空");
    }
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        visualization_topic, qos);
  }
  gimbal_state_sub_ = create_subscription<hero_msgs::msg::GimbalState>(
      gimbal_state_topic, qos,
      [this](const hero_msgs::msg::GimbalState::ConstSharedPtr message) {
        receiveGimbalState(message);
      });
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        receiveCameraInfo(message);
      });
  armor_pose_sub_ = create_subscription<hero_msgs::msg::ArmorPoseArray>(
      armor_pose_topic, qos,
      [this](const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr message) {
        receiveArmorPoses(message);
      });
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  RCLCPP_INFO(get_logger(),
              "反前哨节点已启动：输入 %s，候选输出 %s，当前%s、%s开火",
              armor_pose_topic.c_str(), control_candidate_topic.c_str(),
              enabled_ ? "启用" : "禁用", enable_fire_ ? "允许" : "禁止");
}

void AimAntitopNode::receiveGimbalState(
    const hero_msgs::msg::GimbalState::ConstSharedPtr &message) {
  if (message->right_clicked && !previous_right_clicked_) {
    tracker_.reset();
    controller_.reset();
  }
  previous_right_clicked_ = message->right_clicked;
  if (message->mode != hero_msgs::msg::GimbalState::MODE_ANTI_TOP) {
    tracker_.reset();
    controller_.reset();
    latest_camera_info_.reset();
  }
  latest_gimbal_state_ = *message;
}

void AimAntitopNode::receiveCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &message) {
  // selected 在 mode4 切到基地相机；反前哨只应消费 mode2 的相机内参
  if (!latest_gimbal_state_.has_value() ||
      latest_gimbal_state_->mode !=
          hero_msgs::msg::GimbalState::MODE_ANTI_TOP) {
    return;
  }
  if (message->header.frame_id.empty() || message->k[0] <= 0.0 ||
      message->k[4] <= 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "反前哨忽略无效 CameraInfo");
    return;
  }
  latest_camera_info_ = *message;
}

std::optional<double> AimAntitopNode::projectRotationCenterX(
    const AntitopTrackerState &tracker_state,
    const builtin_interfaces::msg::Time &stamp) {
  if (!latest_camera_info_.has_value() || !tf_buffer_) {
    return std::nullopt;
  }
  try {
    const auto transform = tf_buffer_->lookupTransform(
        latest_camera_info_->header.frame_id, target_frame_id_,
        rclcpp::Time(stamp), tf2::durationFromSec(0.0));
    geometry_msgs::msg::PointStamped center_world;
    center_world.header.frame_id = target_frame_id_;
    center_world.header.stamp = stamp;
    center_world.point.x = tracker_state.rotation_center_m.x();
    center_world.point.y = tracker_state.rotation_center_m.y();
    center_world.point.z = tracker_state.rotation_center_m.z();
    geometry_msgs::msg::PointStamped center_camera;
    tf2::doTransform(center_world, center_camera, transform);
    if (!std::isfinite(center_camera.point.x) ||
        !std::isfinite(center_camera.point.y) ||
        !std::isfinite(center_camera.point.z) ||
        center_camera.point.z <= 1e-6) {
      return std::nullopt;
    }
    // selected 图像仍是原始畸变图像，必须与旧 Solver::reproject
    // 一样带畸变投影。 camera_optical_frame 遵循 RDF，恰好与 OpenCV 相机坐标系
    // x 右、y 下、z 前一致
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        camera_matrix.at<double>(row, column) =
            latest_camera_info_->k[3 * row + column];
      }
    }
    cv::Mat distortion_coefficients(
        static_cast<int>(latest_camera_info_->d.size()), 1, CV_64F);
    for (std::size_t index = 0U; index < latest_camera_info_->d.size();
         ++index) {
      distortion_coefficients.at<double>(static_cast<int>(index), 0) =
          latest_camera_info_->d[index];
    }
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(std::vector<cv::Point3d>{cv::Point3d(
                          center_camera.point.x, center_camera.point.y,
                          center_camera.point.z)},
                      cv::Vec3d::all(0.0), cv::Vec3d::all(0.0), camera_matrix,
                      distortion_coefficients, image_points);
    if (image_points.size() != 1U || !std::isfinite(image_points.front().x)) {
      return std::nullopt;
    }
    return image_points.front().x;
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "反前哨未找到同刻旋转中心 TF：%s", error.what());
    return std::nullopt;
  }
}

void AimAntitopNode::receiveArmorPoses(
    const hero_msgs::msg::ArmorPoseArray::ConstSharedPtr &message) {
  if (!latest_gimbal_state_.has_value() ||
      latest_gimbal_state_->mode !=
          hero_msgs::msg::GimbalState::MODE_ANTI_TOP) {
    return;
  }
  if (message->header.frame_id != target_frame_id_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "反前哨仅接受 %s 坐标系位姿，当前为 %s",
                         target_frame_id_.c_str(),
                         message->header.frame_id.c_str());
    return;
  }

  std::vector<AntitopObservation> observations;
  observations.reserve(message->armors.size());
  for (const auto &armor : message->armors) {
    const auto &position = armor.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
      continue;
    }
    observations.push_back(AntitopObservation{
        armor.id, Eigen::Vector3d(position.x, position.y, position.z),
        armor.image_center.x});
  }

  const auto tracker_state = tracker_.update(observations);
  const auto control_time = now();
  std::optional<AntitopAimResult> result;
  std::optional<double> center_image_x_px;
  std::optional<AntitopControllerResult> controller_result;
  if (tracker_state.has_value()) {
    result = aimer_.aim(*tracker_state, latest_gimbal_state_->yaw,
                        latest_gimbal_state_->pitch);
    if (result.has_value()) {
      center_image_x_px =
          projectRotationCenterX(*tracker_state, message->header.stamp);
      if (center_image_x_px.has_value()) {
        controller_result = controller_.update(
            *tracker_state, *center_image_x_px, result->flight_time_sec,
            stampToSeconds(message->header.stamp), control_time.seconds());
        // 最低层过区后，控制器把近期 Z 中位数作为目标高度；和旧 AntiTop 的
        // target_aim_z 一致，随后每帧都以这一高度重新解弹道
        result = aimer_.aim(*tracker_state, latest_gimbal_state_->yaw,
                            latest_gimbal_state_->pitch,
                            controller_result->target_z_m);
      } else {
        // 无法把旋转中心投影到本帧图像时，取消倒计时，不能保留潜在的旧开火许可
        controller_.reset();
      }
    }
  }
  const bool shoot_status = controller_result.has_value() && enable_fire_ &&
                            controller_result->shoot_ready;
  publishDebug(*message, tracker_state, result, center_image_x_px,
               controller_result, shoot_status, control_time);
  publishVisualization(*message, tracker_state, result, control_time);
  if (!enabled_ || !result.has_value()) {
    return;
  }

  hero_msgs::msg::ControlCommand command;
  command.header.stamp = control_time;
  command.header.frame_id = latest_gimbal_state_->header.frame_id;
  command.yaw = static_cast<float>(result->command_yaw_rad);
  command.pitch = static_cast<float>(result->command_pitch_rad);
  command.shoot_status = shoot_status ? 1U : 0U;
  command.target_id = result->tracker_state.tracked_armor.id;
  control_candidate_pub_->publish(command);
}

void AimAntitopNode::publishDebug(
    const hero_msgs::msg::ArmorPoseArray &message,
    const std::optional<AntitopTrackerState> &tracker_state,
    const std::optional<AntitopAimResult> &result,
    const std::optional<double> &center_image_x_px,
    const std::optional<AntitopControllerResult> &controller_result,
    bool shoot_status, const rclcpp::Time &control_time) {
  if (debug_pub_->get_subscription_count() == 0U) {
    return;
  }
  hero_msgs::msg::AntitopDebug debug;
  debug.header.stamp = control_time;
  debug.header.frame_id = target_frame_id_;
  debug.measurement_stamp = message.header.stamp;
  debug.tracking_valid = tracker_state.has_value();
  debug.fire_enabled = enable_fire_;
  debug.shoot_status = shoot_status;
  if (tracker_state.has_value()) {
    debug.center_valid = tracker_state->center_valid;
    debug.calibrated = tracker_state->calibrated;
    debug.target_id = tracker_state->tracked_armor.id;
    debug.tracked_position.x = tracker_state->tracked_armor.position_m.x();
    debug.tracked_position.y = tracker_state->tracked_armor.position_m.y();
    debug.tracked_position.z = tracker_state->tracked_armor.position_m.z();
    debug.rotation_center.x = tracker_state->rotation_center_m.x();
    debug.rotation_center.y = tracker_state->rotation_center_m.y();
    debug.rotation_center.z = tracker_state->rotation_center_m.z();
    debug.center_sample_count =
        static_cast<std::uint32_t>(tracker_state->center_sample_count);
    debug.calibration_sample_count =
        static_cast<std::uint32_t>(tracker_state->calibration_sample_count);
    debug.z_layers = tracker_state->z_layers_m;
    debug.tracked_image_x_px =
        static_cast<float>(tracker_state->tracked_armor.image_center_x_px);
    debug.rotation_direction = tracker_state->rotation_direction;
  }
  if (center_image_x_px.has_value() && tracker_state.has_value()) {
    debug.center_image_x_px = static_cast<float>(*center_image_x_px);
    debug.center_image_distance_px = static_cast<float>(std::abs(
        *center_image_x_px - tracker_state->tracked_armor.image_center_x_px));
  }
  if (result.has_value()) {
    debug.target_z = result->target_z_m;
    debug.raw_yaw = static_cast<float>(result->raw_yaw_rad);
    debug.raw_pitch = static_cast<float>(result->raw_pitch_rad);
    debug.command_yaw = static_cast<float>(result->command_yaw_rad);
    debug.command_pitch = static_cast<float>(result->command_pitch_rad);
    debug.flight_time_sec = static_cast<float>(result->flight_time_sec);
    debug.pitch_locked = result->pitch_locked;
  }
  if (controller_result.has_value()) {
    debug.matched_z_layer = controller_result->matched_z_layer;
    debug.average_period_sec =
        static_cast<float>(controller_result->average_period_sec);
    debug.zone_stamp = secondsToStamp(controller_result->zone_stamp_sec);
    debug.permit_stamp = secondsToStamp(controller_result->permit_stamp_sec);
    debug.in_shoot_zone = controller_result->in_shoot_zone;
    debug.countdown_active = controller_result->countdown_active;
    debug.countdown_remaining_sec =
        static_cast<float>(controller_result->countdown_remaining_sec);
    debug.shoot_ready = controller_result->shoot_ready;
  }
  debug_pub_->publish(debug);
}

void AimAntitopNode::publishVisualization(
    const hero_msgs::msg::ArmorPoseArray &message,
    const std::optional<AntitopTrackerState> &tracker_state,
    const std::optional<AntitopAimResult> &result,
    const rclcpp::Time &control_time) {
  if (!visualization_pub_ ||
      visualization_pub_->get_subscription_count() == 0U) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  // 跟踪器的数据直接来自本次图像观测，因此这些标记代表 T0
  const auto measurement_header = message.header;
  if (!tracker_state.has_value()) {
    appendDeleteMarker(markers, measurement_header, "tracked_armor", 0);
    appendDeleteMarker(markers, measurement_header, "tracked_radius", 0);
    appendDeleteMarker(markers, measurement_header, "rotation_axis", 0);
    for (int id = 0; id < 3; ++id) {
      appendDeleteMarker(markers, measurement_header, "z_layer", id);
      appendDeleteMarker(markers, measurement_header, "outpost_armor", id);
      appendDeleteMarker(markers, measurement_header, "outpost_armor_inward",
                         id);
    }
  } else {
    const auto &state = *tracker_state;
    markers.markers.push_back(makeSphereMarker(
        measurement_header, "tracked_armor", 0, state.tracked_armor.position_m,
        0.07, 0.1F, 0.9F, 1.0F));
    if (state.center_valid) {
      // 旋转中心只拟合 XY；线段末端采用当前板的
      // Z，仅用来表示该板的水平转动半径
      const Eigen::Vector3d axis_at_observation_height(
          state.rotation_center_m.x(), state.rotation_center_m.y(),
          state.tracked_armor.position_m.z());
      markers.markers.push_back(
          makeLineMarker(measurement_header, "tracked_radius", 0,
                         state.tracked_armor.position_m,
                         axis_at_observation_height, 0.01, 0.1F, 0.9F, 1.0F));

      const double radius_m = std::hypot(
          state.tracked_armor.position_m.x() - state.rotation_center_m.x(),
          state.tracked_armor.position_m.y() - state.rotation_center_m.y());
      if (std::isfinite(radius_m) && radius_m > 1e-4) {
        // 每帧由当前观测的极角重建同相位三层板，因此 Marker 会随前哨实时转动
        const double phase_rad = std::atan2(
            state.tracked_armor.position_m.y() - state.rotation_center_m.y(),
            state.tracked_armor.position_m.x() - state.rotation_center_m.x());
        const double inward_yaw_rad = phase_rad + kPi;
        const int layer_count = state.calibrated ? 3 : 1;
        for (int id = 0; id < layer_count; ++id) {
          const double z_m = state.calibrated
                                 ? state.z_layers_m[id]
                                 : state.tracked_armor.position_m.z();
          const Eigen::Vector3d armor_position(
              state.rotation_center_m.x() + radius_m * std::cos(phase_rad),
              state.rotation_center_m.y() + radius_m * std::sin(phase_rad),
              z_m);
          markers.markers.push_back(makeOutpostArmorMarker(
              measurement_header, id, armor_position, inward_yaw_rad));
          markers.markers.push_back(makeInwardArrowMarker(
              measurement_header, id, armor_position, inward_yaw_rad));
        }
        for (int id = layer_count; id < 3; ++id) {
          appendDeleteMarker(markers, measurement_header, "outpost_armor", id);
          appendDeleteMarker(markers, measurement_header,
                             "outpost_armor_inward", id);
        }
      } else {
        for (int id = 0; id < 3; ++id) {
          appendDeleteMarker(markers, measurement_header, "outpost_armor", id);
          appendDeleteMarker(markers, measurement_header,
                             "outpost_armor_inward", id);
        }
      }
    } else {
      appendDeleteMarker(markers, measurement_header, "tracked_radius", 0);
      for (int id = 0; id < 3; ++id) {
        appendDeleteMarker(markers, measurement_header, "outpost_armor", id);
        appendDeleteMarker(markers, measurement_header, "outpost_armor_inward",
                           id);
      }
    }

    if (state.calibrated && std::isfinite(state.z_layers_m[0]) &&
        std::isfinite(state.z_layers_m[1]) &&
        std::isfinite(state.z_layers_m[2])) {
      const Eigen::Vector3d axis_bottom(state.rotation_center_m.x(),
                                        state.rotation_center_m.y(),
                                        state.z_layers_m[0] - 0.04);
      const Eigen::Vector3d axis_top(state.rotation_center_m.x(),
                                     state.rotation_center_m.y(),
                                     state.z_layers_m[2] + 0.04);
      markers.markers.push_back(
          makeLineMarker(measurement_header, "rotation_axis", 0, axis_bottom,
                         axis_top, 0.015, 1.0F, 0.55F, 0.05F));
      for (int id = 0; id < 3; ++id) {
        markers.markers.push_back(makeSphereMarker(
            measurement_header, "z_layer", id,
            Eigen::Vector3d(state.rotation_center_m.x(),
                            state.rotation_center_m.y(), state.z_layers_m[id]),
            0.05, 1.0F, 0.55F, 0.05F));
      }
    } else {
      appendDeleteMarker(markers, measurement_header, "rotation_axis", 0);
      for (int id = 0; id < 3; ++id) {
        appendDeleteMarker(markers, measurement_header, "z_layer", id);
      }
    }
  }

  // 瞄点是本次策略在 Tcontrol 得到的弹道输入点，和 T0
  // 观测标记刻意分开保存时间戳
  std_msgs::msg::Header control_header;
  control_header.stamp = control_time;
  control_header.frame_id = target_frame_id_;
  if (result.has_value()) {
    const auto &state = result->tracker_state;
    markers.markers.push_back(makeSphereMarker(
        control_header, "aim_point", 0,
        Eigen::Vector3d(state.rotation_center_m.x(),
                        state.rotation_center_m.y(), result->target_z_m),
        0.09, 0.2F, 1.0F, 0.25F));
  } else {
    appendDeleteMarker(markers, control_header, "aim_point", 0);
  }
  visualization_pub_->publish(std::move(markers));
}

} // namespace aim_antitop
