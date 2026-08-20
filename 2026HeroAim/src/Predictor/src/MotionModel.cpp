#include "../include/MotionModel.hpp"

#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace predictor {
using lyutils::PredictorParam;
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultDt = 0.015;
constexpr double kMinDt = 1e-3;
constexpr double kMaxUpdateDt = 0.08;
constexpr double kMaxPredictDt = 0.35;
constexpr int kArmorCount = 4;
constexpr double kMinDistance = 1e-6;

inline double sanitizeDt(double dt, double max_dt)
{
    if (!std::isfinite(dt)) {
        return kDefaultDt;
    }
    return std::clamp(dt, kMinDt, max_dt);
}

inline double square(double v)
{
    return v * v;
}
} // namespace


void MotionModel::initialize(const ArmorMeasurement& armor, const Time::TimeStamp& timestamp)
{
    if (!finiteMeasurement(armor)) {
        return;
    }

    const double radius = std::max(PredictorParam::init_radius, PredictorParam::min_valid_radius);
    x_.setZero();
    x_[0] = armor.xyz.x + radius * std::cos(armor.yaw);
    x_[2] = armor.xyz.y + radius * std::sin(armor.yaw);
    x_[4] = armor.xyz.z;
    x_[6] = limitRad(armor.yaw);
    x_[8] = radius;

    StateVector p0_diag;
    p0_diag << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;
    p_ = p0_diag.asDiagonal();

    initialized_ = true;
    lost_count_ = 0;
    update_count_ = 1;
    last_timestamp_ = timestamp;
}

void MotionModel::predict(const Time::TimeStamp& timestamp)
{
    if (!initialized_) {
        return;
    }

    const double dt = sanitizeDt((timestamp - last_timestamp_).toSeconds(), kMaxUpdateDt);
    const StateMatrix f = transitionMatrix(dt);
    const StateMatrix q = processNoise(dt, x_);
    p_ = f * p_ * f.transpose() + q;
    x_ = transition(x_, dt);
    last_timestamp_ = timestamp;
}

void MotionModel::update(const ArmorMeasurement& armor)
{
    if (!initialized_ || !finiteMeasurement(armor)) {
        return;
    }

    const int armor_index = matchArmorIndex(armor);

    Eigen::Vector4d z;
    z << armor.ypd.x(), armor.ypd.y(), armor.ypd.z(), limitRad(armor.yaw);

    const auto h = observationJacobian(x_, armor_index);
    Eigen::Vector4d r_diag;
    const double center_yaw = std::atan2(armor.xyz.y, armor.xyz.x);
    const double delta_angle = std::abs(limitRad(armor.yaw - center_yaw));
    r_diag << PredictorParam::measurement_noise_yaw,
              PredictorParam::measurement_noise_pitch,
              std::log(std::abs(armor.ypd.z()) + 1.0) / 200.0 + PredictorParam::measurement_noise_distance_base,
              std::log(delta_angle + 1.0) + PredictorParam::measurement_noise_armor_yaw_base;

    const Eigen::Matrix4d r = r_diag.asDiagonal();
    ekfUpdate(z, h, r, armor_index);
    lost_count_ = 0;
    ++update_count_;
}

void MotionModel::markLost()
{
    if (initialized_) {
        ++lost_count_;
    }
}

bool MotionModel::active() const
{
    return initialized_;
}

bool MotionModel::Stable() const
{
    return initialized_ && update_count_ >= PredictorParam::min_consecutive_detections && !diverged();
}


bool MotionModel::diverged() const
{
    if (!initialized_) {
        return false;
    }
    const double r1 = x_[8];
    const double r2 = x_[8] + x_[9];
    const bool r1_ok = r1 > PredictorParam::min_valid_radius && r1 < PredictorParam::max_valid_radius;
    const bool r2_ok = r2 > PredictorParam::min_valid_radius && r2 < PredictorParam::max_valid_radius;
    return !(r1_ok && r2_ok);
}

int MotionModel::lostCount() const
{
    return lost_count_;
}




StateVector MotionModel::predictState(const Time::TimeStamp& timestamp) const
{
    if (!initialized_) {
        return StateVector::Zero();
    }
    const double dt = sanitizeDt((timestamp - last_timestamp_).toSeconds(), kMaxPredictDt);
    return transition(x_, dt);
}

std::array<Armor, 4> MotionModel::predictedArmors(const StateVector& state) const
{
    std::array<Armor, 4> armors;
    for (int i = 0; i < kArmorCount; ++i) {
        const Eigen::Vector3d xyz = armorPosition(state, i);
        const double yaw = limitRad(state[6] + i * kPi / 2.0);
        armors[static_cast<std::size_t>(i)].center = XYZ(xyz.x(), xyz.y(), xyz.z());
        armors[static_cast<std::size_t>(i)].yaw = yaw;
        armors[static_cast<std::size_t>(i)].theta = yaw;
        armors[static_cast<std::size_t>(i)].id = i;
        armors[static_cast<std::size_t>(i)].status = Armor::AVAILABLE;
    }
    return armors;
}

Prediction MotionModel::toPrediction(int car_id, const Time::TimeStamp& timestamp) const
{
    const StateVector state = predictState(timestamp);
    Prediction prediction;
    prediction.center = XYZ(state[0], state[2], state[4]);
    prediction.id = car_id;
    prediction.vx = state[1];
    prediction.vy = state[3];
    prediction.ax = 0.0;
    prediction.ay = 0.0;
    prediction.z1 = state[4];
    prediction.z2 = state[4] + state[10];
    prediction.theta = limitRad(state[6]);
    prediction.omega = state[7];
    prediction.r1 = state[8];
    prediction.r2 = state[8] + state[9];
    prediction.armors = predictedArmors(state);
    prediction.stable = Stable();
    return prediction;
}

double MotionModel::limitRad(double angle)
{
    return std::remainder(angle, 2.0 * kPi);
}

Eigen::Vector3d MotionModel::xyzToYpd(const Eigen::Vector3d& xyz)
{
    const double xy = std::hypot(xyz.x(), xyz.y());
    const double distance = std::max(xyz.norm(), kMinDistance);
    return {std::atan2(xyz.y(), xyz.x()), std::atan2(xyz.z(), xy), distance};
}

Eigen::Matrix3d MotionModel::xyzToYpdJacobian(const Eigen::Vector3d& xyz)
{
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();
    const double xy2 = std::max(x * x + y * y, kMinDistance);
    const double xy = std::sqrt(xy2);
    const double d2 = std::max(xy2 + z * z, kMinDistance);
    const double d = std::sqrt(d2);

    Eigen::Matrix3d j = Eigen::Matrix3d::Zero();
    j(0, 0) = -y / xy2;
    j(0, 1) = x / xy2;
    j(1, 0) = -x * z / (xy * d2);
    j(1, 1) = -y * z / (xy * d2);
    j(1, 2) = xy / d2;
    j(2, 0) = x / d;
    j(2, 1) = y / d;
    j(2, 2) = z / d;
    return j;
}

bool MotionModel::finiteMeasurement(const ArmorMeasurement& armor)
{
    return armor.car_id >= 0 &&
           std::isfinite(armor.xyz.x) && std::isfinite(armor.xyz.y) && std::isfinite(armor.xyz.z) &&
           std::isfinite(armor.yaw) && armor.ypd.allFinite() && armor.ypd.z() > 0.1;
}

StateVector MotionModel::transition(const StateVector& state, double dt) const
{
    StateVector next = state;
    next[0] += state[1] * dt;
    next[2] += state[3] * dt;
    next[4] += state[5] * dt;
    next[6] = limitRad(state[6] + state[7] * dt);
    return next;
}

StateMatrix MotionModel::transitionMatrix(double dt) const
{
    StateMatrix f = StateMatrix::Identity();
    f(0, 1) = dt;
    f(2, 3) = dt;
    f(4, 5) = dt;
    f(6, 7) = dt;
    return f;
}

StateMatrix MotionModel::processNoise(double dt, const StateVector& state) const
{
    const double middle_threshold = std::min(
        std::abs(PredictorParam::middle_speed_angular_velocity_threshold),
        std::abs(PredictorParam::high_speed_angular_velocity_threshold));
    const double high_threshold = std::max(
        std::abs(PredictorParam::middle_speed_angular_velocity_threshold),
        std::abs(PredictorParam::high_speed_angular_velocity_threshold));
    const double abs_omega = std::abs(state[7]);

    double q_xy = PredictorParam::high_speed_process_noise_xy;
    double q_z = PredictorParam::high_speed_process_noise_z;
    double q_yaw = PredictorParam::high_speed_process_noise_yaw;
    if (abs_omega < middle_threshold) {
        q_xy = PredictorParam::low_speed_process_noise_xy;
        q_z = PredictorParam::low_speed_process_noise_z;
        q_yaw = PredictorParam::low_speed_process_noise_yaw;
    } else if (abs_omega < high_threshold) {
        q_xy = PredictorParam::middle_speed_process_noise_xy;
        q_z = PredictorParam::middle_speed_process_noise_z;
        q_yaw = PredictorParam::middle_speed_process_noise_yaw;
    }

    const double a = square(square(dt)) / 4.0;
    const double b = dt * dt * dt / 2.0;
    const double c = dt * dt;

    StateMatrix q = StateMatrix::Zero();
    q(0, 0) = a * q_xy;
    q(0, 1) = b * q_xy;
    q(1, 0) = b * q_xy;
    q(1, 1) = c * q_xy;
    q(2, 2) = a * q_xy;
    q(2, 3) = b * q_xy;
    q(3, 2) = b * q_xy;
    q(3, 3) = c * q_xy;
    q(4, 4) = a * q_z;
    q(4, 5) = b * q_z;
    q(5, 4) = b * q_z;
    q(5, 5) = c * q_z;
    q(6, 6) = a * q_yaw;
    q(6, 7) = b * q_yaw;
    q(7, 6) = b * q_yaw;
    q(7, 7) = c * q_yaw;
    q(8, 8) = 1e-4 * dt;
    q(9, 9) = 1e-4 * dt;
    q(10, 10) = 1e-4 * dt;
    return q;
}

Eigen::Vector3d MotionModel::armorPosition(const StateVector& state, int armor_index) const
{
    const double angle = limitRad(state[6] + armor_index * kPi / 2.0);
    const bool use_offset = armor_index == 1 || armor_index == 3;
    const double radius = use_offset ? state[8] + state[9] : state[8];
    const double armor_x = state[0] - radius * std::cos(angle);
    const double armor_y = state[2] - radius * std::sin(angle);
    const double armor_z = use_offset ? state[4] + state[10] : state[4];
    return {armor_x, armor_y, armor_z};
}

Eigen::Matrix<double, 4, kStateDim> MotionModel::observationJacobian(
    const StateVector& state,
    int armor_index) const
{
    const double angle = limitRad(state[6] + armor_index * kPi / 2.0);
    const bool use_offset = armor_index == 1 || armor_index == 3;
    const double radius = use_offset ? state[8] + state[9] : state[8];

    const double dx_da = radius * std::sin(angle);
    const double dy_da = -radius * std::cos(angle);
    const double dx_dr = -std::cos(angle);
    const double dy_dr = -std::sin(angle);
    const double dx_do = use_offset ? -std::cos(angle) : 0.0;
    const double dy_do = use_offset ? -std::sin(angle) : 0.0;
    const double dz_dh = use_offset ? 1.0 : 0.0;

    Eigen::Matrix<double, 4, kStateDim> h_xyzyaw = Eigen::Matrix<double, 4, kStateDim>::Zero();
    h_xyzyaw(0, 0) = 1.0;
    h_xyzyaw(0, 6) = dx_da;
    h_xyzyaw(0, 8) = dx_dr;
    h_xyzyaw(0, 9) = dx_do;
    h_xyzyaw(1, 2) = 1.0;
    h_xyzyaw(1, 6) = dy_da;
    h_xyzyaw(1, 8) = dy_dr;
    h_xyzyaw(1, 9) = dy_do;
    h_xyzyaw(2, 4) = 1.0;
    h_xyzyaw(2, 10) = dz_dh;
    h_xyzyaw(3, 6) = 1.0;

    const Eigen::Vector3d xyz = armorPosition(state, armor_index);
    const Eigen::Matrix3d h_ypd = xyzToYpdJacobian(xyz);

    Eigen::Matrix4d h_ypdyaw = Eigen::Matrix4d::Zero();
    h_ypdyaw.block<3, 3>(0, 0) = h_ypd;
    h_ypdyaw(3, 3) = 1.0;
    return h_ypdyaw * h_xyzyaw;
}

Eigen::Vector4d MotionModel::observation(const StateVector& state, int armor_index) const
{
    const Eigen::Vector3d xyz = armorPosition(state, armor_index);
    const Eigen::Vector3d ypd = xyzToYpd(xyz);
    Eigen::Vector4d z;
    z << ypd.x(), ypd.y(), ypd.z(), limitRad(state[6] + armor_index * kPi / 2.0);
    return z;
}

int MotionModel::matchArmorIndex(const ArmorMeasurement& armor) const
{
    double best_error = std::numeric_limits<double>::max();
    int best_index = std::clamp(armor.armor_id, 0, kArmorCount - 1);
    for (int i = 0; i < kArmorCount; ++i) {
        const Eigen::Vector4d predicted = observation(x_, i);
        const double angle_error =
            std::abs(limitRad(armor.yaw - predicted[3])) +
            std::abs(limitRad(armor.ypd.x() - predicted[0])) +
            0.25 * std::abs(armor.ypd.z() - predicted[2]);
        double tracker_id_bonus = 0.0;
        if (armor.armor_id >= 0 && armor.armor_id < kArmorCount && armor.armor_id != i) {
            tracker_id_bonus = 0.15;
        }
        const double score = angle_error + tracker_id_bonus;
        if (score < best_error) {
            best_error = score;
            best_index = i;
        }
    }
    return best_index;
}

void MotionModel::ekfUpdate(
    const Eigen::Vector4d& z,
    const Eigen::Matrix<double, 4, kStateDim>& h,
    const Eigen::Matrix4d& r,
    int armor_index)
{
    const Eigen::Matrix4d s = h * p_ * h.transpose() + r;
    const Eigen::Matrix<double, kStateDim, 4> k = p_ * h.transpose() * s.inverse();
    Eigen::Vector4d innovation = z - observation(x_, armor_index);
    innovation[0] = limitRad(innovation[0]);
    innovation[1] = limitRad(innovation[1]);
    innovation[3] = limitRad(innovation[3]);

    x_ += k * innovation;
    x_[6] = limitRad(x_[6]);
    const StateMatrix identity = StateMatrix::Identity();
    p_ = (identity - k * h) * p_ * (identity - k * h).transpose() + k * r * k.transpose();
}

} // namespace predictor
