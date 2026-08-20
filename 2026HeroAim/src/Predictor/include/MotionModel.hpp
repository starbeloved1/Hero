#pragma once

#include "type.hpp"
#include "utils/include/EigenCompat.hpp"
#include "utils/include/Location.hpp"
#include "utils/include/TimeStamp.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <vector>

namespace predictor {

constexpr int kStateDim = 11;
using StateVector = Eigen::Matrix<double, kStateDim, 1>;
using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

struct ArmorMeasurement {
    int car_id = -1;
    int armor_id = -1;
    XYZ xyz;
    double yaw = 0.0;
    Eigen::Vector3d ypd = Eigen::Vector3d::Zero();
};

class MotionModel {
public:
    MotionModel() = default;
    void initialize(const ArmorMeasurement& armor, const Time::TimeStamp& timestamp);
    void predict(const Time::TimeStamp& timestamp);
    void update(const ArmorMeasurement& armor);
    void markLost();

    [[nodiscard]] bool active() const;
    [[nodiscard]] bool Stable() const;
    [[nodiscard]] bool diverged() const;
    [[nodiscard]] int lostCount() const;
    [[nodiscard]] StateVector predictState(const Time::TimeStamp& timestamp) const;
    [[nodiscard]] std::array<Armor, 4> predictedArmors(const StateVector& state) const;
    [[nodiscard]] Prediction toPrediction(int car_id, const Time::TimeStamp& timestamp) const;

private:
    [[nodiscard]] static double limitRad(double angle);
    [[nodiscard]] static Eigen::Vector3d xyzToYpd(const Eigen::Vector3d& xyz);
    [[nodiscard]] static Eigen::Matrix3d xyzToYpdJacobian(const Eigen::Vector3d& xyz);
    [[nodiscard]] static bool finiteMeasurement(const ArmorMeasurement& armor);

    [[nodiscard]] StateVector transition(const StateVector& state, double dt) const;
    [[nodiscard]] StateMatrix transitionMatrix(double dt) const;
    [[nodiscard]] StateMatrix processNoise(double dt, const StateVector& state) const;
    [[nodiscard]] Eigen::Vector3d armorPosition(const StateVector& state, int armor_index) const;
    [[nodiscard]] Eigen::Matrix<double, 4, kStateDim> observationJacobian(
        const StateVector& state,
        int armor_index) const;
    [[nodiscard]] Eigen::Vector4d observation(const StateVector& state, int armor_index) const;
    [[nodiscard]] int matchArmorIndex(const ArmorMeasurement& armor) const;

    void ekfUpdate(
        const Eigen::Vector4d& z,
        const Eigen::Matrix<double, 4, kStateDim>& h,
        const Eigen::Matrix4d& r,
        int armor_index);

    bool initialized_ = false;
    int lost_count_ = 0;
    int update_count_ = 0;
    Time::TimeStamp last_timestamp_ = Time::TimeStamp::now();
    StateVector x_ = StateVector::Zero();
    StateMatrix p_ = StateMatrix::Identity();
};

} // namespace predictor
