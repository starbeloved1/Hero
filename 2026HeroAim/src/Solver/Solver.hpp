#pragma once

#include "utils/include/EigenCompat.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

#include "utils/include/Location.hpp"
#include "Tracker/Type.hpp"
#include "utils/include/Params.hpp"
#include "Type.hpp"
#include "Estimator/include/Type.hpp"
#include "Driver/include/SerialPort.hpp"
#include "Driver/include/VideoCapture.h"

namespace solver {
    using driver::SerialPortData;
    using tracker::ArmorXYV;
    using lyutils::Camera8mmParam;

class Solver : public BaseSolver
{
private:
    Eigen::Matrix3d cameraIntrinsicMatrix;
    Eigen::Vector3d cameraOffset;
    Eigen::Vector5d distorationCoefficients;
    Eigen::Matrix3d cameraRotationMatrix;
    double f_x, f_y, c_x, c_y;

public:
    explicit Solver();

    // 相机输入切换时同步切换整套内参、畸变和云台到相机的外参。
    void setCameraProfile(driver::CameraProfile profile);

    inline PYD XYZ2PYD(const XYZ& in) const override
    {
        double distance = sqrt(in.x * in.x + in.y * in.y + in.z * in.z);
        double pitch = asin(in.z / distance);
        double yaw = atan2(in.y, in.x);
        return PYD(pitch, yaw, distance);
    }

    inline XYZ PYD2XYZ(const PYD& in) const override
    {
        XYZ out;
        out.x = in.distance * cos(in.pitch) * cos(in.yaw);
        out.y = in.distance * cos(in.pitch) * sin(in.yaw);
        out.z = in.distance * sin(in.pitch);
        return out;
    }

    inline XYZ CXYD2XYZ(const CXYD& in) const override
    {
        XYZ out;
        out.x = in.k;
        out.y = (c_x - in.cx) * in.k / f_x;
        out.z = (c_y - in.cy) * in.k / f_y;
        return out;
    }

    inline CXYD XYZ2CXYD(const XYZ& in) const override
    {
        double cx = c_x - in.y / in.x * f_x;
        double cy = c_y - in.z / in.x * f_y;
        CXYD out;
        out.cx = cx;
        out.cy = cy;
        out.k = in.x;
        return out;
    }

    inline XYZ camera2world(const XYZ& in, const PYD& imuData) const override
    {
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = cameraRotationMatrix.transpose() * in_eigen + cameraOffset;
        Eigen::Vector3d world = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                              * Eigen::AngleAxisd(-imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                              * gimbal;
        return XYZ(world(0), world(1), world(2));
    }

    inline XYZ world2camera(const XYZ& in, const PYD& imuData) const override
    {
        double imu_yaw = imuData.yaw;
        double imu_pitch = imuData.pitch;
        Eigen::Vector3d in_eigen = Eigen::Vector3d(in.x, in.y, in.z);
        Eigen::Vector3d gimbal = Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
                               * Eigen::AngleAxisd(-imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                               * in_eigen;
        Eigen::Vector3d camera = cameraRotationMatrix * (gimbal - cameraOffset);
        return XYZ(camera(0), camera(1), camera(2));
    }

    std::pair<XYZ, double> camera2world(const ArmorXYV& trackResult,
                                        const ImuData& imuData,
                                        bool isLarge);

    void solveAndFilterTrackResults(tracker::TrackResults& results,
                                    const tracker::CarTrackResults& carResults,
                                    const ImuData& imu);

    void solveArmorPose(estimator::Armor& armor, const driver::SerialPortData& imu_data);
    void PreProcessArmor(estimator::Armors& armors, const driver::SerialPortData& imu_data);
    cv::Point2f reproject(cv::Point3f world_point_flu);
    std::vector<cv::Point2f> reprojectArmorCorners(const estimator::Armor& armor);

    struct PnPResult {
        cv::Mat rvec;
        cv::Mat tvec;
        double armor_yaw = 0.0;
        XYZ camera_flu;
        XYZ world_flu;
        bool success = false;
    };

private:
    PnPResult solvePnP(const std::vector<cv::Point2f>& imagePoints, bool isLarge);
    PnPResult solveArmor(const std::vector<cv::Point2f>& imagePoints,
                         const ImuData& imuData_deg,
                         bool isLarge);

    cv::Mat cv_camera_matrix_;
    cv::Mat cv_dist_coeffs_;
    Eigen::Matrix3d last_imu_rotation_ = Eigen::Matrix3d::Identity();
    driver::CameraProfile active_camera_profile_ = driver::CameraProfile::Base;
};

std::shared_ptr<Solver> createSolver();

} // namespace solver
