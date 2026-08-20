#include "Solver/Solver.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <PoseLib/poselib.h>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>
#include <array>
#include <cstddef>
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp" // 读取相机内参

auto solver::createSolver() -> std::shared_ptr<Solver> {
    return std::make_unique<solver::Solver>();
}

namespace solver {

namespace {

struct CameraCalibration {
    double fx, fy, u0, v0;
    double k1, k2, k3, p1, p2;
    double trans_x, trans_y, trans_z;
    double yaw, pitch, roll;
};

template <typename CameraParamT>
CameraCalibration makeCalibration() {
    return {
        CameraParamT::fx, CameraParamT::fy, CameraParamT::u0, CameraParamT::v0,
        CameraParamT::k1, CameraParamT::k2, CameraParamT::k3,
        CameraParamT::p1, CameraParamT::p2,
        CameraParamT::trans_x, CameraParamT::trans_y, CameraParamT::trans_z,
        CameraParamT::yaw, CameraParamT::pitch, CameraParamT::roll,
    };
}

}  // namespace

Solver::Solver() {
    setCameraProfile(driver::CameraProfile::Aim8mm);
}

void Solver::setCameraProfile(driver::CameraProfile profile) {
    // Base 当前只运行 AntiBase 图像链路，不进行 PnP/弹道解算；保留当前标定。
    if (profile == driver::CameraProfile::Base || profile == active_camera_profile_) {
        return;
    }

    const CameraCalibration calibration = profile == driver::CameraProfile::Aim8mm2
        ? makeCalibration<lyutils::Camera8mm2Param>()
        : makeCalibration<Camera8mmParam>();

    // 1. 加载相机内参 (Intrinsic)
    cameraIntrinsicMatrix << calibration.fx, 0, calibration.u0,
                             0, calibration.fy, calibration.v0,
                             0, 0, 1;
    f_x = cameraIntrinsicMatrix(0, 0);
    f_y = cameraIntrinsicMatrix(1, 1);
    c_x = cameraIntrinsicMatrix(0, 2);
    c_y = cameraIntrinsicMatrix(1, 2);
    // 2. 加载畸变系数 (Distortion)
    distorationCoefficients << calibration.k1, calibration.k2,
                               calibration.p1, calibration.p2,
                               calibration.k3;
    // 3. 加载相机平移外参 (Translation)
    cameraOffset << calibration.trans_x,       // FLU X 前后
                    calibration.trans_y,       // FLU Y 左右
                    calibration.trans_z;       // FLU Z 上下
    // 4. 加载相机旋转外参 (Rotation)
    const double r_yaw = calibration.yaw * CV_PI / 180.0;
    const double r_pitch = calibration.pitch * CV_PI / 180.0;
    const double r_roll = calibration.roll * CV_PI / 180.0;
    // 使用 yaw/pitch/roll 构造相机外参旋转矩阵。
    // Build camera extrinsic rotation from yaw/pitch/roll.
    Eigen::Matrix3d rotationMatrix_Eigen;
    rotationMatrix_Eigen = Eigen::AngleAxisd(r_yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(r_pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(r_roll, Eigen::Vector3d::UnitX());
    
    cameraRotationMatrix = rotationMatrix_Eigen;
    // 预计算 OpenCV 格式的相机参数，供 solveArmorPose/reproject 使用。
    // Cache OpenCV camera parameters for pose solving and reprojection.
    cv::eigen2cv(cameraIntrinsicMatrix, cv_camera_matrix_);
    cv::eigen2cv(distorationCoefficients, cv_dist_coeffs_);
    active_camera_profile_ = profile;
    LOG(INFO) << "[Solver] Switched calibration to "
              << (profile == driver::CameraProfile::Aim8mm2 ? "camera_8mm_2" : "camera_8mm")
              << " fx=" << calibration.fx << " fy=" << calibration.fy
              << " yaw/pitch/roll=" << calibration.yaw << "/"
              << calibration.pitch << "/" << calibration.roll;
}

    inline constexpr auto SAHW = 0.0675f;   // 小装甲板半宽 (m)
    inline constexpr auto SAHH = 0.028f;    // 小装甲板半高 (m)，灯条长度 56mm
	const std::vector SmallArmorPoints =
	{
		cv::Point3f(-SAHW, SAHH, 0.0f),
		cv::Point3f(SAHW, SAHH, 0.0f),
		cv::Point3f(SAHW, -SAHH, 0.0f),
		cv::Point3f(-SAHW, -SAHH, 0.0f)
	};
    inline constexpr auto LAHW = 0.115f;    // 大装甲板半宽 (m)
    inline constexpr auto LAHH = 0.028f;    // 大装甲板半高 (m)，灯条长度 56mm
	const std::vector LargeArmorPoints =
	{
		cv::Point3f(-LAHW, LAHH, 0.0f),
		cv::Point3f(LAHW, LAHH, 0.0f),
		cv::Point3f(LAHW, -LAHH, 0.0f),
		cv::Point3f(-LAHW, -LAHH, 0.0f)
	};

inline double normalizeAngle(double angle) {
    return std::remainder(angle, 2 * M_PI);
}

inline bool makePoseLibCamera(const cv::Mat& camera_matrix,
                              const cv::Mat& distortion_coefficients,
                              poselib::Camera& camera) {
    if (camera_matrix.empty() || camera_matrix.rows < 3 || camera_matrix.cols < 3) {
        return false;
    }

    cv::Mat camera_matrix_64f;
    camera_matrix.convertTo(camera_matrix_64f, CV_64F);
    const double fx = camera_matrix_64f.at<double>(0, 0);
    const double fy = camera_matrix_64f.at<double>(1, 1);
    const double cx = camera_matrix_64f.at<double>(0, 2);
    const double cy = camera_matrix_64f.at<double>(1, 2);
    if (fx <= 0.0 || fy <= 0.0 ||
        !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(cx) || !std::isfinite(cy)) {
        return false;
    }

    std::array<double, 8> distortion{};
    if (!distortion_coefficients.empty()) {
        cv::Mat distortion_64f;
        distortion_coefficients.convertTo(distortion_64f, CV_64F);
        distortion_64f = distortion_64f.reshape(1, 1);
        const int count = std::min<int>(distortion_64f.cols, static_cast<int>(distortion.size()));
        for (int i = 0; i < count; ++i) {
            distortion[static_cast<std::size_t>(i)] = distortion_64f.at<double>(0, i);
        }
    }

    camera = poselib::Camera(poselib::FULL_OPENCV,
                             {fx, fy, cx, cy,
                              distortion[0], distortion[1], distortion[2], distortion[3],
                              distortion[4], distortion[5], distortion[6], distortion[7]});
    return true;
}


inline bool isFinitePose(const cv::Mat& tvec) {
    return !tvec.empty() && tvec.rows >= 3 &&
           std::isfinite(tvec.at<double>(0)) &&
           std::isfinite(tvec.at<double>(1)) &&
           std::isfinite(tvec.at<double>(2));
}

// ============================================================
// Layer 1: PoseLib PnPL pose solving with RANSAC and bundle refinement.
// 保持 Hero 物体坐标轴定义：装甲板点位于 XY 平面，+Z 为法向。
// ============================================================
Solver::PnPResult Solver::solvePnP(const std::vector<cv::Point2f>& imagePoints, bool isLarge) {
    PnPResult result;
    if (imagePoints.size() != 4) return result;

    std::vector<cv::Point3f> objectPoints = isLarge ? LargeArmorPoints : SmallArmorPoints;

    poselib::Camera camera;
    if (!makePoseLibCamera(cv_camera_matrix_, cv_dist_coeffs_, camera)) {
        return result;
    }

    std::vector<poselib::Point2D> points2d;
    std::vector<poselib::Point3D> points3d;
    points2d.reserve(imagePoints.size());
    points3d.reserve(objectPoints.size());
    for (std::size_t i = 0; i < imagePoints.size(); ++i) {
        points2d.emplace_back(imagePoints[i].x, imagePoints[i].y);
        points3d.emplace_back(objectPoints[i].x, objectPoints[i].y, objectPoints[i].z);
    }

    constexpr std::array<std::array<int, 2>, 4> kEdges{{
        {{0, 1}},
        {{1, 2}},
        {{2, 3}},
        {{3, 0}},
    }};

    std::vector<poselib::Line2D> lines2d;
    std::vector<poselib::Line3D> lines3d;
    lines2d.reserve(kEdges.size());
    lines3d.reserve(kEdges.size());
    for (const auto& edge : kEdges) {
        const auto first = static_cast<std::size_t>(edge[0]);
        const auto second = static_cast<std::size_t>(edge[1]);
        lines2d.emplace_back(points2d[first], points2d[second]);
        lines3d.emplace_back(points3d[first], points3d[second]);
    }

    poselib::AbsolutePoseOptions options;
    options.max_error = 8.0;
    options.max_errors = {8.0, 4.0};
    options.ransac.min_iterations = 64;
    options.ransac.max_iterations = 512;
    options.bundle.loss_scale = options.max_error;

    poselib::CameraPose pose;
    std::vector<char> point_inliers;
    std::vector<char> line_inliers;
    const poselib::RansacStats stats = poselib::estimate_absolute_pose_pnpl(
        points2d, points3d, lines2d, lines3d, camera, options,
        &pose, &point_inliers, &line_inliers);
    if (stats.num_inliers == 0U) {
        return result;
    }

    const auto R = pose.R();
    cv::Mat rotMat = (cv::Mat_<double>(3, 3) <<
        R(0, 0), R(0, 1), R(0, 2),
        R(1, 0), R(1, 1), R(1, 2),
        R(2, 0), R(2, 1), R(2, 2));
    result.tvec = (cv::Mat_<double>(3, 1) << pose.t.x(), pose.t.y(), pose.t.z());

    if (!isFinitePose(result.tvec) || result.tvec.at<double>(2) <= 0.0) {
        return result;
    }

    cv::Rodrigues(rotMat, result.rvec);
    result.armor_yaw = normalizeAngle(atan2(rotMat.at<double>(0, 2), rotMat.at<double>(2, 2)));
    result.camera_flu = XYZ(result.tvec.at<double>(2), -result.tvec.at<double>(0), -result.tvec.at<double>(1));

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(objectPoints, result.rvec, result.tvec, cv_camera_matrix_, cv_dist_coeffs_, reprojected);
    double reprojection_error = 0.0;
    for (std::size_t i = 0; i < imagePoints.size(); ++i) {
        reprojection_error += cv::norm(reprojected[i] - imagePoints[i]);
    }
    reprojection_error /= static_cast<double>(imagePoints.size());
    LOG(INFO) << "[PoseLibPnPL] inliers=" << stats.num_inliers
              << " reprojection_error=" << reprojection_error;

    result.success = true;
    return result;
}
// ============================================================
// Layer 2: convert PnPL result to camera FLU, then world FLU.
// ============================================================
Solver::PnPResult Solver::solveArmor(const std::vector<cv::Point2f>& imagePoints,
                                      const ImuData& imuData_deg, bool isLarge) {
    auto result = solvePnP(imagePoints, isLarge);
    if (!result.success) return result;

    result.world_flu = camera2world(result.camera_flu, static_cast<PYD>(imuData_deg));
    // 缓存 IMU 旋转矩阵，供 reproject 使用。
    // Cache IMU rotation for reprojection.
    double imu_yaw_rad = imuData_deg.yaw * M_PI / 180.0;
    double imu_pitch_rad = imuData_deg.pitch * M_PI / 180.0;
    last_imu_rotation_ = Eigen::AngleAxisd(imu_yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                        * Eigen::AngleAxisd(-imu_pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();

    return result;
}

// Mode 3 入口：tracker 装甲板像素角点 -> 世界坐标 + 装甲板世界 yaw
std::pair<XYZ,double> Solver::camera2world(const ArmorXYV& trackResult, const ImuData& imuData_deg, bool isLarge) {
    if (trackResult.size() != 4)
        return {XYZ(), 0.0};

    std::vector<cv::Point2f> imagePoints;
    for (const auto& xyv : trackResult)
        imagePoints.emplace_back(xyv.x, xyv.y);

    auto pnp = solveArmor(imagePoints, imuData_deg, isLarge);
    if (!pnp.success)
        return {XYZ(), 0.0};

    double imu_yaw = imuData_deg.yaw * M_PI / 180;
    return {pnp.world_flu, pnp.armor_yaw + imu_yaw};
}

// Mode 1/2 入口：Estimator Armor -> PnPL 解算 + 世界坐标
void Solver::solveArmorPose(estimator::Armor& armor, const driver::SerialPortData& imu_data) {
    bool isLarge = (armor._class == 1);
    armor.is_big_armor = isLarge;
    // 串口 IMU 数据单位是度，后续通过 PYD 转换为弧度。
    // Serial IMU data is in degrees; PYD conversion turns it into radians later.
    ImuData imuData_deg(imu_data.pitch, imu_data.yaw, 0.0f);
    auto pnp = solveArmor(armor.corners, imuData_deg, isLarge);
    if (!pnp.success) {
        armor.x = armor.y = armor.z = 0;
        return;
    }

    armor.rvec = pnp.rvec;
    armor.tvec = pnp.tvec;
    armor.angle = std::abs(pnp.armor_yaw * 180.0 / M_PI);
    armor.x = pnp.world_flu.x;
    armor.y = pnp.world_flu.y;
    armor.z = pnp.world_flu.z;
}

void Solver::PreProcessArmor(estimator::Armors& armors, const driver::SerialPortData& imu_data) {
    // 1. 先对所有装甲板进行 PnPL 位姿解算。
    // Solve every armor pose first.
    for (estimator::Armor& a : armors) {
        solveArmorPose(a, imu_data);
        a.reprojected_corners = reprojectArmorCorners(a);
    }
    // 解算后每个类别只保留图像宽度最大的装甲板。
    // Keep the widest armor for each class after solving.
    std::map<int, size_t> best_idx_per_class;
    std::map<int, double> best_width_per_class;
    constexpr double OUTPOST_MAX_ASPECT_RATIO = 3.0;
    
    for (size_t i = 0; i < armors.size(); ++i) {
        const auto& a = armors[i];
        // 计算图像宽度。
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        for (const auto& pt : a.corners) {
            min_x = std::min(min_x, static_cast<double>(pt.x));
            max_x = std::max(max_x, static_cast<double>(pt.x));
        }
        double img_width = max_x - min_x;

        if (a._class == 7 && a.corners.size() == 4) {
            const double top_width = cv::norm(a.corners[0] - a.corners[1]);
            const double bottom_width = cv::norm(a.corners[3] - a.corners[2]);
            const double left_height = cv::norm(a.corners[0] - a.corners[3]);
            const double right_height = cv::norm(a.corners[1] - a.corners[2]);
            const double armor_width = 0.5 * (top_width + bottom_width);
            const double armor_height = 0.5 * (left_height + right_height);
            const double aspect_ratio = armor_width / std::max(1.0, armor_height);

            if (aspect_ratio > OUTPOST_MAX_ASPECT_RATIO) {
                LOG(WARNING) << "[PreProcess] skip class7 flat armor";
                continue;
            }
        }
        
        if (best_idx_per_class.find(a._class) == best_idx_per_class.end()) {
            best_idx_per_class[a._class] = i;
            best_width_per_class[a._class] = img_width;
        } else if (img_width > best_width_per_class[a._class]) {
            best_idx_per_class[a._class] = i;
            best_width_per_class[a._class] = img_width;
        }
    }
    estimator::Armors filtered;
    for (const auto& [cls, idx] : best_idx_per_class) {
        //LOG(INFO) << "[PreProcess] >>> Selected: class=" << cls;
        filtered.push_back(armors[idx]);
    }
    for (const estimator::Armor& a : filtered) {
        LOG(INFO) << "[World] class=" << a._class
                  << " x:" << a.x
                  << ",y:" << a.y
                  << ",z:" << a.z
                  << ",dist:" << std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    }
    armors = std::move(filtered);
}

void Solver::solveAndFilterTrackResults(tracker::TrackResults& results, 
                                        const tracker::CarTrackResults& carResults,
                                        const ImuData& imu) {
    (void)carResults;
    if (results.empty()) return;

    for (auto& tr : results) {
        // Hero uses the large armor model.
        bool isLarge = (tr.car_id == 1);
        auto [xyz_imu, yaw] = camera2world(tr.armor, imu, isLarge);
        tr.location.imu = imu;
        tr.location.xyz_imu = xyz_imu;
        tr.yaw = yaw;
    }

    results.erase(
        std::remove_if(results.begin(), results.end(), [](const auto& tr) {
            XYZ xyz = static_cast<XYZ>(tr.location.xyz_imu);
            double dist = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
            return tr.armor_id < 0 || tr.armor_id > 3 ||
                   !std::isfinite(xyz.x) || !std::isfinite(xyz.y) ||
                   !std::isfinite(xyz.z) || !std::isfinite(tr.yaw) ||
                   dist < 0.1;
        }),
        results.end());
    if (results.empty()) return;

    for (const auto& tr : results) {
        XYZ xyz = static_cast<XYZ>(tr.location.xyz_imu);
        LOG(INFO) << "[Solver] car=" << tr.car_id << " armor_id=" << tr.armor_id
                  << " xyz=(" << xyz.x << "," << xyz.y << "," << xyz.z << ")";
    }
}

cv::Point2f Solver::reproject(cv::Point3f world_point_flu) {
    // 输入已经是 FLU 世界坐标，无需额外转换。
    Eigen::Vector3d world_flu(world_point_flu.x, world_point_flu.y, world_point_flu.z);

    // FLU 世界 -> FLU 云台：使用 IMU 旋转的逆变换。
    Eigen::Vector3d gimbal = last_imu_rotation_.transpose() * world_flu;

    // FLU 云台 -> FLU 相机：camera2world 的逆变换。
    Eigen::Vector3d camera_flu = cameraRotationMatrix * (gimbal - cameraOffset);

    // FLU 相机 -> OpenCV 相机。
    Eigen::Vector3d camera_cv(-camera_flu.y(), -camera_flu.z(), camera_flu.x());

    std::vector<cv::Point3f> pts3d = {cv::Point3f(
        static_cast<float>(camera_cv.x()),
        static_cast<float>(camera_cv.y()),
        static_cast<float>(camera_cv.z()))};
    cv::Mat rvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec_zero = cv::Mat::zeros(3, 1, CV_64F);
    std::vector<cv::Point2f> pts2d;
    cv::projectPoints(pts3d, rvec_zero, tvec_zero, cv_camera_matrix_, cv_dist_coeffs_, pts2d);

    return pts2d[0];
}

std::vector<cv::Point2f> Solver::reprojectArmorCorners(const estimator::Armor& armor) {
    std::vector<cv::Point3f> objectPoints = armor.is_big_armor ?
        std::vector<cv::Point3f>(LargeArmorPoints.begin(), LargeArmorPoints.end()) :
        std::vector<cv::Point3f>(SmallArmorPoints.begin(), SmallArmorPoints.end());

    std::vector<cv::Point2f> reprojected;
    if (!armor.rvec.empty() && !armor.tvec.empty()) {
        cv::projectPoints(objectPoints, armor.rvec, armor.tvec, cv_camera_matrix_, cv_dist_coeffs_, reprojected);
    }
    return reprojected;
}

} // namespace solver
