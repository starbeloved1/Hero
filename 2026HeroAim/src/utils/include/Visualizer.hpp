#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Tracker/Type.hpp"
#include "Tracker/Tracker.hpp"       // 引用 Tracker 结果定义
#include "Predictor/include/predictor.hpp" // 引用 Prediction 定义
#include "utils/include/Location.hpp" // 引用 Location 和 PYD 定义
#include "Controller/Type.hpp"        // 引用 ControlResult 定义

namespace lyutils {

class Visualizer {
public:
    /**
     * @brief 统一绘制函数，在主循环中调用这一次即可
     * @param image 图像引用
     * @param track_armors 跟踪到的装甲板列表
     * @param track_cars 跟踪到的车辆列表
     * @param predictions 预测结果列表
     * @param imu 当前的 IMU 数据 (用于坐标反解算)
     */
    static void drawAll(cv::Mat &image, 
                        const std::vector<tracker::TrackResult>& track_armors,
                        const std::vector<tracker::CarTrackResult>& track_cars,
                        const std::vector<predictor::Prediction>& predictions,
                        const PYD& imu,
                        int target_car_id = -1,
                        int target_armor_id = -1);
    
    /**
     * @brief 绘制控制信息：左上角显示速度/加速度，右上角显示朝向角/对准帧数/开火
     * @param image 图像引用
     * @param result Controller的控制结果
     */
    static void drawControlInfo(cv::Mat &image, const controller::ControlResult& result);

private:
    // 绘制跟踪到的装甲板（包括ID、Yaw、以及映射到底盘的连线）
    static void drawTrackedArmors(cv::Mat &image, 
                                  const std::vector<tracker::TrackResult>& armors, 
                                  const PYD& imu);

    // 绘制车辆边框和类型
    static void drawTrackedCars(cv::Mat &image, 
                                const std::vector<tracker::CarTrackResult>& cars);

    // 绘制预测结果（预测轨迹、装甲板位置）
    static void drawPredictions(cv::Mat &image, 
                                const std::vector<predictor::Prediction>& predictions, 
                                const PYD& imu,
                                int target_car_id = -1,
                                int target_armor_id = -1);

    // 绘制屏幕中心十字和其他固定调试信息
    static void drawOverlays(cv::Mat &image);
};

} // namespace lyutils