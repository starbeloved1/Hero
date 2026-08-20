#include "../include/Visualizer.hpp"
#include <cmath>
#include <algorithm>
#include <set>

namespace lyutils {

using namespace tracker;
using namespace location;
using namespace predictor;

void Visualizer::drawAll(cv::Mat &image, 
                         const std::vector<TrackResult>& track_armors,
                         const std::vector<CarTrackResult>& track_cars,
                         const std::vector<Prediction>& predictions,
                         const PYD& imu,
                         int target_car_id,
                         int target_armor_id) 
{
    // 按图层顺序绘制：预测在底层，跟踪在上层（跟踪更准确）
    drawPredictions(image, predictions, imu, target_car_id, target_armor_id);
    drawTrackedCars(image, track_cars);            // 2. 车辆
    drawTrackedArmors(image, track_armors, imu);   // 3. 跟踪装甲板（上层，实线）
    drawOverlays(image);                           // 4. 固定参考线
}

void Visualizer::drawTrackedArmors(cv::Mat &image, 
                                   const std::vector<TrackResult>& armors, 
                                   const PYD& imu) 
{
    for (const auto& result : armors) {
        // 1. 绘制装甲板中心 - 青色圆圈（跟踪结果，上层）
        CXYD coord = result.location.cxy;
        cv::circle(image, cv::Point(coord.cx, coord.cy), 12, cv::Scalar(255, 255, 0), -1);
        cv::circle(image, cv::Point(coord.cx, coord.cy), 14, cv::Scalar(0, 0, 0), 2); // 黑色边框增强对比

        // 2. 绘制 ID 文字 - 左上方，红色加粗
        std::string text_id = "T" + std::to_string(result.armor_id); // T表示Track
        cv::putText(image, text_id, cv::Point(coord.cx - 40, coord.cy - 25), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);

        // 5. 绘制俯视图 - 红色较大圆点（跟踪结果）
        XYZ armor_center = result.location.xyz_imu;
        cv::circle(image, cv::Point2f(500 - armor_center.y * 100.0, 500 - armor_center.x * 100.0), 6,
                   cv::Scalar(0, 0, 255), -1);
        
        // 绘制俯视图方向线
        double k = tan(result.yaw);
        double dx = 10 / sqrt(1 + k * k);
        double dy = k * dx;
        cv::line(image,
                 cv::Point2f(500 - armor_center.y * 100.0 + dx, 500 - armor_center.x * 100.0 - dy),
                 cv::Point2f(500 - armor_center.y * 100.0 - dx, 500 - armor_center.x * 100.0 + dy),
                 cv::Scalar(0, 0, 255), 2);
    }
}

void Visualizer::drawTrackedCars(cv::Mat &image, const std::vector<CarTrackResult>& cars) {
    for (const auto& car : cars) {
        cv::rectangle(image, car.bounding_rect, cv::Scalar(255, 0, 0), 5);
        // 车辆ID - 红色
        cv::putText(image, std::to_string(car.car_id), car.bounding_rect.tl() + cv::Point2f(5, -5), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
    }
}

void Visualizer::drawPredictions(cv::Mat &image, 
                                 const std::vector<Prediction>& predictions, 
                                 const PYD& imu,
                                 int target_car_id,
                                 int target_armor_id) 
{
    Location temp;
    temp.imu = imu; // 设置当前 IMU 用于投影

    for (const auto &prediction : predictions) {
        XYZ center = prediction.center;
        
        // 1. 绘制车辆中心 - 紫色大圆圈（明显标识）
        temp.xyz_imu = center;
        CXYD coord = temp.cxy;
        cv::circle(image, cv::Point(coord.cx, coord.cy), 15, cv::Scalar(255, 0, 255), 3); // 紫色空心圆
        cv::circle(image, cv::Point(coord.cx, coord.cy), 5, cv::Scalar(255, 0, 255), -1); // 紫色实心点
        
        // 车辆中心文字标识
        std::string center_text = "CAR_" + std::to_string(prediction.id);
        cv::putText(image, center_text, cv::Point(coord.cx - 50, coord.cy - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
        
        // 俯视图中的车辆中心 - 紫色较大圆点
        cv::circle(image, cv::Point2f(500 - center.y * 100.0, 500 - center.x * 100.0), 8,
                   cv::Scalar(255, 0, 255), -1);

        // 2. 绘制预测的四个装甲板（所有状态都显示）
        // 先计算所有装甲板的朝向角绝对值，找出最小的两个
        std::vector<std::pair<double, int>> angle_idx_pairs;
        double see_angle = std::atan2(center.y, center.x);
        for (size_t i = 0; i < prediction.armors.size(); ++i) {
            double armor_angle = std::remainder(prediction.armors[i].theta + M_PI / 2 - see_angle, 2.0 * M_PI);
            angle_idx_pairs.emplace_back(std::abs(armor_angle), static_cast<int>(i));
        }
        std::sort(angle_idx_pairs.begin(), angle_idx_pairs.end());
        std::set<int> top2_indices;
        for (size_t i = 0; i < std::min(size_t(2), angle_idx_pairs.size()); ++i) {
            top2_indices.insert(angle_idx_pairs[i].second);
        }

        for (size_t armor_idx = 0; armor_idx < prediction.armors.size(); ++armor_idx) {
            const auto &armor = prediction.armors[armor_idx];
            auto armor_center = armor.center;
            temp.xyz_imu = armor_center;
            CXYD armor_coord = temp.cxy;
            
            // 检查投影点是否在图像范围内（加一些余量）
            if (armor_coord.cx < -100 || armor_coord.cx > image.cols + 100 ||
                armor_coord.cy < -100 || armor_coord.cy > image.rows + 100) {
                continue; // 跳过图像外的点
            }

            // 判断是否为 Controller 选中的目标装甲板
            bool is_selected = (prediction.id == target_car_id && armor.id == target_armor_id);

            // 根据状态选择颜色：选中=红色，AVAILABLE=绿色，UNSEEN=橙色
            cv::Scalar color;
            if (is_selected)
                color = cv::Scalar(0, 0, 255); // 红色 - 选中目标
            else if (armor.status == armor.AVAILABLE)
                color = cv::Scalar(100, 255, 100);
            else
                color = cv::Scalar(0, 165, 255);

            cv::circle(image, cv::Point(armor_coord.cx, armor_coord.cy), 10, color, 2);

            // 选中的装甲板：连线到车辆中心
            if (is_selected) {
                int cx = armor_coord.cx, cy = armor_coord.cy;

                // 从车辆中心 → 选中装甲板 画红色连线
                cv::line(image, cv::Point(coord.cx, coord.cy), cv::Point(cx, cy),
                         cv::Scalar(0, 0, 255), 2);

                // 俯视图中也高亮：车辆中心→装甲板 红色连线
                cv::Point2f bev_car(500 - center.y * 100.0, 500 - center.x * 100.0);
                cv::Point2f bev_armor(500 - armor.center.y * 100.0, 500 - armor.center.x * 100.0);
                cv::line(image, bev_car, bev_armor, cv::Scalar(0, 0, 255), 2);
                cv::circle(image, bev_armor, 7, cv::Scalar(0, 0, 255), -1);
            }

            // ID在左侧，颜色跟圆圈一致
            std::string text_id = "P" + std::to_string(armor.id);
            cv::putText(image, text_id, cv::Point(armor_coord.cx - 45, armor_coord.cy - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
            
            // 只显示朝向角最小的两个装甲板的角度（背向的不显示）
            if (top2_indices.count(static_cast<int>(armor_idx))) {
                double armor_angle = std::remainder(armor.theta + M_PI / 2 - see_angle, 2.0 * M_PI);
                double armor_angle_deg = armor_angle * 180.0 / M_PI;
                
                char pred_buf[32];
                std::snprintf(pred_buf, sizeof(pred_buf), "%.1f", armor_angle_deg);
                cv::putText(image, pred_buf, cv::Point(armor_coord.cx + 20, armor_coord.cy + 25), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }

            // 绘制预测俯视图 - 灰色小圆（预测结果）
            cv::circle(image, cv::Point2f(500 - armor.center.y * 100.0, 500 - armor.center.x * 100.0), 4,
                       cv::Scalar(200, 200, 200), -1);
            // 不绘制朝向指示线，避免视觉干扰
        }
    }
}

void Visualizer::drawOverlays(cv::Mat &image) {
    // 绘制屏幕中心十字
    int center_x = image.cols / 2;
    int center_y = image.rows / 2;
    cv::line(image, cv::Point(center_x - 10, center_y), cv::Point(center_x + 10, center_y), cv::Scalar(0, 255, 0), 4);
    cv::line(image, cv::Point(center_x, center_y - 10), cv::Point(center_x, center_y + 10), cv::Scalar(0, 255, 0), 4);
}

void Visualizer::drawControlInfo(cv::Mat &image, const controller::ControlResult& result) {
    // ========== 左上角：速度和加速度 ==========
    float speed = std::sqrt(result.vx * result.vx + result.vy * result.vy);
    float accel = std::sqrt(result.ax * result.ax + result.ay * result.ay);
    
    // 速度颜色：正常绿色
    cv::Scalar speed_color(0, 255, 0);
    
    // 加速度颜色：超过阈值显示红色，否则绿色
    cv::Scalar accel_color = (accel > result.accel_threshold) ? 
                             cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
    
    char buf[64];
    int y_offset = 30;
    
    // 速度
    std::snprintf(buf, sizeof(buf), "Speed: %.2f m/s", speed);
    cv::putText(image, buf, cv::Point(10, y_offset), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, speed_color, 2);
    
    // 加速度
    y_offset += 35;
    std::snprintf(buf, sizeof(buf), "Accel: %.2f m/s2", accel);
    cv::putText(image, buf, cv::Point(10, y_offset), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, accel_color, 2);
    
    // 高加速度模式标志
    if (result.high_accel_mode) {
        y_offset += 35;
        cv::putText(image, "HIGH ACCEL!", cv::Point(10, y_offset), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }
    
    // ========== 右上角：朝向角、对准帧数、开火 ==========
    int right_x = image.cols - 250;
    y_offset = 30;
    
    // 目标装甲板朝向角
    std::snprintf(buf, sizeof(buf), "Angle: %.1f deg", result.armor_angle_deg);
    cv::putText(image, buf, cv::Point(right_x, y_offset), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
    
    // 对准帧数（进度条形式）
    y_offset += 35;
    std::snprintf(buf, sizeof(buf), "Ready: %d/%d", result.phase_ready_cnt, result.phase_confirm_frames);
    cv::Scalar ready_color = (result.phase_ready_cnt >= result.phase_confirm_frames) ? 
                             cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 0);
    cv::putText(image, buf, cv::Point(right_x, y_offset), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, ready_color, 2);
    
    // 开火提示
    if (result.shoot_flag) {
        y_offset += 45;
        cv::putText(image, "FIRE!", cv::Point(right_x + 40, y_offset), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
        // 绘制红色边框强调开火
        cv::rectangle(image, cv::Point(right_x + 30, y_offset - 35), 
                      cv::Point(right_x + 180, y_offset + 10), 
                      cv::Scalar(0, 0, 255), 3);
    }
}

} // namespace lyutils