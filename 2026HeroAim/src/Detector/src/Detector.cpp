#include "Detector/include/Detector.hpp"
#include "utils/include/Params.hpp" 
#include "utils/include/Log.hpp"   
#include "Estimator/include/Type.hpp"  // for estimator::Armor
#include "Estimator/include/AntiTop.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <unordered_map> 
#include <deque>
#include <algorithm>
#include <cmath>

using namespace cv;
using namespace lyutils; 
using namespace driver;

namespace {

void updateArmorGeometry(detector::BBox& det) {
    det.center = (det.corners[0] + det.corners[1] + det.corners[2] + det.corners[3]) / 4.0f;
    std::vector<cv::Point2f> tmp(det.corners, det.corners + 4);
    det.rect = cv::boundingRect(tmp);
    det.area = static_cast<float>(det.rect.area());
}

} // namespace

namespace detector
{
    Detector::Detector(const std::string& armor_model, const std::string& yolo_model)
        : stream_enabled_(lyutils::GlobalParam::STREAM_ENABLE)
    {
        armor_infer = std::make_unique<ArmorOneStage>(armor_model);
        yolo_infer = std::make_unique<YoloDetector>(yolo_model);


        if (stream_enabled_) {
            std::string ip = lyutils::GlobalParam::IS_NUC ? "0.0.0.0" : "127.0.0.1";
            video_streamer_ = std::make_unique<driver::VideoStreamer>(8090, ip, 50);
        }
    }

    Detector::~Detector() = default;

    // void Detector::setParams(const Params_ToVideo &params_to_video, 
    //     const Params_ToSerialPort &params_to_serial_port)
    // {
    //     _detector_thread_params.frame_pp = params_to_video.frame_pp;

    // }

    DetectResult Detector::startdetect(const cv::Mat& img, int mode, int color, const cv::Rect &roi)
    {
        DetectResult result;
        Detections detections;
        CarDetections car_detections;
        if (img.empty()) return result;
        result.debug_img = img.clone();

        // 设置敌方颜色
        armor_infer->setColorFlag(color);

        // 运行装甲板检测（所有模式都需要）
        auto detected_armors = armor_infer->detect(img);
        for (auto& det : detected_armors) {
            updateArmorGeometry(det);
        }

        detections.reserve(detected_armors.size());
        const int offset_x = (roi.size() != cv::Size(0,0)) ? roi.x : 0;
        const int offset_y = (roi.size() != cv::Size(0,0)) ? roi.y : 0;
        
        for(auto &det : detected_armors) {
            detections.emplace_back();
            auto &result = detections.back();
            // 引用解析边框并设置边界框和中心
            result.bounding_rect = cv::Rect2f(
                det.rect.x + offset_x,
                det.rect.y + offset_y,
                det.rect.width,
                det.rect.height
            );
            result.center = cv::Point2f(
                det.center.x + offset_x,
                det.center.y + offset_y
            );
            // 高效处理角点
            result.corners.reserve(4);
            for(auto &corner : det.corners) {
                result.corners.push_back(cv::Point2f(
                    corner.x + offset_x,
                    corner.y + offset_y
                ));
            }            
            // 复制其他属性
            result.tag_id = det.tag_id;
            result.score = det.confidence;
            result.color_id = det.color_id;
        }

        // 运行整车检测（仅模式 3 需要）
        if (mode == 3) {
            auto detected_cars = yolo_infer->detect(img);          
            car_detections.reserve(detected_cars.size());//提前分配内存空间
            for(auto &det : detected_cars) {
                car_detections.emplace_back();
                auto &car_detection = car_detections.back();               
                car_detection.bounding_rect = cv::Rect2f(det.rect);
                car_detection.center = det.center;
                car_detection.tag_id = det.class_id;
                car_detection.score = det.confidence;
                //LOG(INFO) << "yolo car detection: " << det.rect.x << " " << det.rect.y 
                //          << " " << det.rect.width << " " << det.rect.height;
                //LOG(INFO) << "yolo confidence: " << det.confidence;

                // 在image上绘制检测框
                cv::rectangle(result.debug_img, det.rect, cv::Scalar(0, 255, 0), 2);
                std::string label = "Class: " + std::to_string(det.class_id) + " " + std::to_string(det.confidence).substr(0, 4);
                int baseline = 0;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            }    
        }
        if (GlobalParam::STREAM_ENABLE) {
            img.copyTo(visualization_frame);
            Visualization(visualization_frame, detected_armors);                
        }

        result.armors = detections;
        result.cars = car_detections;
        return result;
    }

    void Detector::Visualization(cv::Mat& drawing, BBoxes armor_targets){ // 可视化DEBUG
        if(armor_targets.size()>0){
            for (int i = 0; i < armor_targets.size(); i++)
            {
                cv::line(drawing, armor_targets[i].corners[0], armor_targets[i].corners[2],cv::Scalar(0, 255, 0), 2);
                cv::line(drawing, armor_targets[i].corners[1], armor_targets[i].corners[3], cv::Scalar(0, 255, 0), 2);
                cv::putText(drawing, std::to_string(armor_targets[i].tag_id), Point2f(armor_targets[i].center.x-20, armor_targets[i].center.y-20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
                Point2f center = armor_targets[i].center;
                circle(drawing, center, 10, Scalar(0, 0, 255), -1);
            }
        }
    }

    void Detector::drawReprojection(cv::Mat& drawing, const std::vector<estimator::Armor>& armors) {
        if (armors.empty()) return;

        for (size_t armor_idx = 0; armor_idx < armors.size(); ++armor_idx) {
            const auto& armor = armors[armor_idx];
            // 绘制重投影的装甲板角点（红色），用于对比解算精度
            if (!armor.reprojected_corners.empty() && armor.reprojected_corners.size() == 4) {
                // 绘制重投影的四个角点
                for (int j = 0; j < 4; j++) {
                    cv::circle(drawing, armor.reprojected_corners[j], 4, cv::Scalar(0, 0, 255), 2);
                }
                // 绘制重投影的装甲板轮廓（红色）
                cv::line(drawing, armor.reprojected_corners[0], armor.reprojected_corners[1], cv::Scalar(0, 0, 255), 1);
                cv::line(drawing, armor.reprojected_corners[1], armor.reprojected_corners[2], cv::Scalar(0, 0, 255), 1);
                cv::line(drawing, armor.reprojected_corners[2], armor.reprojected_corners[3], cv::Scalar(0, 0, 255), 1);
                cv::line(drawing, armor.reprojected_corners[3], armor.reprojected_corners[0], cv::Scalar(0, 0, 255), 1);
                
                // 计算重投影误差
                double total_error = 0.0;
                for (int j = 0; j < 4; j++) {
                    double error = cv::norm(armor.corners[j] - armor.reprojected_corners[j]);
                    total_error += error;
                }
                double avg_error = total_error / 4.0;
                
                // 显示重投影误差
                std::string error_text = "Reproj: " + cv::format("%.2f", avg_error);
                cv::putText(drawing, error_text, Point2f(armor.center.x - 50, armor.center.y - 40), 
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);

                // 显示装甲板朝向角
                std::string angle_text = cv::format("Ang:%.1f", armor.angle);
                cv::putText(drawing, angle_text, Point2f(armor.center.x - 50, armor.center.y - 55),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
                
                // 显示位置信息
                std::string pos_text = cv::format("X:%.2f Y:%.2f Z:%.2f", armor.x, armor.y, armor.z);
                cv::putText(drawing, pos_text, Point2f(armor.center.x - 80, armor.center.y + 50), 
                            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 255), 1);
            }
        }

        // ========== AntiTop 专属可视化：Z-MAP 信息面板 ==========
        const auto& zmap = GlobalParam::Z_MAP;
        if (!zmap.empty()) {
            int panel_x = drawing.cols - 280;
            int panel_y = 30;

            // 标题
            cv::Scalar title_color = GlobalParam::IS_CALIBRATED ? Scalar(0, 255, 0) : Scalar(0, 255, 255);
            std::string title = GlobalParam::IS_CALIBRATED ? "Z-MAP [CALIBRATED]" : cv::format("Z-MAP [%d/3]", (int)zmap.size());
            cv::putText(drawing, title, cv::Point(panel_x, panel_y),
                        FONT_HERSHEY_SIMPLEX, 0.7, title_color, 2);

            // 列出每个 z 值
            for (size_t i = 0; i < zmap.size(); ++i) {
                panel_y += 28;
                std::string z_text = cv::format("  Z[%d] = %.4f m", (int)i, zmap[i]);
                cv::putText(drawing, z_text, cv::Point(panel_x, panel_y),
                            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
            }

            // 显示层间差值
            if (zmap.size() >= 2) {
                panel_y += 28;
                for (size_t i = 1; i < zmap.size(); ++i) {
                    double diff_cm = (zmap[i] - zmap[i - 1]) * 100.0;
                    std::string diff_text = cv::format("  d(%d-%d)=%.1fcm", (int)(i - 1), (int)i, diff_cm);
                    cv::putText(drawing, diff_text, cv::Point(panel_x, panel_y),
                                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
                    panel_y += 22;
                }
            }
        }
        
        // 添加图例说明
        cv::putText(drawing, "Green: Detected | Red: Reprojected", cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    }

    void Detector::drawMode1PitchPlot(cv::Mat& drawing) {
        if (drawing.empty() || drawing.cols <= 160 || drawing.rows <= 120) return;

        static std::deque<double> pitch_history;
        static double last_pitch = 0.0;
        static bool has_last_pitch = false;
        constexpr std::size_t kMaxSamples = 180;
        constexpr double kMinHalfRangeDeg = 0.5;

        const double pitch = SerialParam::send_data.pitch;
        pitch_history.push_back(pitch);
        while (pitch_history.size() > kMaxSamples) {
            pitch_history.pop_front();
        }

        double min_pitch = pitch;
        double max_pitch = pitch;
        for (double value : pitch_history) {
            min_pitch = std::min(min_pitch, value);
            max_pitch = std::max(max_pitch, value);
        }
        double center_pitch = 0.5 * (min_pitch + max_pitch);
        double half_range = std::max(kMinHalfRangeDeg, 0.5 * (max_pitch - min_pitch));
        min_pitch = center_pitch - half_range;
        max_pitch = center_pitch + half_range;

        const int margin = 12;
        const int panel_w = std::min(360, drawing.cols - margin * 2);
        const int panel_h = 130;
        const int px = margin;
        const int py = drawing.rows - panel_h - margin;
        const cv::Rect panel(px, py, panel_w, panel_h);

        cv::rectangle(drawing, panel, cv::Scalar(18, 18, 18), cv::FILLED);
        cv::rectangle(drawing, panel, cv::Scalar(120, 120, 120), 1);

        const int left = px + 10;
        const int right = px + panel_w - 10;
        const int top = py + 30;
        const int bottom = py + panel_h - 20;
        auto mapPitchToY = [&](double value) {
            const double t = std::clamp((value - min_pitch) / (max_pitch - min_pitch), 0.0, 1.0);
            return bottom - static_cast<int>(t * (bottom - top));
        };

        const int mid_y = mapPitchToY(center_pitch);
        cv::line(drawing, cv::Point(left, mid_y), cv::Point(right, mid_y), cv::Scalar(70, 70, 70), 1);

        const int n = static_cast<int>(pitch_history.size());
        for (int i = 1; i < n; ++i) {
            const double p0 = pitch_history[static_cast<std::size_t>(i - 1)];
            const double p1 = pitch_history[static_cast<std::size_t>(i)];
            const int x0 = left + (i - 1) * (right - left) / std::max(1, n - 1);
            const int x1 = left + i * (right - left) / std::max(1, n - 1);
            cv::line(drawing, cv::Point(x0, mapPitchToY(p0)),
                     cv::Point(x1, mapPitchToY(p1)), cv::Scalar(0, 220, 255), 2);
        }

        const double delta = has_last_pitch ? pitch - last_pitch : 0.0;
        last_pitch = pitch;
        has_last_pitch = true;

        cv::putText(drawing, cv::format("mode1 send pitch: %.3f deg", pitch),
                    cv::Point(px + 8, py + 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(drawing, cv::format("d %.3f", delta),
                    cv::Point(px + 220, py + 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 220, 255), 1);
        cv::putText(drawing, cv::format("max %.2f", max_pitch),
                    cv::Point(right - 78, top + 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
        cv::putText(drawing, cv::format("min %.2f", min_pitch),
                    cv::Point(right - 78, bottom),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
    }

    int Detector::determineOperationMode(uint8_t flag)
    {
        static const std::unordered_map<uint8_t, int> flag_mapping{
            {0x01, 1},  // NormalAim - 普通自瞄
            {0x02, 2},  // AntiTop - 反前哨
            {0x03, 3},  // AutoAim - 小陀螺
            {0x04, 4}   // Sniper - 吊射图传
        };

        int mode;
        auto it = flag_mapping.find(flag);
        if (it != flag_mapping.end()) {
            mode = it->second;
        } else {
            mode = 1; // 默认模式
        }

        // 只有在调试模式下才使用配置文件中的MODE
        if (GlobalParam::DEBUG_MODE) {
            mode = GlobalParam::MODE;
        }

        return mode;
    }

    void Detector::pushStream(const cv::Mat& img)
    {
        if (stream_enabled_ && video_streamer_) {
            video_streamer_->setFrame(img);
        }
    }

    void Detector::drawAntiTopStatus(cv::Mat& drawing)
    {
        int x = 10;
        int y = 55;  // 避开 y=30 处的 legend 文字

        // ---- 标题 ----
        cv::putText(drawing, "[ ANTITOP ]", cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
        y += 30;

        // ---- 数据收集进度 ----
        int data_cnt = GlobalParam::ANTITOP_DATA_COUNT;
        cv::Scalar data_color = (data_cnt >= 30) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 165, 0);
        std::string data_str = (data_cnt >= 30) ?
            "DATA: READY" : cv::format("DATA: %d/30", data_cnt);
        cv::putText(drawing, data_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, data_color, 2);
        y += 30;

        // ---- 旋转方向 ----
        int dir = SerialParam::direction;
        std::string dir_str;
        cv::Scalar dir_color;
        if (dir == 1)       { dir_str = "DIR: CW";      dir_color = cv::Scalar(0, 255, 0); }
        else if (dir == 0)  { dir_str = "DIR: CCW";     dir_color = cv::Scalar(0, 165, 255); }
        else                { dir_str = "DIR: UNKNOWN"; dir_color = cv::Scalar(180, 180, 180); }
        cv::putText(drawing, dir_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, dir_color, 2);
        y += 30;

        // ---- 标定进度 ----
        const auto& zmap = GlobalParam::Z_MAP;
        cv::Scalar calib_color = GlobalParam::IS_CALIBRATED ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
        std::string calib_str = GlobalParam::IS_CALIBRATED ?
            "CALIB: DONE" :
            cv::format("CALIB: %d/3", (int)zmap.size());
        cv::putText(drawing, calib_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, calib_color, 2);
        y += 30;

        // ---- 射击区状态 ----
        if (GlobalParam::ANTITOP_IN_SHOOT_ZONE) {
            cv::putText(drawing, "IN ZONE", cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::rectangle(drawing,
                          cv::Point(x - 4, y - 22),
                          cv::Point(x + 120, y + 8),
                          cv::Scalar(0, 0, 255), 2);
        } else {
            cv::putText(drawing, "OUT ZONE", cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(160, 160, 160), 2);
        }
        y += 35;

        // ---- Pitch 锁定状态 ----
        int ps = GlobalParam::ANTITOP_PITCH_STATE;
        std::string pitch_str;
        cv::Scalar pitch_color;
        if (ps == 1)      { pitch_str = "PITCH: LOCKED";     pitch_color = cv::Scalar(0, 255, 255); }
        else if (ps == 2) { pitch_str = "PITCH: POST-FIRE";  pitch_color = cv::Scalar(0, 165, 255); }
        else              { pitch_str = "PITCH: FREE";        pitch_color = cv::Scalar(255, 255, 255); }
        cv::putText(drawing, pitch_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, pitch_color, 2);
        y += 30;

        const bool target_valid = GlobalParam::ANTITOP_TARGET_VALID;
        cv::Scalar target_info_color = target_valid ?
            cv::Scalar(255, 255, 255) : cv::Scalar(160, 160, 160);
        std::string target_z_str = target_valid ?
            cv::format("TARGET_Z: %.3fm", GlobalParam::ANTITOP_TARGET_Z) : "TARGET_Z: --";
        cv::putText(drawing, target_z_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, target_info_color, 2);
        y += 30;

        std::string target_pitch_str = target_valid ?
            cv::format("TARGET_PITCH: %.2f", GlobalParam::ANTITOP_TARGET_PITCH) : "TARGET_PITCH: --";
        cv::putText(drawing, target_pitch_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, target_info_color, 2);
        y += 30;

        int zone_idx = GlobalParam::ANTITOP_LAST_ZONE_INDEX;
        std::string zone_idx_str;
        cv::Scalar zone_idx_color;
        if (zone_idx >= 0) {
            zone_idx_str = cv::format("Z_IDX: %d", zone_idx);
            zone_idx_color = cv::Scalar(0, 0, 255);
        } else if (zone_idx == -2) {
            zone_idx_str = "Z_IDX: ERROR";
            zone_idx_color = cv::Scalar(0, 0, 255);
        } else {
            zone_idx_str = "Z_IDX: --";
            zone_idx_color = cv::Scalar(160, 160, 160);
        }
        cv::putText(drawing, zone_idx_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, zone_idx_color, 2);
        y += 30;

        int avg_ms = GlobalParam::ANTITOP_AVG_MS;
        std::string avg_ms_str = avg_ms > 0 ?
            cv::format("AVG_MS: %d", avg_ms) : "AVG_MS: --";
        cv::Scalar avg_ms_color = avg_ms > 0 ?
            cv::Scalar(0, 255, 0) : cv::Scalar(160, 160, 160);
        cv::putText(drawing, avg_ms_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, avg_ms_color, 2);

        // ---- 旋转中心竖线（数据足够时绘制） ----
        y += 30;

        const bool shoot_flag = GlobalParam::ANTITOP_SHOOT_FLAG;
        const std::string shoot_flag_str = shoot_flag ? "SHOOT_FLAG: 1" : "SHOOT_FLAG: 0";
        const cv::Scalar shoot_flag_color = shoot_flag ? cv::Scalar(0, 0, 255) : cv::Scalar(160, 160, 160);
        if (shoot_flag) {
            cv::rectangle(drawing,
                          cv::Point(x - 4, y - 22),
                          cv::Point(x + 170, y + 8),
                          shoot_flag_color, 2);
        }
        cv::putText(drawing, shoot_flag_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, shoot_flag_color, 2);
        y += 30;

        cv::Point2f sc = SerialParam::shoot_center;
        if (GlobalParam::IS_CALIBRATED &&
            std::isfinite(sc.y) &&
            sc.y > 5 && sc.y < drawing.rows - 5)
        {
            const int sc_y = static_cast<int>(std::lround(sc.y));
            cv::Scalar target_z_color(255, 255, 0);
            cv::line(drawing, cv::Point(0, sc_y), cv::Point(drawing.cols, sc_y), target_z_color, 2);
            cv::putText(drawing, "LOW Z", cv::Point(10, std::max(24, sc_y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, target_z_color, 2);
        }
        if (data_cnt >= 5 &&
            sc.x > 5 && sc.x < drawing.cols - 5)
        {
            int sc_x = static_cast<int>(sc.x);
            cv::Scalar line_color(0, 255, 255);  // 青色
            // 画竖线贯穿全图
            cv::line(drawing, cv::Point(sc_x, 0), cv::Point(sc_x, drawing.rows), line_color, 2);
            // 标签
            cv::putText(drawing, "ROT X", cv::Point(sc_x + 6, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, line_color, 2);
        }

        // ---- 左下角 tracking z 曲线 ----
        static std::deque<double> tracking_z_hist;
        static uint64_t last_z_seq = 0;
        static constexpr int kMaxPoints = 180;
        static constexpr double kFixedHalfRangeM = 0.40; // 上下各 40cm
        static bool axis_center_initialized = false;
        static double axis_center_z = 0.0;

        if (GlobalParam::ANTITOP_TRACKING_Z_VALID &&
            GlobalParam::ANTITOP_TRACKING_Z_SEQ != last_z_seq) {
            tracking_z_hist.push_back(GlobalParam::ANTITOP_TRACKING_Z);
            if (tracking_z_hist.size() > kMaxPoints)
                tracking_z_hist.pop_front();
            if (!axis_center_initialized) {
                axis_center_z = GlobalParam::ANTITOP_TRACKING_Z;
                axis_center_initialized = true;
            }
            last_z_seq = GlobalParam::ANTITOP_TRACKING_Z_SEQ;
        }

        if (!tracking_z_hist.empty() && drawing.cols > 120 && drawing.rows > 120) {
            const int margin = 12;
            const int panel_w = std::min(340, drawing.cols - margin * 2);
            const int panel_h = 130;
            const int px = margin;
            const int py = drawing.rows - panel_h - margin;

            cv::rectangle(drawing, cv::Rect(px, py, panel_w, panel_h), cv::Scalar(20, 20, 20), cv::FILLED);
            cv::rectangle(drawing, cv::Rect(px, py, panel_w, panel_h), cv::Scalar(120, 120, 120), 1);
            
            const int left = px + 10;
            const int right = px + panel_w - 10;
            const int top = py + 28;
            const int bottom = py + panel_h - 20;

            // 固定 y 轴量程：围绕首次观测 z，上下各 50cm
            const double min_z = axis_center_z - kFixedHalfRangeM;
            const double max_z = axis_center_z + kFixedHalfRangeM;
            auto mapZToY = [&](double z) {
                const double t = std::clamp((z - min_z) / (max_z - min_z), 0.0, 1.0);
                return bottom - static_cast<int>(t * (bottom - top));
            };

            const int n = static_cast<int>(tracking_z_hist.size());
            for (int i = 1; i < n; ++i) {
                const double z0 = tracking_z_hist[static_cast<size_t>(i - 1)];
                const double z1 = tracking_z_hist[static_cast<size_t>(i)];
                const int x0 = left + (i - 1) * (right - left) / std::max(1, n - 1);
                const int x1 = left + i * (right - left) / std::max(1, n - 1);
                const int y0 = mapZToY(z0);
                const int y1 = mapZToY(z1);
                cv::line(drawing, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 220, 255), 2);
            }

            const double cur_z = tracking_z_hist.back();
            cv::putText(drawing, cv::format("tracking z: %.3fm", cur_z), cv::Point(px + 8, py + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::putText(drawing, cv::format("center %.3f", axis_center_z), cv::Point(px + 132, py + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(180, 180, 180), 1);
            cv::putText(drawing, cv::format("max %.3f", max_z), cv::Point(right - 92, top + 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
            cv::putText(drawing, cv::format("min %.3f", min_z), cv::Point(right - 92, bottom),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(180, 180, 180), 1);
        }
    }
}
