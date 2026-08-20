#ifndef HERO_26_ANTIBASE_PREPROCESS_HPP
#define HERO_26_ANTIBASE_PREPROCESS_HPP

#include <deque>

#include <opencv2/opencv.hpp>

namespace antibase {

struct PreprocessConfig {
    int crop_size;
    // 相对于居中 ROI 的水平偏移；负值向左，正值向右。
    int crop_offset_x = 0;
    int output_size;
    bool static_simplify;
    int motion_threshold; //运动检测阈值
    int motion_erode_px;
    int motion_dilate_px;
    int motion_trail_frames;
    // 拖影合成区域的亮度倍率；1.0 表示不额外提亮。
    double trail_brightness_gain = 1.0;
    double trail_disable_motion_ratio; //全局运动进入阈值：超过时旁路静态简化和拖影
    double trail_resume_motion_ratio; //全局运动退出阈值：低于时恢复静态简化和拖影
    double motion_ratio_ema_alpha; //全局运动比例 EMA 系数，减小阈值附近反复切换
    double bg_update_alpha;
    double bg_blur_sigma;
    int center_clear_size;
    bool force_monochrome; //单通道输出（可选）
    int target_bitrate_kbps; //编码目标码率
};

struct PreprocessResult {
    cv::Mat final_frame;
    cv::Mat roi_downsample;
    cv::Mat static_removed;
    double motion_ratio = 0.0;
    bool suppress_trail = false;
};

class Preprocess {
public:
    explicit Preprocess(const PreprocessConfig& cfg);

    PreprocessResult run(const cv::Mat& frame);

private:
    PreprocessConfig cfg_;

    cv::Mat background_gray_f32_;
    cv::Mat motion_erode_kernel_;
    cv::Mat motion_dilate_kernel_;
    std::deque<cv::Mat> motion_mask_history_;
    std::deque<cv::Mat> trail_frame_history_;
    double smoothed_motion_ratio_ = 0.0;
    bool motion_ratio_initialized_ = false;
    bool global_motion_active_ = false;
};

}  // namespace antibase

#endif
