#include "AntiBase/include/Preprocess.hpp"

#include <algorithm>
#include <vector>

namespace antibase {

Preprocess::Preprocess(const PreprocessConfig& cfg)
    : cfg_(cfg) {}

PreprocessResult Preprocess::run(const cv::Mat& frame) {
    PreprocessResult result;
    if (frame.empty()) {
        return result;
    }

    //roi+resize
    int width = frame.cols;
    int height = frame.rows;
    int side = std::min({cfg_.crop_size, width, height});

    const int centered_x = std::max(0, (width - side) / 2);
    const int max_x = std::max(0, width - side);
    int x = std::clamp(centered_x + cfg_.crop_offset_x, 0, max_x);
    int y = std::max(0, (height - side) / 2);
    cv::Rect roi(x, y, side, side);

    cv::Mat cropped = frame(roi).clone();

    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(cfg_.output_size, cfg_.output_size), 0, 0, cv::INTER_LINEAR);
    result.roi_downsample = resized.clone();

    //单色化(可选)
    cv::Mat working = resized;
    if (cfg_.force_monochrome) {
        cv::Mat gray_full;
        cv::cvtColor(working, gray_full, cv::COLOR_BGR2GRAY);
        cv::cvtColor(gray_full, working, cv::COLOR_GRAY2BGR);
    }

    // 运动检测与拖影独立于静态简化开关：即使输出完整 ROI，仍需识别
    // 真实局部运动，供拖影使用。
    cv::Mat gray;
    cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);

    //背景建模初始化
    if (background_gray_f32_.empty()) {
        gray.convertTo(background_gray_f32_, CV_32F);
        result.static_removed = working.clone();
        result.final_frame = working;
        return result;
    }

    //运动检测
    cv::Mat bg_u8;
    cv::convertScaleAbs(background_gray_f32_, bg_u8);

    cv::Mat diff;
    cv::absdiff(gray, bg_u8, diff);

    cv::Mat motion_mask;
    cv::threshold(diff, motion_mask, cfg_.motion_threshold, 255, cv::THRESH_BINARY);

    //形态学清洗
    if (cfg_.motion_erode_px > 0) {
        if (motion_erode_kernel_.empty()) {
            const int k = 2 * cfg_.motion_erode_px + 1;
            motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        }
        cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
    }
    if (cfg_.motion_dilate_px > 0) {
        if (motion_dilate_kernel_.empty()) {
            const int k = 2 * cfg_.motion_dilate_px + 1;
            motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        }
        cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
    }

    // 以真实运动掩码判断全局运动；中心保护区不能参与此判断，也不能被
    // 当成拖影区域，否则扩大中心保护区会把全画面都加入历史残影。
    result.motion_ratio = static_cast<double>(cv::countNonZero(motion_mask)) /
                          static_cast<double>(motion_mask.total());
    const double ema_alpha = std::clamp(cfg_.motion_ratio_ema_alpha, 0.01, 1.0);
    if (!motion_ratio_initialized_) {
        smoothed_motion_ratio_ = result.motion_ratio;
        motion_ratio_initialized_ = true;
    } else {
        smoothed_motion_ratio_ += ema_alpha * (result.motion_ratio - smoothed_motion_ratio_);
    }
    const double enter_ratio = std::clamp(cfg_.trail_disable_motion_ratio, 0.0, 1.0);
    const double resume_ratio = std::clamp(
        std::min(cfg_.trail_resume_motion_ratio, enter_ratio), 0.0, 1.0);
    if (global_motion_active_) {
        if (smoothed_motion_ratio_ <= resume_ratio) {
            global_motion_active_ = false;
        }
    } else if (smoothed_motion_ratio_ >= enter_ratio) {
        global_motion_active_ = true;
    }
    result.suppress_trail = global_motion_active_;

    // 相机或机器人整体抖动时，运动掩码往往只能覆盖画面的一部分。
    // 若仍把掩码外区域灰度/模糊，会在物体边缘留下明显的“撕裂”和失真。
    // 此时宁可把完整 ROI 交给编码器，也不要继续做静态区域简化。
    if (result.suppress_trail) {
        motion_mask_history_.clear();
        trail_frame_history_.clear();
        cv::accumulateWeighted(
            gray, background_gray_f32_, std::clamp(cfg_.bg_update_alpha, 0.001, 0.2));

        // 显示辅助线统一由 MQTT 解码端叠加，避免被编码进车端 H.264 码流。
        result.static_removed = working.clone();
        result.final_frame = working;
        return result;
    }

    // 拖影只使用真实运动区域；静态简化的中心保护只影响显示用掩码。
    const cv::Mat trail_motion_mask = motion_mask.clone();
    cv::Mat focused;
    if (cfg_.static_simplify) {
        cv::Mat focus_mask = motion_mask.clone();
        if (cfg_.center_clear_size > 0) {
            const int clear_size = std::min({cfg_.center_clear_size, working.cols, working.rows});
            const int x0 = std::max(0, working.cols / 2 - clear_size / 2);
            const int y0 = std::max(0, working.rows / 2 - clear_size / 2);
            const int cw = std::min(clear_size, working.cols - x0);
            const int ch = std::min(clear_size, working.rows - y0);
            cv::rectangle(focus_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
        }

        cv::Mat static_base = working.clone();
        if (!cfg_.force_monochrome) {
            cv::Mat gray_bg;
            cv::cvtColor(static_base, gray_bg, cv::COLOR_BGR2GRAY);
            cv::cvtColor(gray_bg, static_base, cv::COLOR_GRAY2BGR);
        }

        cv::Mat blurred_static;
        cv::GaussianBlur(
            static_base,
            blurred_static,
            cv::Size(),
            std::max(0.0, cfg_.bg_blur_sigma),
            std::max(0.0, cfg_.bg_blur_sigma));

        focused = blurred_static.clone();
        working.copyTo(focused, focus_mask);
    } else {
        focused = working.clone();
    }
    result.static_removed = focused.clone();

    //运动拖影
    if (cfg_.motion_trail_frames > 0) {
        motion_mask_history_.push_back(trail_motion_mask);
        trail_frame_history_.push_back(working.clone());

        const size_t max_history = static_cast<size_t>(cfg_.motion_trail_frames + 1);
        while (motion_mask_history_.size() > max_history) {
            motion_mask_history_.pop_front();
        }
        while (trail_frame_history_.size() > max_history) {
            trail_frame_history_.pop_front();
        }

        const size_t history_size = motion_mask_history_.size();
        if (!result.suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
            cv::Mat trail_mask = trail_motion_mask.clone();
            cv::Mat trail_img = working.clone();
            for (size_t i = 0; i < history_size - 1; ++i) {
                cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
                cv::max(trail_img, trail_frame_history_[i], trail_img);
            }
            // 仅在历史帧实际参与拖影时提亮合成区域。只放大亮度通道，保持
            // BGR 色相/饱和度不变；掩码外的背景及当前非运动区域完全不受影响。
            const double gain = std::clamp(cfg_.trail_brightness_gain, 1.0, 3.0);
            if (gain > 1.0) {
                cv::Mat ycrcb;
                cv::cvtColor(trail_img, ycrcb, cv::COLOR_BGR2YCrCb);
                std::vector<cv::Mat> channels;
                cv::split(ycrcb, channels);
                channels[0].convertTo(channels[0], CV_8U, gain);
                cv::merge(channels, ycrcb);
                cv::cvtColor(ycrcb, trail_img, cv::COLOR_YCrCb2BGR);
            }
            trail_img.copyTo(focused, trail_mask);
        }
    } else {
        motion_mask_history_.clear();
        trail_frame_history_.clear();
    }

    //背景更新
    cv::accumulateWeighted(gray, background_gray_f32_, std::clamp(cfg_.bg_update_alpha, 0.001, 0.2));

    cv::Mat bgr;
    if (focused.channels() == 1) {
        cv::cvtColor(focused, bgr, cv::COLOR_GRAY2BGR);
    } else {
        bgr = focused;
    }

    result.final_frame = bgr;
    return result;
}

}  // namespace antibase
