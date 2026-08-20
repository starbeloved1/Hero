#include "Detector/include/CornerRefiner.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace detector {
namespace {

float pointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    return cv::norm(a - b);
}

bool isFinitePoint(const cv::Point2f& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

} // namespace

bool CornerRefiner::refine(const cv::Mat& frame,
                           cv::Point2f corners[4],
                           int /*color_id*/) const {
    if (!config_.enabled || frame.empty()) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (!isFinitePoint(corners[i])) {
            return false;
        }
    }

    cv::Mat gray_image;
    if (frame.channels() == 1) {
        gray_image = frame;
    } else {
        cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
    }

    LightBar left{};
    left.top = corners[0];
    left.bottom = corners[3];
    left.length = pointDistance(left.top, left.bottom);
    left.width = estimateLightBarWidth(corners);

    LightBar right{};
    right.top = corners[1];
    right.bottom = corners[2];
    right.length = pointDistance(right.top, right.bottom);
    right.width = left.width;

    if (left.length <= 1.0f || right.length <= 1.0f) {
        return false;
    }

    LightBar left_copy = left;
    LightBar right_copy = right;
    if (!correctLightBar(left_copy, gray_image) ||
        !correctLightBar(right_copy, gray_image)) {
        return false;
    }

    const std::vector<cv::Point2f> refined = {
        left_copy.top,
        right_copy.top,
        right_copy.bottom,
        left_copy.bottom
    };
    const cv::Rect refined_rect = cv::boundingRect(refined);
    if (refined_rect.width <= 0 || refined_rect.height <= 0) {
        return false;
    }
    const float ratio = static_cast<float>(std::max(refined_rect.width, refined_rect.height)) /
                        static_cast<float>(std::max(1, std::min(refined_rect.width, refined_rect.height)));
    if (ratio < 1.0f || ratio > 6.0f) {
        return false;
    }

    corners[0] = left_copy.top;
    corners[1] = right_copy.top;
    corners[2] = right_copy.bottom;
    corners[3] = left_copy.bottom;
    return true;
}

bool CornerRefiner::correctLightBar(LightBar& lightbar, const cv::Mat& gray_image) const {
    if (lightbar.width <= static_cast<float>(config_.pass_optimize_lightbar_width)) {
        return false;
    }

    const auto axis = findSymmetryAxis(gray_image, lightbar);
    if (!axis.has_value()) {
        return false;
    }

    lightbar.center = axis->centroid;
    lightbar.axis = axis->direction;

    const auto top = findCorner(gray_image, lightbar, axis.value(), true);
    const auto bottom = findCorner(gray_image, lightbar, axis.value(), false);
    if (!top.has_value() || !bottom.has_value()) {
        return false;
    }

    lightbar.top = top.value();
    lightbar.bottom = bottom.value();
    return true;
}

std::optional<CornerRefiner::SymmetryAxis> CornerRefiner::findSymmetryAxis(
    const cv::Mat& gray_image,
    const LightBar& lightbar) const {
    auto light_box = lightbar.boundingRect();
    const int pad_x = static_cast<int>(
        std::lround(static_cast<float>(light_box.width) * config_.padding_scale));
    const int pad_y = static_cast<int>(
        std::lround(static_cast<float>(light_box.height) * config_.padding_scale));
    light_box.x -= pad_x;
    light_box.y -= pad_y;
    light_box.width += pad_x * 2;
    light_box.height += pad_y * 2;

    light_box.x = std::clamp(light_box.x, 0, gray_image.cols - 1);
    light_box.y = std::clamp(light_box.y, 0, gray_image.rows - 1);
    light_box.width = std::min(light_box.width, gray_image.cols - light_box.x);
    light_box.height = std::min(light_box.height, gray_image.rows - light_box.y);
    if (light_box.width <= 1 || light_box.height <= 1) {
        return std::nullopt;
    }

    cv::Mat roi = gray_image(light_box).clone();
    const float mean_val = static_cast<float>(cv::mean(roi)[0]);
    if (mean_val <= config_.lightbar_min_mean_brightness) {
        return std::nullopt;
    }

    roi.convertTo(roi, CV_32F);
    cv::normalize(roi, roi, 0, config_.normalize_max_brightness, cv::NORM_MINMAX);

    const cv::Moments moments = cv::moments(roi, false);
    if (moments.m00 == 0.0) {
        return std::nullopt;
    }

    cv::Point2f centroid(
        static_cast<float>(moments.m10 / moments.m00) + static_cast<float>(light_box.x),
        static_cast<float>(moments.m01 / moments.m00) + static_cast<float>(light_box.y));

    const double mu20 = moments.mu20;
    const double mu11 = moments.mu11;
    const double mu02 = moments.mu02;
    if (mu20 == 0.0 && mu11 == 0.0 && mu02 == 0.0) {
        return std::nullopt;
    }

    const double theta = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
    cv::Point2f direction(static_cast<float>(std::cos(theta)),
                          static_cast<float>(std::sin(theta)));
    const float norm = cv::norm(direction);
    if (norm <= 1e-6f) {
        return std::nullopt;
    }
    direction /= norm;

    const cv::Point2f hint = lightbar.top - lightbar.bottom;
    if (direction.dot(hint) < 0.0f) {
        direction = -direction;
    }

    return SymmetryAxis{centroid, direction, mean_val};
}

std::optional<cv::Point2f> CornerRefiner::findCorner(
    const cv::Mat& gray_image,
    const LightBar& lightbar,
    const SymmetryAxis& axis,
    bool find_top) const {
    const auto is_in_image = [&gray_image](const cv::Point& point) {
        return point.x >= 0 && point.x < gray_image.cols &&
               point.y >= 0 && point.y < gray_image.rows;
    };

    const int direction_sign = find_top ? 1 : -1;
    const float length = lightbar.length;
    const float dx = axis.direction.x * static_cast<float>(direction_sign);
    const float dy = axis.direction.y * static_cast<float>(direction_sign);

    std::vector<cv::Point2f> candidates;
    const int sample_width = std::max(
        config_.min_sample_width,
        static_cast<int>(std::lround(lightbar.width)) - 2);
    const int half_width = std::max(0, sample_width / 2);

    for (int i = -half_width; i <= half_width; ++i) {
        const float x0 = axis.centroid.x + length * config_.search_start_ratio * dx +
                         static_cast<float>(i);
        const float y0 = axis.centroid.y + length * config_.search_start_ratio * dy;

        cv::Point2f prev(x0, y0);
        cv::Point2f corner(x0, y0);
        float max_brightness_diff = 0.0f;
        bool has_corner = false;

        for (float x = x0 + dx, y = y0 + dy;
             pointDistance(cv::Point2f(x, y), cv::Point2f(x0, y0)) <
             length * (config_.search_end_ratio - config_.search_start_ratio);
             x += dx, y += dy) {
            const cv::Point prev_pt(static_cast<int>(std::lround(prev.x)),
                                    static_cast<int>(std::lround(prev.y)));
            const cv::Point cur_pt(static_cast<int>(std::lround(x)),
                                   static_cast<int>(std::lround(y)));
            if (!is_in_image(prev_pt) || !is_in_image(cur_pt)) {
                break;
            }

            const float brightness_diff =
                static_cast<float>(gray_image.at<uchar>(prev_pt) -
                                   gray_image.at<uchar>(cur_pt));
            if (brightness_diff > max_brightness_diff &&
                static_cast<float>(gray_image.at<uchar>(prev_pt)) > axis.mean_val) {
                max_brightness_diff = brightness_diff;
                corner = prev;
                has_corner = true;
            }
            prev = cv::Point2f(x, y);
        }

        if (has_corner) {
            candidates.emplace_back(corner);
        }
    }

    if (candidates.empty()) {
        return std::nullopt;
    }

    const auto sum = std::accumulate(candidates.begin(),
                                     candidates.end(),
                                     cv::Point2f(0.0f, 0.0f));
    return sum * (1.0f / static_cast<float>(candidates.size()));
}

float CornerRefiner::estimateLightBarWidth(const cv::Point2f corners[4]) const {
    const float top_width = pointDistance(corners[0], corners[1]);
    const float bottom_width = pointDistance(corners[3], corners[2]);
    return std::max(1.0f, 0.5f * (top_width + bottom_width) * config_.estimated_width_ratio);
}

cv::Rect CornerRefiner::LightBar::boundingRect() const {
    const float half_width = width * 0.5f;
    const float min_x = std::min(top.x, bottom.x) - half_width;
    const float min_y = std::min(top.y, bottom.y) - half_width;
    const float max_x = std::max(top.x, bottom.x) + half_width;
    const float max_y = std::max(top.y, bottom.y) + half_width;
    return cv::Rect(
        cv::Point(static_cast<int>(std::floor(min_x)),
                  static_cast<int>(std::floor(min_y))),
        cv::Point(static_cast<int>(std::ceil(max_x)),
                  static_cast<int>(std::ceil(max_y))));
}

} // namespace detector