#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace detector {

struct CornerRefinerConfig {
    bool enabled = true;
    int pass_optimize_lightbar_width = 5;
    float normalize_max_brightness = 25.0f;
    float lightbar_min_mean_brightness = 35.0f;
    float padding_scale = 0.10f;
    float search_start_ratio = 0.35f;
    float search_end_ratio = 0.65f;
    float estimated_width_ratio = 0.16f;
    int min_sample_width = 4;
};

class CornerRefiner {
public:
    CornerRefiner() = default;
    explicit CornerRefiner(const CornerRefinerConfig& config) : config_(config) {}

    bool refine(const cv::Mat& frame, cv::Point2f corners[4], int color_id) const;
    void setConfig(const CornerRefinerConfig& config) { config_ = config; }
    const CornerRefinerConfig& getConfig() const { return config_; }

private:
    struct SymmetryAxis {
        cv::Point2f centroid;
        cv::Point2f direction;
        float mean_val = 0.0f;
    };

    struct LightBar {
        cv::Point2f top;
        cv::Point2f bottom;
        cv::Point2f center;
        cv::Point2f axis;
        float length = 0.0f;
        float width = 0.0f;

        cv::Rect boundingRect() const;
    };

    bool correctLightBar(LightBar& lightbar, const cv::Mat& gray_image) const;
    std::optional<SymmetryAxis> findSymmetryAxis(const cv::Mat& gray_image,
                                                 const LightBar& lightbar) const;
    std::optional<cv::Point2f> findCorner(const cv::Mat& gray_image,
                                          const LightBar& lightbar,
                                          const SymmetryAxis& axis,
                                          bool find_top) const;
    float estimateLightBarWidth(const cv::Point2f corners[4]) const;

    CornerRefinerConfig config_;
};

} // namespace detector