#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>
#include "utils/include/Params.hpp"
#include "ArmorOneStage.hpp"
#include "YoloDetector.hpp"
#include "Type.hpp"
#include "Driver/include/Videostreamer.hpp"
#include "Driver/include/VideoCapture.h"
#include "Estimator/include/Type.hpp"  // 包含 estimator::Armor 的定义

namespace detector
{
    class Detector
    {
    public:
        Detector(const std::string& armor_model, const std::string& yolo_model);
        ~Detector();
        DetectResult startdetect(const cv::Mat& img, int mode, int color, const cv::Rect &roi = cv::Rect());
        int determineOperationMode(uint8_t flag);
        void Visualization(cv::Mat& drawing, BBoxes armor_targets);
        void drawReprojection(cv::Mat& drawing, const std::vector<estimator::Armor>& armors);  // 绘制重投影
        void drawAntiTopStatus(cv::Mat& drawing);  // Mode 2 AntiTop 专属HUD
        void drawMode1PitchPlot(cv::Mat& drawing);
        void pushStream(const cv::Mat& img);
        cv::Rect roi;
        cv::Mat visualization_frame;

    private:
        std::unique_ptr<ArmorOneStage> armor_infer;
        std::unique_ptr<YoloDetector> yolo_infer;
        std::unique_ptr<driver::VideoStreamer> video_streamer_;
        bool stream_enabled_;
        struct {
            int frame_pp;
        } _detector_thread_params;
    };
}
