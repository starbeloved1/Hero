#pragma once

// opencv
#include "opencv2/opencv.hpp"

// driver
#include "VideoCapture.h"

// utils
#include "Params.hpp"

using namespace cv;

namespace detector {
    class VideoSaver{
    public: 
        VideoSaver();
        ~VideoSaver();
        void initSaver();
        void SaveVideo(const cv::Mat& frame);
    private:
        int id;
        float gamma;
        unordered_map<int, float> gamma_table;
        VideoWriter writer;
        VideoWriter writer_visual;
    };
}
