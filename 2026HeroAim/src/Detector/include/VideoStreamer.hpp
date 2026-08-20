// Create by Qmian 2025/3/17

#pragma once

// opencv
#include <opencv2/opencv.hpp>

// http 
#include <httplib.h>
#include <mutex>

// std
#include <thread>
#include <vector>

//utils
#include "Params.hpp"
#include "Config.hpp"

using namespace lyutils;

namespace detector {
    class VideoStreamer {
    public:
        VideoStreamer();
        ~VideoStreamer();
        void setFrame(const cv::Mat& frame);

    private:
        void startServer();

        int port_ = 8090;
        std::string local_ip_ = GlobalParam::IS_NUC? "0.0.0.0" : "127.0.0.1";
        int jpeg_quality_ = 50;
        
        std::vector<double> send_yaws;
        std::vector<double> send_pitches;
        std::vector<double> recv_yaws;
        std::vector<double> recv_pitches;
        std::vector<int> shoot_flags;
        std::vector<int> right_clickes;



        std::unique_ptr<httplib::Server> server_;
        std::thread server_thread_;
        std::mutex frame_mutex_;
        std::vector<uchar> current_frame_;
        bool server_running_;
    };
}