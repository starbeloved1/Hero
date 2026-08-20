// Create by Qmian 2025/3/17

#pragma once

// opencv
#include <opencv2/opencv.hpp>

// http
#include <httplib.h>
#include <condition_variable>
#include <cstdint>
#include <mutex>

// std
#include <thread>
#include <vector>
#include <atomic>

namespace driver {
    class VideoStreamer {
    public:
        VideoStreamer(int port = 8090, const std::string& ip = "0.0.0.0", int jpeg_quality = 50);
        ~VideoStreamer();

        void setFrame(const cv::Mat& frame);
        void stop();
        bool isRunning() const { return server_running_.load(); }

    private:
        void startServer();

        int port_;
        std::string local_ip_;
        int jpeg_quality_;

        std::unique_ptr<httplib::Server> server_;
        std::thread server_thread_;
        std::mutex frame_mutex_;
        std::condition_variable frame_updated_;
        std::vector<uchar> current_frame_;
        uint64_t frame_sequence_ = 0;
        std::atomic<bool> server_running_;
    };
}
