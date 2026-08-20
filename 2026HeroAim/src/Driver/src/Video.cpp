#include "VideoCapture.h"
#include "SerialPort.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"
#include "utils/include/Thread.hpp"

using namespace cv;
using namespace utils;


namespace driver{

    void NativeVideo::open() {
        LOG(INFO) << "[Video] Opening video: " << CameraParam::video_path;
        this->video = cv::VideoCapture(CameraParam::video_path);
        LOG_IF(ERROR, !this->video.isOpened()) << "can't find video in " << CameraParam::video_path;
        if (this->video.isOpened()) {
            int width = (int)this->video.get(cv::CAP_PROP_FRAME_WIDTH);
            int height = (int)this->video.get(cv::CAP_PROP_FRAME_HEIGHT);
            int fps = (int)this->video.get(cv::CAP_PROP_FPS);
            int total_frames = (int)this->video.get(cv::CAP_PROP_FRAME_COUNT);
            LOG(INFO) << "[Video] Opened: " << width << "x" << height << " @ " << fps << "fps, total frames: " << total_frames;
        }
    }
    void NativeVideo::startCapture(Params_ToVideo& params) {
        // params in
        _video_thread_params.video = this->video;
        _video_thread_params.__this = this;
    
        // params out
        _video_thread_params.frame_pp = params.frame_pp;
    
        int id = 0;
        constexpr int size = 10;
        Image frame[size];
        for (auto& m : frame)
            m.mat = new Mat();
    
        unique_lock<mutex> umtx_video(Thread::mtx_image, defer_lock);
    
        this->video >> *frame[id].mat;
        frame[id].time_stamp = std::chrono::steady_clock::now();
        *_video_thread_params.frame_pp = &frame[id];
        id = (id + 1) % size;
        if ((*_video_thread_params.frame_pp)->mat->empty()) {
            LOG(ERROR) << "[Video] First frame is empty! Video may not be readable.";
        } else {
            LOG(INFO) << "[Video] First frame OK: " << (*_video_thread_params.frame_pp)->mat->cols << "x" << (*_video_thread_params.frame_pp)->mat->rows;
        }
    
        Mat hsv;
        double alpha = 1, beta = 30;

        // 根据实际视频帧率动态计算睡眠时间（而非固定10ms）
        const double fps = this->video.get(cv::CAP_PROP_FPS);
        const int frame_us = (fps > 1.0) ? static_cast<int>(1000000.0 / fps) : 33333;
        LOG(INFO) << "[Video] FPS: " << fps << ", frame interval: " << frame_us << " us";

        while (!Thread::should_stop && !(*_video_thread_params.frame_pp)->mat->empty()) {
            umtx_video.lock();
    
            _video_thread_params.video >> *(frame[id].mat);
            frame[id].time_stamp = std::chrono::steady_clock::now();
            *_video_thread_params.frame_pp = &frame[id];
    
            id = (id + 1) % size;
    
            Thread::image_is_update = true;
            Thread::cond_is_update.notify_all();

            umtx_video.unlock();
    
            usleep(frame_us);
        }
    }
    NativeVideo::NativeVideo() {
        
    }

    NativeVideo::~NativeVideo() {
        video.release();
    }
}
