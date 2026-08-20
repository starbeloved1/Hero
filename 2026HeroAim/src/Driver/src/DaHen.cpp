#ifdef USE_DAHENG_CAMERA
#include "Driver/include/VideoCapture.h"
#include "Driver/include/GxCamera.h"
#include "utils/include/Params.hpp" 
#include "utils/include/Thread.hpp"
#include "utils/include/Log.hpp"
#include <mutex>
#include <chrono>
#include <cmath>
#include <stdexcept>

using namespace cv;
using namespace lyutils; 
using namespace std;

namespace driver{
    namespace {
        const char* profileName(CameraProfile profile) {
            switch (profile) {
                case CameraProfile::Base: return "Base";
                case CameraProfile::Aim8mm2: return "Aim8mm2";
                default: return "Aim8mm";
            }
        }
    }

    void DaHenCamera::open() {
        const bool is_base = profile_ == CameraProfile::Base;
        const bool is_8mm2 = profile_ == CameraProfile::Aim8mm2;
        const std::string& sn = is_base ? CameraParam::sn
            : (is_8mm2 ? Camera8mm2Param::sn : Camera8mmParam::sn);
        const int exposure_time = is_base ? CameraParam::exposure_time
            : (is_8mm2 ? Camera8mm2Param::exposure_time : Camera8mmParam::exposure_time);
        const double gain = is_base ? CameraParam::gain
            : (is_8mm2 ? Camera8mm2Param::gain : Camera8mmParam::gain);

        if (is_base) {
            width = static_cast<uint16_t>(CameraParam::width);
            height = static_cast<uint16_t>(CameraParam::height);
        }

        LOG(INFO) << "[DaHenCamera] Opening " << profileName(profile_) << " camera, sn=" << sn
                  << ", size=" << width << "x" << height
                  << ", exposure=" << exposure_time
                  << ", gain=" << gain
                  << (is_base ? ", acquisition_fps=" +
                      std::to_string(CameraParam::acquisition_fps) : "");
        if (camera->initLib() != GX_STATUS_SUCCESS) {
            throw std::runtime_error("Daheng camera library init failed");
        }
        if (camera->openDevice(sn.c_str()) != GX_STATUS_SUCCESS) {
            throw std::runtime_error("Daheng camera open failed");
        }
        camera->setRoiParam(width, height, offset_x, offset_y);
        camera->setExposureGainParam( false, false, exposure_time, 1000, 3000, gain, 3, 10, 127,true);
        if (is_base) {
            camera->setExposureControlRangeUs(
                CameraParam::exposure_min_us, CameraParam::exposure_max_us);
            camera->setAcquisitionFrameRate(CameraParam::acquisition_fps);
        }
        camera->setWhiteBalanceParam(true,GX_AWB_LAMP_HOUSE_ADAPTIVE);
        if (camera->acquisitionStart() != GX_STATUS_SUCCESS) {
            throw std::runtime_error("Daheng camera acquisition start failed");
        }
        
    }

    void DaHenCamera::startCapture(Params_ToVideo &params_to_video) {

        // params out
        _video_thread_params.frame_pp = params_to_video.frame_pp;

        int id = 0;
        constexpr int size = 10;

        // Mat frame[size];
        // for(auto & m : frame) m =  Mat(Size(1280, 1024), CV_32FC3);

        Image frame[size];
        for(auto& m: frame) {
            m.mat = new Mat(Size(width, height), CV_8UC3);
        }
        std::chrono::steady_clock::time_point start, end;
        
        do {
            // DLOG(WARNING) <<"                                               Capture     ";
            // 拿到对应的锁, 拿到之后会进行上锁
            unique_lock<mutex> umtx;
            bool& image_is_update = profile_ == CameraProfile::Base ? Thread::image2_is_update
                : (profile_ == CameraProfile::Aim8mm2 ? Thread::image3_is_update : Thread::image_is_update);
            auto& mtx_image = profile_ == CameraProfile::Base ? Thread::mtx_image2
                : (profile_ == CameraProfile::Aim8mm2 ? Thread::mtx_image3 : Thread::mtx_image);
            auto& cond_is_process = profile_ == CameraProfile::Base ? Thread::cond2_is_process
                : (profile_ == CameraProfile::Aim8mm2 ? Thread::cond3_is_process : Thread::cond_is_process);
            auto& cond_is_update = profile_ == CameraProfile::Base ? Thread::cond2_is_update
                : (profile_ == CameraProfile::Aim8mm2 ? Thread::cond3_is_update : Thread::cond_is_update);

            umtx = unique_lock<mutex>(mtx_image);
            // DLOG(WARNING) <<"                                               umtx     ";
            // 等它拿走对应的图像，带超时以便检测停止信号
            while(image_is_update && !Thread::should_stop)
            { cond_is_process.wait_for(umtx, std::chrono::milliseconds(100)); }
            if (Thread::should_stop) {
                umtx.unlock();
                break;
            }
            // DLOG(WARNING) <<"                                               capture 2     ";
            start = std::chrono::steady_clock::now();  
            // 预处理
            if(!camera->ProcGetImage(frame[id].mat, &start)){
                continue;
            }
            // DLOG(WARNING) <<"                                               capture 3     ";
            end = std::chrono::steady_clock::now();
            double delta_t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;

            // DLOG(INFO) << "get frame for " << delta_t << "(ms)";

            frame[id].time_stamp = start+ (end-start) / 2;
            {
                std::lock_guard<std::mutex> serial_lock(SerialParam::serial_mutex);
                frame[id].imu_data = SerialParam::recv_data;
            }
            *_video_thread_params.frame_pp = &frame[id];
            image_is_update = true;  
            cond_is_update.notify_one();  
            umtx.unlock();  
            id = (id+1) % size;
            
            LOG_IF(ERROR, (*_video_thread_params.frame_pp)->mat->empty()) << "get empty picture mat!";
            // DLOG(WARNING) <<"                                               capture end  ";
        // 读取到的图像不是空的，且程序未请求停止
        } while(!(*_video_thread_params.frame_pp)->mat->empty() && !Thread::should_stop);
    }

    bool DaHenCamera::adjustExposureUs(double delta_us, double* new_exposure_us)
    {
        if (camera == nullptr || !camera->adjustExposureUs(delta_us, new_exposure_us)) {
            return false;
        }
        // 同步运行期状态，不写回 init.json。
        if (profile_ == CameraProfile::Base && new_exposure_us != nullptr) {
            CameraParam::exposure_time = static_cast<int>(std::lround(*new_exposure_us));
        }
        return true;
    }


    DaHenCamera::DaHenCamera(CameraProfile profile) : VideoCapture(profile) {
        camera = new GxCamera();
    }

    DaHenCamera::~DaHenCamera() {
        delete camera;
    }
}
#endif // USE_DAHENG_CAMERA
