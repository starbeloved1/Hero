#ifndef VIDEO_CAPTURE_H
#define VIDEO_CAPTURE_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include "utils/include/TimeStamp.hpp"
#include "Driver/include/SerialPort.hpp"

#ifdef USE_DAHENG_CAMERA
namespace driver { class GxCamera; }
#endif
using namespace lyutils;

namespace driver
{

    void *saveFrameToNative(void *params_p);
    void *getFrameFromPicture(void *params_p);

    // 相机类型
    enum CameraType
    {
        DaHen,              // 大恒
        Video,              // 视频 debug
        Picture,            // 图片 debug
    };

    enum class CameraProfile
    {
        Aim8mm,
        Aim8mm2,
        Base,
    };

    // Image 包含了图像、时间戳以及imu数据       追求时钟同步
    class Image
    {
    public:
        cv::Mat* mat;                                           // 图像
        std::chrono::steady_clock::time_point time_stamp;   // 时间戳
        SerialPortData imu_data;                            // imu_data
    };
    
    struct Params_ToVideo
    {
        cv::VideoCapture video;        // 相机
        Image **frame_pp;              // Image
        void *__this;
        cv::VideoWriter writer;            // 写入 

        Params_ToVideo()
        {
            frame_pp = (Image **)malloc(sizeof(Image *));
            *frame_pp = nullptr;
        }
        ~Params_ToVideo(){
            free(frame_pp);
        }
    };

    /**
     * @brief:
     */
    class VideoCapture
    {
    public:
        explicit VideoCapture(CameraProfile profile = CameraProfile::Aim8mm);
        virtual ~VideoCapture();
        virtual void open() = 0;
        virtual void startCapture(Params_ToVideo &) = 0;
        // 运行期调整曝光；不支持的输入源返回 false。
        virtual bool adjustExposureUs(double /*delta_us*/, double* /*new_exposure_us*/ = nullptr) { return false; }
        void startSave(Params_ToVideo &params_to_video);
        static void chooseCameraType(VideoCapture *&, CameraProfile profile = CameraProfile::Aim8mm);
        CameraProfile profile() const { return profile_; }

    protected:
        CameraProfile profile_;
        double rate{};
        int _id;
        uint16_t height;
        uint16_t width;
        uint16_t offset_x;
        uint16_t offset_y;

        pthread_t threadID{};
        pthread_t threadID2{};

        Params_ToVideo _video_thread_params;

        cv::VideoWriter writer;

        SerialPort *_serial_port;
        SerialPortData *_data_read;
    };

#ifdef USE_DAHENG_CAMERA
    class DaHenCamera : public VideoCapture
    {
    public:
        explicit DaHenCamera(CameraProfile profile = CameraProfile::Aim8mm);
        ~DaHenCamera();
        void startCapture(Params_ToVideo &) override;
        void open() override;
        bool adjustExposureUs(double delta_us, double* new_exposure_us = nullptr) override;

    private:
        GxCamera *camera;
        //        int _id;
    };
#endif

    class NativeVideo : public VideoCapture
    {
    public:
        explicit NativeVideo();
        ~NativeVideo();
        void open() override;
        void startCapture(Params_ToVideo &params) override;

    private:
        cv::VideoCapture video;
    };

    class NativePicture : public VideoCapture
    {
    public:
        explicit NativePicture() = default;
        void open() override;
        void startCapture(Params_ToVideo &) override;

    private:
        std::string base_dir;
        std::string suffix;
        int id = 0;
    };

    struct TimeImageData
    {
        cv::Mat image;
        std::chrono::steady_clock::time_point steady_timestamp;
        Time::TimeStamp timestamp;
    };
}

#endif //AUTOAIM_VIDEOCAPTURE_H
