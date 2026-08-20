#include "../include/VideoCapture.h"
#include "SerialPort.hpp"
#include "../../utils/include/Params.hpp"
#include <exception>

using namespace cv;


namespace driver{
    VideoCapture::VideoCapture(CameraProfile profile) : profile_(profile) {
        width = 1280;
        height = 1024;
        offset_x = 0;
        offset_y = 0;
    }

    VideoCapture::~VideoCapture() {

    }

    void VideoCapture::chooseCameraType(VideoCapture *& video, CameraProfile profile) {
        try {
            switch (CameraParam::device_type) {
#ifdef USE_DAHENG_CAMERA
                case DaHen:
                    video = new DaHenCamera(profile);
                    break;
#endif
                case Video:
                    video = new NativeVideo();
                    break;
                case Picture:
                    video = new NativePicture();
                    break;
                default:
                    video = new NativeVideo();
                    break;
            }
            if (video == nullptr) {
                LOG(ERROR) << "Video create failed" << endl;
            }
            video->open();
            LOG(INFO) << "Video opened" << endl;
        } catch (const std::exception& e) {
            LOG(ERROR) << "Video open failed: " << e.what() << endl;
            delete video;
            video = nullptr;
        } catch (...) {
            LOG(ERROR) << "Video open failed" << endl;
            delete video;
            video = nullptr;
        }
    }
}
