#include "../include/VideoSaver.hpp"
#include <filesystem>
#include <algorithm>
namespace fs = std::filesystem;

namespace detector{
    VideoSaver::VideoSaver() {
    }

    void VideoSaver::initSaver(){
        std::string save_path = CameraParam::video_path.substr(0, CameraParam::video_path.rfind('/') + 1);
        std::vector<int> existing_ids;
        for (const auto& entry : fs::directory_iterator(save_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".avi") {
                std::string filename = entry.path().stem().string();
                try {
                    int id = std::stoi(filename);
                    existing_ids.push_back(id);
                } catch (const std::invalid_argument&) {
                    continue;
                }
            }
        }

        int new_id = 1;
        if (!existing_ids.empty()) {
            new_id = *std::max_element(existing_ids.begin(), existing_ids.end()) + 1;
        }
        std::string video_filename = save_path + std::to_string(new_id) + ".avi";
        writer = cv::VideoWriter(video_filename, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 70.0, cv::Size(1280, 1024));

        LOG(INFO) << "Save video in: " << video_filename;
    }

    VideoSaver::~VideoSaver(){
        writer.release();
    }

    void VideoSaver::SaveVideo(const cv::Mat& frame){
        writer.write(frame);
    }
}
