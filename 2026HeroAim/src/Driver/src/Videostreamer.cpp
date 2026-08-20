#include "Videostreamer.hpp"
#include <iostream>
#include "Log.hpp"

namespace driver {

    VideoStreamer::VideoStreamer(int port, const std::string& ip, int jpeg_quality)
        : port_(port), local_ip_(ip), jpeg_quality_(jpeg_quality), server_running_(false)
    {
        server_ = std::make_unique<httplib::Server>();
        server_running_ = true;

        server_thread_ = std::thread([this]() {
            server_->Get("/stream", [this](const httplib::Request& req, httplib::Response& res) {
                // Each HTTP client keeps its own last delivered sequence number.
                auto last_frame_sequence = std::make_shared<uint64_t>(0);
                res.set_chunked_content_provider(
                    "multipart/x-mixed-replace; boundary=frame",
                    [this, last_frame_sequence](size_t offset, httplib::DataSink& sink) {
                        std::vector<uchar> frame_data;
                        {
                            std::unique_lock<std::mutex> lock(frame_mutex_);
                            frame_updated_.wait(lock, [this, last_frame_sequence] {
                                return !server_running_.load() ||
                                       (!current_frame_.empty() && frame_sequence_ > *last_frame_sequence);
                            });

                            if (!server_running_.load()) {
                                return false;
                            }

                            frame_data = current_frame_;
                            *last_frame_sequence = frame_sequence_;
                        }

                        const std::string boundary = "\r\n--frame\r\n";
                        const std::string header = "Content-Type: image/jpeg\r\nContent-Length: " +
                                                   std::to_string(frame_data.size()) + "\r\n\r\n";
                        if (!sink.write(boundary.data(), boundary.size()) ||
                            !sink.write(header.data(), header.size()) ||
                            !sink.write(reinterpret_cast<const char*>(frame_data.data()), frame_data.size())) {
                            return false;
                        }
                        return true;
                    });
            });

            std::cout << "VideoStreamer started at http://" << local_ip_ << ":" << port_ << "/stream" << std::endl;
            LOG(WARNING) << "[VideoStreamer] Listening at http://" << local_ip_ << ":" << port_ << "/stream";
            server_->listen(local_ip_.c_str(), port_);
        });
    }

    VideoStreamer::~VideoStreamer() {
        stop();
    }

    void VideoStreamer::stop() {
        if (server_running_.load()) {
            server_running_ = false;
            frame_updated_.notify_all();
            if (server_) {
                server_->stop();
            }
            if (server_thread_.joinable()) {
                server_thread_.join();
            }
        }
    }

    void VideoStreamer::setFrame(const cv::Mat& frame) {
        if (frame.empty() || !server_running_.load()) {
            return;
        }

        std::vector<uchar> buf;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);

        cv::imencode(".jpg", frame, buf, compression_params);
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_.swap(buf);
            ++frame_sequence_;
        }
        frame_updated_.notify_all();
    }
}
