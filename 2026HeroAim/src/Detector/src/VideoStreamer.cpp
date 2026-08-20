#include "VideoStreamer.hpp"
#include "Log.hpp"

namespace detector{

    VideoStreamer::VideoStreamer(){
        server_ = std::make_unique<httplib::Server>();
        server_running_ = true;
        server_thread_ = std::thread([this]() {
            server_->Get("/stream", [this](const httplib::Request& req, httplib::Response& res) {
                res.set_chunked_content_provider(
                    "multipart/x-mixed-replace; boundary=frame",
                    [this](size_t offset, httplib::DataSink& sink) {
                        std::vector<uchar> frame_data;
                        {
                            std::lock_guard<std::mutex> lock(frame_mutex_);
                            frame_data = current_frame_;
                        }

                        if (!frame_data.empty()) {
                            std::string boundary = "\r\n--frame\r\n";
                            std::string header = "Content-Type: image/jpeg\r\n\r\n";
                            sink.write(boundary.data(), boundary.size());
                            sink.write(header.data(), header.size());
                            sink.write(reinterpret_cast<const char*>(frame_data.data()), frame_data.size());
                        }
                        return true;
                    });
            });

            LOG(WARNING) << "[VideoStreamer] Listening at http://" << local_ip_ << ":" << port_ << "/stream";
            server_->listen(local_ip_.c_str(), port_);
        });
    }

    VideoStreamer::~VideoStreamer() {
        server_running_ = false;
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    void VideoStreamer::setFrame(const cv::Mat& frame) {
        std::vector<uchar> buf;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);
        // 在frame上绘制SerialParam::recv_data数据和send_data
        if (GlobalParam::SHOW_SERIAL) {
            float recv_pitch = 0.0f;
            float recv_yaw = 0.0f;
            {
                std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
                recv_pitch = SerialParam::recv_data.pitch;
                recv_yaw = SerialParam::recv_data.yaw;
            }
            recv_pitches.push_back(recv_pitch);
            recv_yaws.push_back(recv_yaw);
            send_pitches.push_back(SerialParam::send_data.pitch);
            send_yaws.push_back(SerialParam::send_data.yaw);
            shoot_flags.push_back(SerialParam::send_data.shootStatus);
            right_clickes.push_back(SerialParam::right_clicked?1:0);
            
            if (recv_pitches.size() > 500) {
                recv_pitches.erase(recv_pitches.begin());
            }
            if (recv_yaws.size() > 500) {
                recv_yaws.erase(recv_yaws.begin());
            }
            if (send_pitches.size() > 500) {
                send_pitches.erase(send_pitches.begin());
            }
            if (send_yaws.size() > 500) {
                send_yaws.erase(send_yaws.begin());
            }
            if(shoot_flags.size()>500){
                shoot_flags.erase(shoot_flags.begin());
            }
            if(right_clickes.size()>500){
                right_clickes.erase(right_clickes.begin());
            }

            // 绘制曲线
            cv::putText(frame, "Recv Pitch", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            for (int i = 0; i < recv_pitches.size() - 1; ++i) {
                cv::line(frame, cv::Point(i, 100 - recv_pitches[i]), cv::Point(i + 1, 100 - recv_pitches[i + 1]), cv::Scalar(0, 0, 255), 1);
            }
            cv::putText(frame, "Recv Yaw", cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            for (int i = 0; i < recv_yaws.size() - 1; ++i) {
                cv::line(frame, cv::Point(i, 200 - recv_yaws[i]), cv::Point(i + 1, 200 - recv_yaws[i + 1]), cv::Scalar(0, 255, 0), 1);
            }
            cv::putText(frame, "Send Pitch", cv::Point(10, 270), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            for (int i = 0; i < send_pitches.size() - 1; ++i) {
                cv::line(frame, cv::Point(i, 300 - send_pitches[i]), cv::Point(i + 1, 300 - send_pitches[i + 1]), cv::Scalar(255, 0, 0), 1);
            }
            cv::putText(frame, "Send Yaw", cv::Point(10, 370), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            for (int i = 0; i < send_yaws.size() - 1; ++i) {
                cv::line(frame, cv::Point(i, 400 - send_yaws[i]), cv::Point(i + 1, 400 - send_yaws[i + 1]), cv::Scalar(255, 255, 0), 1);
            }
            cv::putText(frame,"shoot_flag",cv::Point(10,470),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,255),1);
            for(int i=0;i<shoot_flags.size()-1;++i){
                cv::line(frame, cv::Point(i, 500-20*(shoot_flags[i])), cv::Point(i + 1, 500-20*(shoot_flags[i + 1])), cv::Scalar(255, 255, 255), 1);
                cv::line(frame,cv::Point(i,500),cv::Point(i+1,500),cv::Scalar(0, 0, 255), 1);
            }
            cv::putText(frame,"right_clicked",cv::Point(10,570),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(128,0,128),1);
            for(int i=0;i<right_clickes.size()-1;++i){
                cv::line(frame, cv::Point(i, 600-20*(right_clickes[i])), cv::Point(i + 1, 600-20*(right_clickes[i + 1])), cv::Scalar(255, 0, 255), 1);
                cv::line(frame,cv::Point(i,600),cv::Point(i+1,600),cv::Scalar(0, 0, 255), 1);
            }
            

        }
        // //SerialParam::direction
        // if(GlobalParam::MODE == 2){
            cv::putText(frame, "Direction: " + std::to_string(SerialParam::direction), cv::Point(10, 450), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::circle(frame, SerialParam::shoot_center, 10, cv::Scalar(255, 255, 255), -1);
            cv::putText(frame, "Shoot Center", SerialParam::shoot_center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
        //}

        cv::imencode(".jpg", frame, buf, compression_params);
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_.swap(buf);
        }
    }
}
