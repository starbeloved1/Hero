#pragma once
#include <boost/asio.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <tuple>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include "Params.hpp"


namespace driver {
    struct Params_ToTCP {
        mutable std::mutex data_mutex;
        mutable std::queue<std::tuple<double, double, double, double>> pose_queue; 
        std::condition_variable data_cv; // 数据到达通知
        
        // 线程安全的数据添加方法
        void addPose(double x, double y, double z, double yaw) {
            std::unique_lock<std::mutex> lock(data_mutex);
            pose_queue.emplace(x, y, z, yaw);
            data_cv.notify_one();
        }

        // 线程安全的数据获取方法
        bool getPose(double& x, double& y, double& z, double& yaw) const{
            std::unique_lock<std::mutex> lock(data_mutex);
            if (!pose_queue.empty()) {
                auto data = pose_queue.front();
                pose_queue.pop();
                std::tie(x, y, z, yaw) = data;
                return true;
            }
            return false;
        }
    };

    class TCPServer {
    public:
        TCPServer(int port, Params_ToTCP& params) 
            : port_(port), params_(params), running_(false) {}
        
        void start() {
            running_ = true;
            server_thread_ = std::thread(&TCPServer::run, this);
        }

        void stop() {
            running_ = false;
            if (server_thread_.joinable()) server_thread_.join();
        }

    private:
        void run() {
            // 创建TCP套接字
            int server_fd = socket(AF_INET, SOCK_STREAM, 0);

            // 设置地址重用
            int opt = 1;
            setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            // 绑定端口
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(port_);
            bind(server_fd, (sockaddr*)&address, sizeof(address));
            DLOG(INFO)<<"TCP Server Start";

            // 开始监听
            listen(server_fd, 10);
            
            while (running_) {
                // 接受新连接
                sockaddr_in client_addr{};
                socklen_t client_len = sizeof(client_addr);
                int client_sock = accept(server_fd, (sockaddr*)&client_addr, &client_len);

                client_threads_.emplace_back([this, client_sock] {
                    char buffer[128];
                    while (true) {
                        int len = recv(client_sock, buffer, sizeof(buffer), 0);
                        if (len <= 0) {
                            if (len == 0) DLOG(INFO) << "Client closed connection gracefully";
                            else DLOG(ERROR) << "Recv error:" << strerror(errno);
                            break; 
                        }
                        
                        double x, y, z, yaw;
                        if (sscanf(buffer, "%lf,%lf,%lf,%lf", &x, &y, &z, &yaw) == 4) {
                            params_.addPose(x, y, z, yaw);  // 线程安全写入共享数据
                            SlamParam::x = x;
                            SlamParam::y = y;
                            SlamParam::z = z;
                        }
                        DLOG(INFO)<<"Writing pose data:" << x << "," << y 
                         << "," << z << "," << yaw;
                    }
                    close(client_sock);
                });
            }
            close(server_fd);
        }

        int port_;
        Params_ToTCP& params_;
        std::atomic<bool> running_;
        std::thread server_thread_;
        std::vector<std::thread> client_threads_;
    };
}