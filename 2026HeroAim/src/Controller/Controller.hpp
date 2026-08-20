#pragma once
#include "Type.hpp"
#include "Driver/include/SerialPort.hpp" 
#include "Predictor/include/type.hpp"
#include "Solver/Type.hpp"
#include "utils/include/TimeStamp.hpp"
#include "utils/include/Params.hpp"
#include "Tracker/Type.hpp"
#include <stack>
#include <memory>
#include <functional>
#include <chrono>
#include <map>
// 核心计算函数和预测函数对象声明在 Controller 类内。


namespace controller
{
    using predictor::Armor;
    using predictor::Prediction;
    using predictor::Predictions;
    using driver::SerialPortData;
    using solver::ImuData;
    using lyutils::ControllerParam;
    using lyutils::GlobalParam;
    const double PI = 3.1415926;
    class Controller 
    {
    public:
        Controller();
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        // 设置当前帧观测数据（Solver 的原始 xyz）
        void setCurrentObservations(const tracker::TrackResults& trackResults);
        //
        ControlResult control(const SerialPortData& imu_data);
    private:
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        bool aim_new = false;
        bool aiming = false;
        const int waitFrame = 5;
        int accumulate_aim_request = 0;
        std::pair<int, int> aim_armor_id = {-1, -1}; // (car_id, armor_id)
        int max_iter = 100;
        double tol = 1e-6;
        bool judgeAimNew(bool request);
        // 弹道解算：FLU 坐标系输入（x=前，y=左，z=上），输出 pitch/yaw(rad) 和飞行时间
        bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        std::chrono::duration<double> flyTime{0.0};
        double bullet_speed;
        double system_delay;  // 系统延迟(s)：来自 OutpostParam::tmp_time，与 AntiTop 一致
        double armor_fire_window;   // 低速开火窗口(°)，从配置文件读取
        double speed_fire_window;   // 高速开火窗口(°)，从配置文件读取
        int phase_confirm_frames;   // 低速确认帧数，从配置文件读取
        int speed_confirm_frames;   // 高速确认帧数，从配置文件读取
        // 加速度判断相关参数
        double accel_threshold;        // 加速度阈值(m/s²)，超过则切换到观测跟踪模式
        double accel_stable_threshold; // 加速度稳定阈值(m/s²)，低于此值恢复预测模式
        int accel_stable_frames;       // 加速度稳定确认帧数
        int accel_stable_count = 0;    // 加速度稳定计数器
        bool high_accel_mode = false;  // 高加速度模式标志（使用观测而非预测）
        bool mouse_require;
        bool last_right_clicked = false;
        double pic_camera_x = 640.0;
        double pic_camera_y = 512.0;
        Time::TimeStamp last_shoot_time;  // 在构造函数中正确初始化
        bool first_shoot = true;  // 首次开枪标记，避免 TimeStamp::min() 溢出问题
        int phase_ready_count = 0;
        
        // 当前帧观测 xyz（key = car_id * 10 + armor_id）
        std::map<int, XYZ> current_obs_xyz;
    };

    std::shared_ptr<Controller> createController();
    //std::unique_ptr<Controller>createController2(param::Param json_param);
} // namespace controller
