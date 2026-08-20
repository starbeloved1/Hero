#ifndef BASIC_AIMER_HPP
#define BASIC_AIMER_HPP

#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "Type.hpp" 
#include "utils/include/Params.hpp"
#include "utils/include/Config.hpp"
#include "Driver/include/SerialPort.hpp"

namespace estimator{
    class BasicAimer{
        protected:
            bool is_first_shoot;
            int lost_cnt;
            bool right_clicked = false;
            bool last_right_clicked = false;
            double last_yaw;
            double last_pitch;
            double speed = GlobalParam::SHOOT_SPEED; // 弹速
            static constexpr double ANGLE_FILTER_RATIO_YAW = 0.3;
            static constexpr double ANGLE_FILTER_RATIO_PITCH = 0.7;

            void filterEulerAngle(double current_yaw, double current_pitch);
            void sendControlCommand(const SerialPortData& imu_data, 
                                Angle_t correct_angle);
        public:
            BasicAimer();
            virtual ~BasicAimer() = default;
            virtual Angle_t getShootCenter(const SerialPortData& imu_data) = 0;
            virtual double correctShootCenter(double theta, 
                                            double dz,
                                            double center_distance) = 0;
            virtual void judgeFireCondition(const SerialPortData& imu_data) = 0;
            
        };
}

#endif // BASIC_AIMER_HPP