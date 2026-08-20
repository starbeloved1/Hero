#include "../include/Type.hpp"
#include "../include/BasicAimer.hpp"

namespace estimator {
        BasicAimer::BasicAimer() :
        is_first_shoot(false),
        lost_cnt(0),
        right_clicked(0),
        last_right_clicked(0),
        last_yaw(0.0),
        last_pitch(0.0),
        speed(GlobalParam::SHOOT_SPEED) {}

    void BasicAimer::filterEulerAngle(double current_yaw, double current_pitch) {
        bool useFilter = !is_first_shoot;
        if(abs(SerialParam::send_data.yaw - last_yaw )> 3.0 || 
        abs(SerialParam::send_data.pitch - last_pitch) > 1.0) {
            useFilter = false;
        }
        if(useFilter) {
            SerialParam::send_data.yaw = last_yaw * ANGLE_FILTER_RATIO_YAW + SerialParam::send_data.yaw * (1 - ANGLE_FILTER_RATIO_YAW);
            SerialParam::send_data.pitch = last_pitch * ANGLE_FILTER_RATIO_PITCH + SerialParam::send_data.pitch * (1 - ANGLE_FILTER_RATIO_PITCH);
        }
        last_yaw = SerialParam::send_data.yaw;
        last_pitch = SerialParam::send_data.pitch;
    }

    void BasicAimer::sendControlCommand(const SerialPortData& imu_data,
                                    Angle_t correct_angle) {

        SerialParam::send_data.shootStatus = 1;
        SerialParam::send_data.pitch = correct_angle.pitch;

        double tmp_yaw = correct_angle.yaw / M_PI * ANGLE_FULL_CIRCLE;
        while (abs(tmp_yaw - imu_data.yaw) > ANGLE_FULL_CIRCLE/2) {
            if (tmp_yaw - imu_data.yaw >= ANGLE_FULL_CIRCLE/2)
                tmp_yaw -= ANGLE_FULL_CIRCLE;
            else
                tmp_yaw += ANGLE_FULL_CIRCLE;
        }
        SerialParam::send_data.yaw = tmp_yaw;
        filterEulerAngle(tmp_yaw, correct_angle.pitch);
        LOG(INFO) << "                                        send yaw: " << SerialParam::send_data.yaw
                    << "  send pitch: " << SerialParam::send_data.pitch;
        LOG(INFO) << "                                        recv yaw: " << imu_data.yaw
                    << "  recv pitch: " << imu_data.pitch;
    }

}
