#include "Estimator/include/Estimator.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp" // 使用 SerialParam
#include <chrono> // 计算时间戳

namespace estimator{

    Estimator::Estimator(std::shared_ptr<solver::Solver> solver) : solver_(solver) {
        antiTopper = std::make_unique<AntiTop>(solver_.get());
    }

    void Estimator::startRun(Armors &armors,double delta_t, const SerialPortData& imu_data,SerialPort *SerialPort_,int mode){

        SerialParam::right_clicked = imu_data.right_clicked;
        is_init = (imu_data.right_clicked == 1 && last_right_clicked == 0);
        if (is_init) {
            SerialParam::send_data.shootStatus = 0;
        }
        bool getTarget = false;
        solver_->PreProcessArmor(armors,imu_data);
        if(mode == 2){// antitop
            getTarget = antiTopper->startAim(armors,imu_data, SerialPort_,is_init);
        }

        else{
            getTarget = aimer->startAim(armors, delta_t,imu_data, SerialPort_);
            if(getTarget&&imu_data.yaw!=0&&imu_data.pitch){
              SerialPort_->writeData(&SerialParam::send_data);
          }
        }
        // LOG(WARNING) << "send yaw: " << SerialParam::send_data.yaw << " send pitch: " << SerialParam::send_data.pitch;
        last_mode = mode;
        last_right_clicked = imu_data.right_clicked;

    }
 
}
