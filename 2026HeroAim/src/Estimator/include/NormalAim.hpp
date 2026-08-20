#pragma once
#ifndef NORMAL_AIM_HPP
#define NORMAL_AIM_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include "utils/include/Params.hpp"
#include "utils/include/Config.hpp"
#include "Driver/include/SerialPort.hpp"
#include "Type.hpp"    
#include "BasicAimer.hpp"


using namespace driver;
using namespace lyutils;

namespace estimator{

    class NormalAim : public BasicAimer {
        private:
            Armor current_armor;
            Armor last_armor;
            int top_pri;
            std::vector<Armor> candidates;
        
            bool selectTarget(std::vector<Armor>& armors);
            bool mainTainStatus();


        public:
            NormalAim()= default;
            bool startAim(std::vector<Armor>& armors, 
                        double delta_t,
                        const SerialPortData& imu_data,
                        SerialPort* SerialPort_);
        
            Angle_t getShootCenter(const SerialPortData& imu_data) override;
            double correctShootCenter(double theta,
                                    double dz,
                                    double center_distance) override;
            void judgeFireCondition(const SerialPortData& imu_data) override;
        };
}
#endif // NORMAL_AIM_HPP
