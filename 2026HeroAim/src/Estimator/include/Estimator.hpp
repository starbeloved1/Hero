#pragma once
#include <algorithm>
#include <memory>
#include <vector>
#include "Solver/Solver.hpp"
#include "NormalAim.hpp"
#include "AntiTop.hpp"
#include "utils/include/Params.hpp"
#include "utils/include/Config.hpp"
#include "Driver/include/SerialPort.hpp"

using namespace std;
using namespace driver;
using namespace lyutils;

namespace estimator{

    class Estimator {
    public:
        explicit Estimator(std::shared_ptr<solver::Solver> solver);
        void startRun(vector<Armor> &armors, double delta_t, const SerialPortData &imu_data,
            SerialPort *SerialPort_,int mode);
        
    private:
        std::shared_ptr<solver::Solver> solver_;
        std::unique_ptr<NormalAim> aimer = std::make_unique<NormalAim>();
        std::unique_ptr<AntiTop> antiTopper;
        int last_mode = 1;
        bool is_init=false;
        int last_right_clicked = 0;

    };

}