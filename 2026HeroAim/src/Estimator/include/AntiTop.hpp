#pragma once
#ifndef ANTI_TOP_HPP
#define ANTI_TOP_HPP

#include "Solver/Solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "utils/include/Params.hpp"
#include "utils/include/Config.hpp"
#include "Driver/include/SerialPort.hpp"
#include "Type.hpp"
#include "BasicAimer.hpp"

using namespace std;
using namespace driver;
using namespace lyutils;
using cv::Point2f;
using cv::Point3d;
using cv::Point3f;

namespace estimator {

    class AntiTop : public BasicAimer {
        private:
            bool processFrame(std::vector<Armor>& armors,
                              const SerialPortData& imu_data,
                              SerialPort* serial_port);

            bool selectTarget(std::vector<Armor>& armors);

            bool updateTracking();
            bool updateDirection();
            bool updateRotationCenter();
            void updateZCalibration();
            bool tryBuildZMapFromSamples();

            void updateZoneState();
            bool observeZoneOrder();
            double recentZMedian() const;
            int matchZLayer(double observed_z, double* out_diff = nullptr) const;
            void updateTargetAimZ(double observed_z);
            double getTargetAimZ() const;
            int calculateAvgPeriodMs() const;

            void tryInitiateFiring(std::chrono::steady_clock::time_point now);
            void executeControl(const SerialPortData& imu_data, SerialPort* serial_port);

            void resetCalibrationState(bool reset_direction);
            void resetRuntimeState(bool reset_direction);

            Angle_t calculateBallistics(Point3d target_point, double* out_flyTime = nullptr);

            Armor last_armor;
            Armor tracking_armor;
            std::vector<float> x_diff_history;
            std::vector<Point2f> center_xy_window;

            Point3d shootCenter = Point3d(0, 0, 0);

            static constexpr double Z_CALIB_MATCH_THRESHOLD = 0.05;
            static constexpr double Z_RUNTIME_MATCH_THRESHOLD = 0.05;
            static constexpr double Z_LAYER_GAP_MIN = 0.08;
            static constexpr double Z_LAYER_GAP_MAX = 0.12;
            static constexpr double Z_HIST_BIN_SIZE = 0.01;
            static constexpr size_t Z_CALIB_START_CENTER_SAMPLES = 40;
            static constexpr size_t Z_CALIB_MIN_SAMPLES = 240;
            static constexpr size_t Z_CALIB_MAX_SAMPLES = 360;
            static constexpr size_t Z_LAYER_MIN_SAMPLES = 12;
            static constexpr size_t ZONE_Z_MEDIAN_FRAMES = 20;
            std::vector<double> calib_z_samples;
            std::vector<double> recent_z_buffer;
            double target_aim_z = 0.0;
            static constexpr float DIRECTION_THRESHOLD = 0.0002f;

            double tmp_time = OutpostParam::tmp_time;
            double time_bias = OutpostParam::time_bias;
            double time_bias_inverse = OutpostParam::time_bias_inverse;

            int Direction = -1; // 1: CW, 0: CCW, -1: unknown

            bool in_shoot_zone = false;
            bool has_last_enter_time = false;
            std::chrono::steady_clock::time_point last_enter_time;
            std::vector<std::chrono::milliseconds> zone_period_history;
            int avg_ms = -1;

            int last_zone_index = -1;

            enum class PitchLockState { FITTING, LOCKED_AIM, FIRE_COUNTDOWN };
            PitchLockState pitch_state = PitchLockState::FITTING;
            std::chrono::steady_clock::time_point fire_start_time;
            int fire_delay_ms = 0;
            static constexpr int LOWEST_Z_INDEX = 0;

            static constexpr int MAX_WINDOW_SIZE = 300;
            static constexpr int MAX_DIRE_SIZE = 10;
            static constexpr int ENTER_ZONE_THRESHOLD = 25;
            static constexpr int OUT_ZONE_THRESHOLD = 35;
            static constexpr int MIN_TIME_INTERVAL = 500;
            static constexpr int MAX_TIME_INTERVAL = 1000;
            static constexpr size_t MAX_PERIOD_HISTORY = 20;
            solver::Solver* solver_;

        public:
            static constexpr double zSwitchInThreshold() { return Z_RUNTIME_MATCH_THRESHOLD; }
            AntiTop(solver::Solver* solver);
            bool startAim(vector<Armor>& armors,
                          const SerialPortData& imu_data,
                          SerialPort* SerialPort_,
                          bool is_init);
            Angle_t getShootCenter(const SerialPortData& imu_data) override;
            void judgeFireCondition(const SerialPortData& imu_data) override;
            double correctShootCenter(double theta, double dz, double value) override {
                return 0.0;
            }
    };
}

#endif
