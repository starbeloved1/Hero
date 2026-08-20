#include "../include/predictor.hpp"

#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace predictor {
using lyutils::PredictorParam;
namespace {
constexpr double kPi = 3.14159265358979323846;
Eigen::Vector3d toYpd(const XYZ& xyz)
{
    const double xy = std::hypot(xyz.x, xyz.y);
    const double distance = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
    return {std::atan2(xyz.y, xyz.x), std::atan2(xyz.z, xy), distance};
}

bool isValidTrackResult(const tracker::TrackResult& trackResult)
{
    const XYZ xyz = static_cast<XYZ>(trackResult.location.xyz_imu);
    const double distance = std::sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
    return trackResult.car_id >= 0 &&
           trackResult.armor_id >= 0 && trackResult.armor_id <= 3 &&
           std::isfinite(xyz.x) && std::isfinite(xyz.y) && std::isfinite(xyz.z) &&
           std::isfinite(trackResult.yaw) &&
           distance > 0.1;
}
} // namespace


std::function<Predictions(Time::TimeStamp)> Predictor::predictFunc()
{
    return [this](Time::TimeStamp timestamp) { return predict(timestamp); };
}

Predictions Predictor::predict(Time::TimeStamp timestamp)
{
    Predictions predictions;
    std::lock_guard<std::mutex> lock(car_mutex);
    for (const auto& [car_id, car] : cars) {
        if (!car || !car->active() || car->diverged()) {
            continue;
        }
        predictions.push_back(car->toPrediction(car_id, timestamp));
    }
    return predictions;
}

void Predictor::update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp)
{
    std::map<int, std::vector<ArmorMeasurement>> grouped_measurements;
    for (const auto& trackResult : trackResults.first) {
        if (!isValidTrackResult(trackResult)) {
            continue;
        }

        ArmorMeasurement measurement;
        measurement.car_id = trackResult.car_id;
        measurement.armor_id = trackResult.armor_id;
        measurement.xyz = static_cast<XYZ>(trackResult.location.xyz_imu);
        measurement.yaw = std::remainder(trackResult.yaw, 2.0 * kPi);
        measurement.ypd = toYpd(measurement.xyz);
        grouped_measurements[measurement.car_id].push_back(measurement);
    }

    std::lock_guard<std::mutex> lock(car_mutex);

    for (auto& [car_id, car] : cars) {
        if (car && car->active()) {
            car->predict(timestamp);
            car->markLost();
        }
    }

    std::set<int> observed_ids;
    for (const auto& [car_id, measurements] : grouped_measurements) {
        if (measurements.empty()) {
            continue;
        }
        observed_ids.insert(car_id);
        const int detection_count = ++consecutive_detections[car_id];
        auto car_it = cars.find(car_id);
        const bool needs_init = car_it == cars.end() || !car_it->second ||
                                !car_it->second->active() || car_it->second->diverged() ||
                                car_it->second->lostCount() > PredictorParam::max_lost_count;
        if (needs_init) {
            if (detection_count < PredictorParam::min_consecutive_detections) {
                continue;
            }
            auto car = std::make_unique<MotionModel>();
            car->initialize(measurements.front(), timestamp);
            for (std::size_t i = 1; i < measurements.size(); ++i) {
                car->update(measurements[i]); 
            }
            cars[car_id] = std::move(car);
        } else {
            auto& car = car_it->second;
            for (const auto& measurement : measurements) {
                car->update(measurement);
            }
        }
    }

    for (auto it = consecutive_detections.begin(); it != consecutive_detections.end();) {
        if (observed_ids.find(it->first) == observed_ids.end()) {
            it = consecutive_detections.erase(it);
        } else {
            ++it;
        }
    }
    
    for (auto it = cars.begin(); it != cars.end();) {
        const auto& car = it->second;
        if (!car || car->diverged() || car->lostCount() > PredictorParam::max_lost_count) {
            if(!car){LOG(INFO)<<"[Predictor] no car";}
            else
            {if(car -> diverged()){LOG(INFO)<<"[Predictor] car diverged";}
            if(car->lostCount() > PredictorParam::max_lost_count){LOG(INFO)<<"[Predictor] lost count";}}
            LOG(INFO) << "[Predictor] remove target car=" << it->first;
            consecutive_detections.erase(it->first);
            it = cars.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace predictor
