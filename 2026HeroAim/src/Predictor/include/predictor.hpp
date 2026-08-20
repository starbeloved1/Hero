#pragma once

#include "MotionModel.hpp"
#include "Tracker/Type.hpp"
#include "type.hpp"
#include "utils/include/TimeStamp.hpp"

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace predictor {

using tracker::TrackResultPairs;

class Predictor {
public:
    std::function<Predictions(Time::TimeStamp)> predictFunc();
    Predictions predict(Time::TimeStamp timestamp);
    void update(const TrackResultPairs& trackResults, const Time::TimeStamp& timestamp);
    bool Stable() const { return true; }

private:
    std::mutex car_mutex;
    std::map<int, std::unique_ptr<MotionModel>> cars;
    std::map<int, int> consecutive_detections;
};


} // namespace predictor
