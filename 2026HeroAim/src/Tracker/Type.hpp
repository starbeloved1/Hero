#pragma once
#include "utils/include/TimeStamp.hpp"
#include "utils/include/Location.hpp"
#include <array>
#include <opencv2/opencv.hpp>
#include <vector>

namespace tracker
{
    typedef std::array<XYV, 4> ArmorXYV;
    struct TrackResult
    {
        ArmorXYV armor;
        cv::Rect2f rect;
        location::Location location;
        double yaw;
        bool visible = true;
        int armor_id;
        int car_id;
        TrackResult(const ArmorXYV& armor, int car_id, int armor_id) : armor(armor), armor_id(armor_id), car_id(car_id) {};
        TrackResult() : armor{XYV(0,0),XYV(0,0),XYV(0,0),XYV(0,0)}, visible(false), armor_id(-1), car_id(0) {};
    };
    typedef std::vector<TrackResult> TrackResults;
    struct CarTrackResult
    {
        int car_id;
        int car_type;
        cv::Rect2f bounding_rect;
        CarTrackResult(): car_id(-1), car_type(-1), bounding_rect(0, 0, 0, 0) {};
    };
    typedef std::vector<CarTrackResult> CarTrackResults;
    typedef std::pair<TrackResults, CarTrackResults> TrackResultPairs;
}
