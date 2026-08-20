#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <tuple>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include "Type.hpp"
#include "utils/include/Log.hpp"

using namespace lyutils;
using namespace cv;
using namespace std;
namespace tracker{

class MatcherWholeCar {
public:
    MatcherWholeCar(){}

    using Point = CXYD;
    inline double calcTheta(Point armor_point,cv::Rect2f car_rect,int id)
    {
        double armor_x = armor_point.cx;
        double center_x = car_rect.x + car_rect.width / 2;
        double theta;
        if(std::abs(armor_x - center_x) > car_rect.width / 2)
            theta = (armor_x < center_x)?-M_PI/2:M_PI/2;
        else
            theta = std::asin( (armor_x - center_x) / car_rect.width * 2);
        theta -= id * M_PI / 2;
        return theta;
    }
    template<typename T>
    map<int,T*> track(const vector<tuple<Point,cv::Rect2f, T*>>& currentPoints, const Time::TimeStamp& time) {
        if(currentPoints.empty())
        {
            missFrame++;
            return {};
        }
        if(missFrame > maxMissFrame) newTrack = true;
        missFrame = 0;
        double dt = 0.0;
        if(newTrack)
        {
            newTrack = false;
            oldtheta = 0;
            omega = 0;
            oldtheta = calcTheta(get<0>(currentPoints[0]), get<1>(currentPoints[0]), 0);
        }
        else
            dt = (time - last_time).toSeconds();
        //INFO("delta_time:{}",dt);
        last_time = time;
        double pred_theta = oldtheta + omega * dt;
        auto sortedPoints = currentPoints;
        sort(sortedPoints.begin(), sortedPoints.end(), [](const auto& a, const auto& b) {
            return get<0>(a).cx < get<0>(b).cx;
        });
        // after sort, the first element is the leftmost
        std::array<double,4> res_theta = {0,0,0,0};
        for(int i=0; i<4; i++)
        {
            for(int j=0; j < sortedPoints.size(); j++)
            {
                double diff = calcTheta(get<0>(sortedPoints[j]), get<1>(sortedPoints[j]), i + j)
                        - pred_theta;
                diff = std::remainder(diff,2*M_PI);
                res_theta[i] += std::abs(diff);
            }
        }
        int min_index = std::min_element(res_theta.begin(), res_theta.end()) - res_theta.begin();
        map<int,T*> result;
        double newtheta = 0;
        for(int j=0; j < sortedPoints.size(); j++)
        {
            result[(min_index + j)%4] = get<2>(sortedPoints[j]);
            newtheta += calcTheta(get<0>(sortedPoints[j]), get<1>(sortedPoints[j]), min_index + j);
        }
        newtheta /= sortedPoints.size();
        //INFO("newtheta,oldtheta:{},{}",newtheta,oldtheta);
        double newomega = omega;
        if(dt > 0.0001)
            newomega = std::remainder(newtheta - oldtheta, 2 * M_PI) / dt;
        if(!std::isfinite(newomega))
            newomega = omega;
        newomega = std::clamp(newomega, -maxOmega, maxOmega);
        omega = alpha * newomega + (1 - alpha) * omega;
        //INFO("newomega:{}",omega);
        oldtheta = newtheta;
        return result;
    }

private:
    bool newTrack = true;
    Time::TimeStamp last_time;
    double oldtheta = 0;
    double omega = 0;
    const double maxOmega = 10.0 * 2.0 * M_PI;
    int missFrame = 0;
    const int maxMissFrame = 10;
    const double alpha = 0.9;
};

}
