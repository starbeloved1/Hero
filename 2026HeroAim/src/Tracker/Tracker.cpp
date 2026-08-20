#include "Tracker/Tracker.hpp"


#include "TrackerMatcher.hpp"
#include "TrackerMatcherWithWholeCar.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"
#include <algorithm> // std::remove_if, std::sort, std::clamp
#include <cmath>     // std::hypot, std::abs
#include <tuple>     // std::tuple, std::get
#include <map>       // std::map
#include <vector>    // std::vector
#include <set>       // std::set

namespace tracker {

using detector::Detections;
using detector::CarDetections;
using detector::Detection;
using detector::CarDetection;

auto createTracker() -> std::unique_ptr<Tracker> {
    return std::make_unique<Tracker>();
}

// This func will regard the last one as the final result.
//armors是类内成员变量，根据detections更新
void Tracker::merge(const Detections& detections, double threshold) 
{
    //clear method will put into GetTrackResult method
    //armors.clear();
    for(const auto& detection : detections) {
        ArmorXYV new_armor = {
            XYV(detection.corners[0].x, detection.corners[0].y),
            XYV(detection.corners[1].x, detection.corners[1].y),
            XYV(detection.corners[2].x, detection.corners[2].y),
            XYV(detection.corners[3].x, detection.corners[3].y)
        };
        
        auto dist = [](const XYV& p1, const XYV& p2) {
            return std::hypot(p1.x - p2.x, p1.y - p2.y);
        };
        
        auto it = std::remove_if(armors.begin(), armors.end(), 
            [&](const auto& existing_armor) {
                float avg_dist = 0;
                for(int i = 0; i < 4; i++) {
                    avg_dist += dist(existing_armor.first[i], new_armor[i]);
                }
                return avg_dist/4 < threshold;
            });
        auto it2 = std::remove_if(armors_gray.begin(), armors_gray.end(), 
            [&](const auto& existing_armor) {
                float avg_dist = 0;
                for(int i = 0; i < 4; i++) {
                    avg_dist += dist(existing_armor.first[i], new_armor[i]);
                }
                return avg_dist/4 < threshold;
            });
        armors.erase(it, armors.end());
        armors_gray.erase(it2, armors_gray.end());
        armors.push_back(std::make_pair(new_armor, detection.tag_id));
        dead[detection.tag_id] = 0;
    }
}

void Tracker::merge(const CarDetections& detections, double threshold) 
{
    for(const auto& detection : detections) {
        cv::Rect2f new_car_rect = detection.bounding_rect;
        auto it = std::remove_if(car_rects.begin(), car_rects.end(), 
            [&](const auto& existing_car_rect) {
                // Calculate Intersection over Union (IoU)
                float intersection_area = (std::get<0>(existing_car_rect) & new_car_rect).area();
                float union_area = std::get<0>(existing_car_rect).area() + new_car_rect.area() - intersection_area;
                float iou = intersection_area / union_area;
                return iou > threshold;
            });
        car_rects.erase(it, car_rects.end());
        car_rects.push_back(std::make_tuple(new_car_rect, -1, detection.tag_id));
    }
}

std::vector<cv::Rect2i> Tracker::calcROI(const std::vector<XYV>& projects, int width, int height, int camera_width, int camera_height) 
{
    std::vector<cv::Rect2i> rois;
    std::vector<bool> used(projects.size(), false);
    
    for(size_t i = 0; i < projects.size(); i++) {
        if(used[i] || !projects[i].visible) {
            continue;
        }
        
        // 计算ROI中心
        int center_x = static_cast<int>(projects[i].x);
        int center_y = static_cast<int>(projects[i].y);
        
        // 计算ROI区域，确保不超出边界
        int x = std::clamp(center_x - width/2, 0, camera_width - width);
        int y = std::clamp(center_y - height/2, 0, camera_height - height);
        
        cv::Rect2i roi(x, y, width, height);
        rois.push_back(roi);
        
        // 标记所有在当前ROI中的点
        for(size_t j = i + 1; j < projects.size(); j++) {
            if(roi.contains(cv::Point(static_cast<int>(projects[j].x), 
                                    static_cast<int>(projects[j].y)))) {
                used[j] = true;
            }
        }
    }
    
    return rois;
}

std::pair<double, double> calculateRotatedRect(const ArmorXYV& points_) {
    // 检查输入点数是否为4
    if (points_.size() != 4) {
        return {0.0, 1.0}; // 返回默认值
    }
    std::vector<cv::Point2f> points;
    for (const auto& point : points_) {
        points.push_back(cv::Point2f(point.x, point.y));
    }
    // 使用 OpenCV 的 minAreaRect 函数计算外接旋转矩形
    cv::RotatedRect rotatedRect = cv::minAreaRect(points);
    
    // 获取矩形的角度（OpenCV 中角度范围为 [-90, 0)）
    double angle = rotatedRect.angle;
    
    // 将角度转换为弧度制，并调整到 [0, π) 范围
    double theta = angle * CV_PI / 180.0;
    if (theta < 0) {
        theta += CV_PI; // 转换到 [0, π) 范围
    }
    
    // 获取矩形的宽和高
    double width = rotatedRect.size.width;
    double height = rotatedRect.size.height;
    
    if(width < height) {
        theta += CV_PI / 2;
        if(theta >= CV_PI) {
            theta -= CV_PI;
        }
    }
    double aspectRatio = width / height;
    
    return {theta, aspectRatio};
}

std::map<int, Matcher> matchers;

// 此算法依赖carid的准确性
//old_prediction: (XYV, car_id, armor_id)
TrackResults Tracker::getArmorTrackResult(const Time::TimeStamp& time, const ImuData& imu) 
{
    for(const auto& armor : armors_gray) {
        if(dead.find(armor.second) == dead.end()) {
            dead[armor.second] = maxDeadFrames + 1;
        }
    }
    for(auto& [car_id, dead_frames] : dead) {
        if(dead_frames <= maxDeadFrames) {
            dead_frames++;
        }
    }
    // A very strict condition for gray armor
    // Just for avoid situation: red_4 sentry & blue_4 sentry
    // are all hit and turn to gray
    // only satisfy conditions 1. dead <= maxDeadFrames
    // 2. deadArmor num == 1 
    // 3. no armor in armors
    // 's gray armor will be added to armors
    std::map<int, std::vector<std::pair<ArmorXYV, int>>> gray_armors_by_car;  // (car_id, [(armor, armor_id)])

    // 统计灰色装甲板
    for (const auto& armor : armors_gray) {
        gray_armors_by_car[armor.second].push_back(armor);
    }
    for (const auto& armor : armors) {
        gray_armors_by_car[armor.second].clear();  
    }
    for (auto& [car_id, gray_armors] : gray_armors_by_car) {
        if (gray_armors.size() == 1 && dead[car_id] <= maxDeadFrames) {
            armors.push_back(gray_armors[0]);
        }
    }



    TrackResults results;
    if(armors.empty()) return results;


    std::map<int,std::vector<std::pair<Point2f,const ArmorXYV*>>> params;//(car_id, centers)
    for(const auto& armor : armors) {
        location::Location center_location;
        center_location.cxy = {
            (armor.first[0].x + armor.first[1].x + armor.first[2].x + armor.first[3].x) / 4.0,
            (armor.first[0].y + armor.first[1].y + armor.first[2].y + armor.first[3].y) / 4.0
        };
        CXYD coord = center_location.getImuCXY(imu);
        cv::Point2f center(coord.cx/10.0, coord.cy/10.0);
        params[armor.second].push_back(std::make_pair(center, &armor.first));
    }

    for(auto& [car_id, param] : params) {
        if(matchers.find(car_id) == matchers.end()) {
            matchers.emplace(car_id, Matcher());
        }
        auto result = matchers[car_id].track(param, time, timeRatio);
        if(result.size() != param.size()) {
            LOG(ERROR) << "TrackerMatcher error: result size not equal to center size";
            continue;
        }
        for(auto& [id, armor_ptr] : result) {
            TrackResult track_result;
            track_result.armor = *armor_ptr;
            track_result.armor_id = id;//装甲板的序号1234，与car_id不同
            {
                cv::Point2f point1 = cv::Point2f(armor_ptr->at(0).x, armor_ptr->at(0).y);
                cv::Point2f point2 = cv::Point2f(armor_ptr->at(1).x, armor_ptr->at(1).y);
                cv::Point2f point3 = cv::Point2f(armor_ptr->at(2).x, armor_ptr->at(2).y);
                cv::Point2f point4 = cv::Point2f(armor_ptr->at(3).x, armor_ptr->at(3).y);
                track_result.rect = cv::boundingRect(std::vector<cv::Point2f>{point1, point2, point3, point4});
            }
            //INFO("Car {} Armor {} at ({}, {})", car_id, id, track_result.extra_center.x, track_result.extra_center.y);
            track_result.car_id = car_id;
            results.push_back(track_result);
        }
    }
    return results;
}


//算法描述：
// 【概念定义】
//   - 完整包含：当rect1完整包含rect2时，rect2在rect1内的面积占rect2面积的90%以上
//   - 装甲板的rect：每个armor对象中包含的矩形区域
//
// 【第一阶段：旧车辆矩形匹配】
// 1. 计算old_car_rects与car_rects间任意两矩形的距离（采用顶点距离差之和作为损失）
// 2. 迭代匹配过程：
//    a. 找出当前损失最小的矩形对
//    b. 将该对从匹配集合中移除
//    c. 将car_rects中的矩形标记为对应old_car_rect的编号
// 3. 当最小损失超过阈值时，停止匹配过程
// 4. 将car_rects中未匹配的矩形编号标记为-1
//
// 【第二阶段：装甲板与车辆矩形匹配】
// 5. 对每个装甲板armor：
//    a. 检查是否存在对应car_id的矩形：
//       - 若存在，判断该矩形是否完整包含armor的rect
//         * 若完整包含：无需操作
//         * 若不完整包含：将该矩形标记为未使用(car_id=-1)，执行步骤b
//       - 若不存在，直接执行步骤b
//    b. 查找可匹配的矩形：
//       - 优先在未使用的矩形(car_id=-1)中查找能完整包含armor的矩形
//       - 若找到，将该矩形的car_id设为armor的car_id
//       - 若未找到，在已使用的矩形中查找能完整包含armor的矩形
//       - 若找到，将该矩形的car_id设为armor的car_id
//       - 若仍未找到，放弃该装甲板的匹配
//
// 6. 结果输出：返回所有car_id不为-1的car_rects作为最终匹配结果
CarTrackResults Tracker::getCarTrackResult(const Time::TimeStamp& time, const ImuData& imu, const TrackResults& armor) 
{
    // 第一阶段：旧车辆矩形匹配
    std::vector<std::tuple<float, int, int>> distances; // (distance, old_idx, new_idx)
    for (size_t i = 0; i < old_car_rects.size(); i++) {
        for (size_t j = 0; j < car_rects.size(); j++) {
            cv::Rect2f old_rect = std::get<0>(old_car_rects[i]);
            cv::Rect2f new_rect = std::get<0>(car_rects[j]);
            
            float dist = 0.0;
            // 使用矩形四点差值之和作为距离
            dist += std::abs(old_rect.tl().x - new_rect.tl().x);
            //dist += std::abs(old_rect.tl().y - new_rect.tl().y);
            dist += std::abs(old_rect.br().x - new_rect.br().x);
            //dist += std::abs(old_rect.br().y - new_rect.br().y);
            distances.push_back(std::make_tuple(dist, i, j));
        }
    }
    
    const float max_distance_threshold = 400.0f; // 距离阈值，可根据实际情况调整
    std::sort(distances.begin(), distances.end()); // 按距离从小到大排序
    
    // 标记匹配情况
    std::vector<bool> old_matched(old_car_rects.size(), false);
    std::vector<bool> new_matched(car_rects.size(), false);
    std::vector<int> new_car_ids(car_rects.size(), -1);
    
    // 迭代匹配过程
    for (const auto& [dist, old_idx, new_idx] : distances) {
        if (dist > max_distance_threshold) break; // 当距离超过阈值时停止匹配
        
        if (!old_matched[old_idx] && !new_matched[new_idx]) {
            old_matched[old_idx] = true;
            new_matched[new_idx] = true;
            new_car_ids[new_idx] = std::get<1>(old_car_rects[old_idx]); // 记录匹配的car_id
            // LOG(INFO) << "MATCH carid: " << std::get<1>(old_car_rects[old_idx]) << " to carid: " << std::get<2>(car_rects[new_idx]);
        }
    }
    
    // 第二阶段：装甲板与车辆矩形匹配
    constexpr float contain_threshold = 0.9f; // "完整包含"的阈值
    
    for (const auto& armor_result : armor) {
        int car_id = armor_result.car_id;
        cv::Rect2f armor_rect = armor_result.rect;
        
        bool found = false;
        
        // 检查是否存在对应car_id的矩形
        for (size_t i = 0; i < car_rects.size(); i++) {
            if (new_car_ids[i] == car_id) {
                cv::Rect2f car_rect = std::get<0>(car_rects[i]);
                cv::Rect2f intersection = car_rect & armor_rect;
                float contained_ratio = armor_rect.area() > 0 ? intersection.area() / armor_rect.area() : 0;
                
                if (contained_ratio >= contain_threshold) {
                    // 完整包含，无需操作
                    found = true;
                    break;
                } else {
                    // 不完整包含，标记为未使用
                    new_car_ids[i] = -1;
                }
            }
        }
        
        // 如果没找到对应的车辆矩形，尝试寻找可匹配的矩形
        if (!found) {
            // 优先在未使用的矩形中查找
            for (size_t i = 0; i < car_rects.size(); i++) {
                if (new_car_ids[i] == -1) {
                    cv::Rect2f car_rect = std::get<0>(car_rects[i]);
                    cv::Rect2f intersection = car_rect & armor_rect;
                    float contained_ratio = armor_rect.area() > 0 ? intersection.area() / armor_rect.area() : 0;
                    
                    if (contained_ratio >= contain_threshold) {
                        new_car_ids[i] = car_id;
                        found = true;
                        break;
                    }
                }
            }
            
            // 如果仍未找到，在已使用的矩形中查找
            if (!found) {
                for (size_t i = 0; i < car_rects.size(); i++) {
                    if (new_car_ids[i] != -1 && new_car_ids[i] != car_id) {
                        cv::Rect2f car_rect = std::get<0>(car_rects[i]);
                        cv::Rect2f intersection = car_rect & armor_rect;
                        float contained_ratio = armor_rect.area() > 0 ? intersection.area() / armor_rect.area() : 0;
                        
                        if (contained_ratio >= contain_threshold) {
                            new_car_ids[i] = car_id;
                            found = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // 生成结果
    CarTrackResults results;
    
    // 添加匹配的车辆矩形结果
    for (size_t i = 0; i < car_rects.size(); i++) {
        if (new_car_ids[i] != -1) {
            CarTrackResult car_result;
            std::get<1>(car_rects[i]) = new_car_ids[i];
            car_result.car_id = new_car_ids[i];
            car_result.car_type = std::get<2>(car_rects[i]);
            car_result.bounding_rect = std::get<0>(car_rects[i]);
            results.push_back(car_result);
        }
    }
    
    
    // 更新old_car_rects为当前帧的car_rects，为下一帧做准备
    old_car_rects = car_rects;
    car_rects.clear();
    
    return results;
}

inline cv::Point2f XYV2Point2f(const XYV& xyv)
{
    return cv::Point2f(xyv.x, xyv.y);
}

TrackResults Tracker::initArmorTrackResult(const ImuData& imu)
{
    if(armors.empty()) return TrackResults();
    TrackResults results;
    TrackResult temp;
    for(auto& armor: armors)
    {
        temp.armor = armor.first;
        temp.rect = cv::boundingRect(std::vector<cv::Point2f>{
            XYV2Point2f(armor.first[0]),
            XYV2Point2f(armor.first[1]),
            XYV2Point2f(armor.first[2]),
            XYV2Point2f(armor.first[3])
        });
        temp.location.imu = imu;
        temp.location.cxy = {(armor.first[0].x + armor.first[1].x + armor.first[2].x + armor.first[3].x) / 4,
            (armor.first[0].y + armor.first[1].y + armor.first[2].y + armor.first[3].y) / 4};
        temp.car_id = armor.second;
        temp.armor_id = -1;
        results.push_back(temp);
    }

    return results;
}

std::map<int, MatcherWholeCar> matcher_whole_cars;

void Tracker::getArmorTrackResultWithWholeCar(const Time::TimeStamp& time, const ImuData& imu, const CarTrackResults& car_results, TrackResults& armor_results)
{
    std::map<int,std::vector<std::tuple<CXYD,cv::Rect2f,TrackResult*>>> params;//(car_id, centers)
    for(auto& armor : armor_results) {
        //search in carTrackResults
        auto it = std::find_if(car_results.begin(), car_results.end(), [&](const auto& car_result) {
            return car_result.car_id == armor.car_id;
        });
        if(it == car_results.end()) {
            continue; // 如果没有找到对应的车辆，跳过
        }
        params[armor.car_id].push_back(std::make_tuple(armor.location.cxy, it->bounding_rect, &armor));
    }

    for(auto& [car_id, param] : params) {
        if(matcher_whole_cars.find(car_id) == matcher_whole_cars.end()) {
            matcher_whole_cars.emplace(car_id, MatcherWholeCar());
        }
        auto result = matcher_whole_cars[car_id].track(param, time);
        if(result.size() != param.size()) {
            LOG(ERROR) << "TrackerMatcher error: result size not equal to center size";
            continue;
        }
        for(auto& [id, trackresult_ptr] : result) {
            trackresult_ptr->armor_id = id;
        }
    }
}

TrackResultPairs Tracker::getTrackResult(const Time::TimeStamp& time, const ImuData& imu) 
{
    auto armor_results = initArmorTrackResult(imu);
    //auto armor_results = getArmorTrackResult(time, imu);
    auto car_results = getCarTrackResult(time, imu, armor_results);
    getArmorTrackResultWithWholeCar(time, imu, car_results, armor_results);
    std::set<int> matched_car_ids;
    for (const auto& car : car_results) {
        matched_car_ids.insert(car.car_id);
    }
    armor_results.erase(
        std::remove_if(armor_results.begin(), armor_results.end(), [&](const auto& armor) {
            return armor.armor_id < 0 || armor.armor_id > 3 ||
                   matched_car_ids.find(armor.car_id) == matched_car_ids.end();
        }),
        armor_results.end());
    armors.clear();  // 清空当前帧的装甲板
    armors_gray.clear();
    return std::make_pair(armor_results, car_results);
}

bool Tracker::isDetected() 
{
    return !armors.empty();
}
} // namespace tracker
