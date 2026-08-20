#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <tuple>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include "utils/include/Log.hpp" 
#include "utils/include/TimeStamp.hpp" 

using namespace lyutils;
using namespace cv;
using namespace std;
// TODO:
// 1.在实际使用过程中，将滤波改为时间间隔不定的情况应该会增强表现（反之则可能出现bug）**
// 2.匹配算法有小问题，匹配时先从检测到的点开始匹配，在转速过快情况下会出现错配。
//  应该改为将预测器点加入一起匹配。
// 3.可能需要稳定性判断，即程序发现波动过大自动重启等。
namespace tracker{

struct Track {
    int id;
    KalmanFilter kf;
    Point2f lastPoint; // 滤波后的坐标，用于内部预测更新
    Point2f rawPoint;  // 最近一次收到的原始测量坐标
    int livedFrames = 0;
    int missedFrames;
    bool _matched;

    Track(int id_, const Point2f& pt, const Point2f& vec_init=cv::Point2f(0,0)) : id(id_), missedFrames(0), lastPoint(pt), rawPoint(pt), _matched(false) {
        // 状态为 [x, y, vx, vy]
        kf = KalmanFilter(4, 2, 0);
        // 状态转移矩阵 (dt假设为1)
        kf.transitionMatrix = (Mat_<float>(4,4) << 
            1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1);
        // 观测矩阵：直接观测位置
        kf.measurementMatrix = Mat::zeros(2, 4, CV_32F);
        kf.measurementMatrix.at<float>(0,0) = 1.0f;
        kf.measurementMatrix.at<float>(1,1) = 1.0f;
        // 过程噪声
        setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
        // 测量噪声
        setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
        // 误差协方差矩阵
        setIdentity(kf.errorCovPost, Scalar::all(1));
        // 初始状态：[x, y, 0, 0]
        kf.statePost = (Mat_<float>(4,1) << pt.x, pt.y, vec_init.x, vec_init.y);
    }
};

class Matcher {
public:
    Matcher() : sameLabelError(70.0f), diffLabelError(70.0f), alpha(0.2f), autoFillMode(NO_FILL) {}

    using Point = cv::Point2f;
    // 追踪函数，输入当前帧的原始测量坐标，
    // 对于已匹配的点直接返回传入的原始坐标，
    // 对于新出现的点，根据左右极值关系分配编号：
    //   右侧的新点编号为 (rightmost + 1) mod 4，
    //   左侧的新点编号为 (leftmost - 1 + 4) mod 4。
    // 修改 Observer::track()，增加 dt 参数，并在预测时更新转移矩阵
    template<typename T>
    map<int,const T*> track(const vector<pair<Point, const T*>>& currentPoints, const Time::TimeStamp& time, double timeRatio) {
        double dt = 0.0;
        if(newTrack)
            newTrack = false;
        else
            dt = (time - last_time).toSeconds() / timeRatio;
        last_time = time;
        map<int, Point> output; 
        std::vector<int> return_result(currentPoints.size(), -1);
        vector<bool> used(currentPoints.size(), false);

        // 计算自适应距离门限
        float adaptiveThreshold = sameLabelError * 0.6f + diffLabelError * 0.4f;
        LOG(INFO) << "Adaptive Threshold: " << adaptiveThreshold;
        LOG(INFO) << "Same Label Error: " << sameLabelError;
        LOG(INFO) << "Diff Label Error: " << diffLabelError;

        // 利用卡尔曼滤波预测所有轨迹的下一位置
        for (auto &trk : tracks) {
            // 更新状态转移矩阵，使用 dt 而不是固定的 1
            trk.kf.transitionMatrix = (Mat_<float>(4, 4) <<
                1, 0, static_cast<float>(dt), 0,
                0, 1, 0, static_cast<float>(dt),
                0, 0, 1, 0,
                0, 0, 0, 1);

            // 根据 dt 动态更新过程噪声协方差矩阵 Q
            // 对于状态 [x, y, vx, vy]，通常采用以下 Q 矩阵形式：
            // Q = sigma^2 * [ [dt^3/3,      0, dt^2/2,      0],
            //                 [     0, dt^3/3,      0, dt^2/2],
            //                 [dt^2/2,      0,      dt,      0],
            //                 [     0, dt^2/2,      0,     dt] ]
            float sigma = 1e-2f;
            float dt2 = static_cast<float>(dt * dt);
            float dt3 = static_cast<float>(dt2 * dt);
            trk.kf.processNoiseCov = (Mat_<float>(4,4) <<
                sigma * dt3 / 3.0f, 0,                sigma * dt2 / 2.0f, 0,
                0,                sigma * dt3 / 3.0f, 0,                sigma * dt2 / 2.0f,
                sigma * dt2 / 2.0f, 0,                sigma * dt,       0,
                0,                sigma * dt2 / 2.0f, 0,                sigma * dt );
            // 新增功能：根据 sameLabelError 实时更新测量噪声矩阵 R
            // 这里通过一个比例系数将 sameLabelError 映射为测量噪声，保证噪声下限不低于1e-1
            float r = std::max(1e-1f, static_cast<float>(sameLabelError * 0.5f));
            setIdentity(trk.kf.measurementNoiseCov, Scalar::all(r));
            Mat prediction = trk.kf.predict();
            // 更新滤波坐标，用于匹配
            trk.lastPoint = Point2f(prediction.at<float>(0), prediction.at<float>(1));
        }

        // 生成候选匹配对：使用 std::tuple<float, int, int> 存储：距离、track索引、检测点索引
        vector<tuple<float, int, int>> candidateMatches;
        for (size_t i = 0; i < currentPoints.size(); i++) {
            Point2f pt = currentPoints[i].first;
            for (size_t j = 0; j < tracks.size(); j++) {
                float d = norm(pt - tracks[j].lastPoint);
                if (d < adaptiveThreshold) {
                    candidateMatches.push_back(make_tuple(d, static_cast<int>(j), static_cast<int>(i)));
                }
            }
        }

        // 按距离从小到大排序
        sort(candidateMatches.begin(), candidateMatches.end(), [](const tuple<float, int, int>& a, const tuple<float, int, int>& b) {
            return get<0>(a) < get<0>(b);
        });

        vector<bool> detectedMatched(currentPoints.size(), false);
        vector<bool> trackInstanceMatched(tracks.size(), false);
        // 额外维护一个 set 来记录匹配过的 track.id（相同的 track.id 视为同一个对象）
        set<int> matchedTrackIds;

        for (const auto &m : candidateMatches) {
            float d = get<0>(m);
            int trackIdx = get<1>(m);
            int detectIdx = get<2>(m);
            // 如果该检测点未匹配，且此 track 实例未匹配，同时该 track.id 也还未被匹配，则执行匹配
            if (!detectedMatched[detectIdx] &&
                !trackInstanceMatched[trackIdx] &&
                (matchedTrackIds.find(tracks[trackIdx].id) == matchedTrackIds.end())) {
                
                detectedMatched[detectIdx] = true;
                trackInstanceMatched[trackIdx] = true;
                matchedTrackIds.insert(tracks[trackIdx].id);
                
                // 匹配成功，更新对应 Track
                Track &trk = tracks[trackIdx];
                Mat measurement = (Mat_<float>(2,1) << currentPoints[detectIdx].first.x, currentPoints[detectIdx].first.y);
                trk.kf.correct(measurement);
                trk.lastPoint = currentPoints[detectIdx].first; // 更新滤波值（内部使用）
                trk.rawPoint = currentPoints[detectIdx].first;  // 更新原始测量值
                trk.livedFrames++;
                trk.missedFrames = 0;
                output[trk.id] = currentPoints[detectIdx].first; // 返回原始测量值
                return_result[detectIdx] = trk.id;
                used[detectIdx] = true;
                trk._matched = true;
                // 使用平滑滤波更新 sameLabelError
                sameLabelError = alpha * d + (1 - alpha) * sameLabelError;
            }
        }

        // 对未匹配到的轨迹，记录丢帧，并返回上一次的原始测量值
        for (auto it = tracks.begin(); it != tracks.end(); ) {
            if (!it->_matched) {
                it->missedFrames++;
            }
            it->_matched = false;
            if (it->missedFrames > maxMissedFrames) {
                LOG(INFO) << "Track " << it->id << " is lost";
                LOG(INFO) << "Lived Frames: " << it->livedFrames;
                averageLiveFrames = alpha * it->livedFrames + (1 - alpha) * averageLiveFrames;
                it = tracks.erase(it);
            } else {
                if(autoFillMode == FILTER_PREDICT)
                    output[it->id] = it->lastPoint;
                else if(autoFillMode == LAST_DETECT)
                    output[it->id] = it->rawPoint; //高速下容易失配。
                else if(autoFillMode == NO_FILL)
                    output[it->id] = it->lastPoint; 
                else
                    throw std::runtime_error("Invalid autoFillMode");
                ++it;
            }
        }

        // 收集已有轨迹的编号及其x坐标，用于边界判断
        vector<pair<int, float>> currentOrder;
        for (const auto &p : output) {
            currentOrder.push_back(make_pair(p.first, p.second.x));
        }
        sort(currentOrder.begin(), currentOrder.end(), [](const pair<int, float>& a, const pair<int, float>& b) {
            return a.second < b.second;
        });
        // 记录当前已使用的编号
        set<int> usedIds;
        for (const auto &p : currentOrder) {
            usedIds.insert(p.first);
        }

        // 辅助函数：从 0～3 中选择一个不在 usedIds 中的编号
        auto selectAvailableId = [&usedIds]() -> int {
            for (int i = 0; i < 4; i++) {
                if (usedIds.find(i) == usedIds.end())
                    return i;
            }
            return -1; // 理论上最多不超过 4 个
        };

        // 对于未匹配到的检测点，新建轨迹时，按水平方向位置确定编号
        for (size_t i = 0; i < currentPoints.size(); i++) {
            if (!used[i]) {
                Point pt = currentPoints[i].first;
                int newId = -1;
                if (currentOrder.empty()) {
                    newId = 0;
                }
                else {
                    float xMin = currentOrder.front().second;
                    float xMax = currentOrder.back().second;
                    int leftmostId = currentOrder.front().first;
                    int rightmostId = currentOrder.back().first;
                    float candidateError = 0.0f;
                    if (pt.x < xMin) {
                        newId = (leftmostId + 3) % 4;  // 左侧新点
                        candidateError = xMin - pt.x;
                    } else if (pt.x > xMax) {
                        newId = (rightmostId + 1) % 4; // 右侧新点
                        candidateError = pt.x - xMax;
                    } else {
                        newId = selectAvailableId();
                        if(newId == -1) newId = 0;
                    }
                    // 更新不同标号误差值：若候选误差大于零则更新
                    if (candidateError > 0)
                        diffLabelError = alpha * candidateError + (1 - alpha) * diffLabelError;
                }
                usedIds.insert(newId);
                {
                    Point2f initVel(0, 0);
                    float maxSpeed = 0.0f;
                    // 遍历tracker寻找速度最大的tracker
                    // 使用最大加速初始收敛速度/使用最小可能导致不收敛
                    for (auto &trk : tracks) {
                        //if (trk.missedFrames == 0) { //不使用missed判断可以增强在高速旋转时的鲁棒性
                        float vx = trk.kf.statePost.at<float>(2);
                        float vy = trk.kf.statePost.at<float>(3);
                        float speed = sqrt(vx * vx + vy * vy);
                        if (speed > maxSpeed) {
                            maxSpeed = speed;
                            initVel = Point2f(vx, vy);
                        }
                        //}
                    }
                    if(averageLiveFrames < minLivedFrames)
                    {
                        initVel = Point2f(0, 0);
                        //initVel/=2;
                    }
                    Track newTrack(newId, pt, initVel);
                    tracks.push_back(newTrack);
                }
                output[newId] = pt; // 返回原始测量值
                return_result[i] = newId;
            }
        }
        
        map<int, const T*> output2;
        // INFO("Return Result Size: {}",return_result.size());
        // for(int i=0;i<4;i++)
        // {
        //     if(output.find(i) == output.end())
        //     {
        //         INFO("Output2[{}] is not found,fail.",i);
        //     }
        //     else
        //     {
        //         INFO("Output2[{}] is found",i);
        //     }
        // }
        for (size_t i = 0; i < currentPoints.size(); i++) {
            output2[return_result[i]] = currentPoints[i].second;
        }
        return output2;
        // if(autoFillMode == NO_FILL)
        // {    
        //     map<int, Point> output2;
        //     for (size_t i = 0; i < currentPoints.size(); i++) {
        //         output2[return_result[i]] = currentPoints[i];
        //     }
        //     std::cout<<output2.size()<<std::endl;
        //     return output2;
        // }
        // return output;
    }

    double getAverageLiveFrames()
    {
        return averageLiveFrames;
    }
private:
    bool newTrack = false;
    Time::TimeStamp last_time;

    vector<Track> tracks;
    const int maxMissedFrames = 8;
    const int minLivedFrames = 1;//lower than it means the track is not stable.
    // 自适应距离参数及平滑系数
    double sameLabelError;
    double diffLabelError;
    double averageLiveFrames = 10;
    const float alpha; // 平滑更新因子

    enum autoFillModeEnum {
        NO_FILL,
        LAST_DETECT,//高速下容易失配。// deprecated now
        FILTER_PREDICT//高速下甩飞情形特别严重。（可优化）// deprecated now
    };
    const autoFillModeEnum autoFillMode = NO_FILL;
};

}