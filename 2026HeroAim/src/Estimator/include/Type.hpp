#ifndef ANGLE_T_HPP
#define ANGLE_T_HPP

#include <algorithm>
#include <memory>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "Driver/include/SerialPort.hpp"
#include "utils/include/Params.hpp"
#include "utils/include/Config.hpp"

using namespace std;
using namespace driver;
using namespace lyutils;

namespace estimator{
    static constexpr int MAX_LOST_FRAME = 20;
    static constexpr double ARMOR_DISTANCE_THRESHOLD = 0.3;
    static constexpr float ARMOR_PIXEL_DISTANCE_THRESHOLD = 20.0f;
    static constexpr double ANGLE_FULL_CIRCLE = 180.0;
    static constexpr double BIG_ARMOR_DISTANCE_FACTOR = 1.2;
    static constexpr int IMAGE_CENTER_X = 1024;
    static constexpr int IMAGE_CENTER_Y = 1280;

    typedef struct
    {
        float pitch;    // rad
        float yaw;      // rad
        float time;     // 击打弹道时间(ms)
        float distance; // 距离
    } Angle_t;

    struct Armor
    {
      Armor() = default;
      cv::Point2f center;
      std::vector<cv::Point2f> corners;
      std::vector<cv::Point2f> reprojected_corners;  // 重投影的角点
      bool is_armor = false;

      cv::Rect rect;
      // 数字分类
      int _class;
      bool is_big_armor;
      // pnp解算 - FLU世界坐标系：X前，Y左，Z上
      double x, y, z;
      double angle;
      // 存储每个装甲板的PnP解算结果，用于重投影
      cv::Mat rvec;
      cv::Mat tvec;
      bool operator < (const Armor& a) const
      {
          return x * x + y * y + z * z < a.x * a.x + a.y * a.y + a.z * a.z;
      }
    };
    typedef std::vector<Armor> Armors;

    inline float calcDiff(const Armor &a, const Armor &b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

    // 瞄准跟踪的装甲板
    inline int chooseArmor(const vector<Armor> &armors, const Armor &armor, const Armor &last_armor) {
        int priority_class = 0;
        double min_distance = numeric_limits<double>::max();

        for (const auto &a : armors) {
            if (a._class == 7) // 优先选择前哨战
                return 7;

            if (a._class == last_armor._class)
                priority_class = last_armor._class;

            double distance = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);

            switch (a._class) {
                case 1: // 英雄
                    if (priority_class == 0)
                        priority_class = 1;
                    break;
                case 3: case 4: case 5: // 步兵
                    if (distance < 10.0 && distance < min_distance) {
                        min_distance = distance;
                        priority_class = a._class;
                    }
                    break;
                case 6: // 哨兵
                    if (priority_class == 0)
                        priority_class = 6;
                    break;
                case 2: // 工程
                    if (priority_class == 0)
                        priority_class = 2;
                    break;
            }
        }

        return priority_class;
    }


}
#endif
