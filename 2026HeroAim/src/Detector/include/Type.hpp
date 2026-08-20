#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <vector>

namespace detector
{
    // === 新数据类型定义 ===
    struct Detection
    {
        cv::Rect2f bounding_rect;
        cv::Point2f center;
        int tag_id;   // 数字ID
        int color_id; // 0:Blue, 1:Red
        float score;
        std::vector<cv::Point2f> corners;
        // pnp解算
        double x, y, z;
        double angle;  //yaw
        bool operator < (const Detection& a) const  
        {
            return x * x + y * y + z * z < a.x * a.x + a.y * a.y + a.z * a.z;
        }
    };
    typedef std::vector<Detection> Detections;

    struct CarDetection
    {
        cv::Rect2f bounding_rect;
        cv::Point2f center;
        int tag_id;
        float score;
    };
    typedef std::vector<CarDetection> CarDetections;

    class BBox
    {
    public:
        BBox() : center(0, 0), rect(0, 0, 0, 0), area(0.0f), confidence(0.0f), color_id(0), tag_id(0)
        {
            for(int i = 0; i < 4; i++)
            {
                corners[i] = cv::Point2f(0, 0); // 初始化 corners 数组
            }
            points.clear(); // 清空 points 向量
        }

        cv::Point2f corners[4];
        std::vector<cv::Point2f> points;
        cv::Point2f center;
        cv::Rect rect;
        float area;
        float confidence;
        int color_id;
        int tag_id;
    };
    typedef std::vector<BBox> BBoxes;

    struct YoloDetection {
        cv::Point2f center;                 // 中心点
        cv::Rect rect;                      // 外接矩形
        int class_id;                       // 类别ID
        float confidence;                   // 置信度
        float area;                         // 区域面积
    };
    typedef std::vector<YoloDetection> YoloDetections;

    // === 统一输出结果 ===
    struct DetectResult {
        std::vector<Detection> armors;
        std::vector<CarDetection> cars;
        cv::Mat debug_img;
    };
}
