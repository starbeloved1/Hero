#pragma once
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

struct YoloDetection {
    cv::Rect rect;
    float confidence;
    int class_id;
    cv::Point2f center;
    float area;
};
typedef std::vector<YoloDetection> YoloDetections;

class YoloDetector {
public:
    enum class Precision {
        FP32,
        FP16,
        INT8
    };

    // 构造函数，其中 NUM_CLASSES 可以根据需要修改，目前默认为 1
    YoloDetector(const std::string& model_path, 
                 Precision precision = Precision::INT8,
                 bool use_gpu = false);
    ~YoloDetector() = default;
    
    // 执行推理并返回检测结果
    YoloDetections detect(const cv::Mat& image);
       
    // 设置置信度和 NMS 阈值
    //NMS去除重复的框。类比mIoU阈值。
    void setConfThreshold(float threshold);
    void setNMSThreshold(float threshold);

private:
    // 配置参数，可根据需要调整
    const int INPUT_W = 416;
    const int INPUT_H = 416;
    const int NUM_CLASSES = 1;       // 可配置类别数，目前为1
    const int MAX_OBJECTS = 1000;
    float CONF_THRESHOLD = 0.5f;
    float NMS_THRESHOLD = 0.3f;
    
    ov::Core core;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    
    // 调试图像
    cv::Mat debug_image;
};