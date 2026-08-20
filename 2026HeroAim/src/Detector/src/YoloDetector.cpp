#include "Detector/include/YoloDetector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include "utils/include/TimeStamp.hpp"
#include "utils/include/Log.hpp"

// 辅助函数：对图像进行 letterbox 处理，保持宽高比，并填充黑色背景
static cv::Mat letterbox(const cv::Mat &source) {
    int cols = source.cols;
    int rows = source.rows;
    int max_dim = std::max(cols, rows);
    cv::Mat result = cv::Mat::zeros(max_dim, max_dim, source.type());
    source.copyTo(result(cv::Rect(0, 0, cols, rows)));
    return result;
}
//返回MAT对象，把原图像考入
/* //why usw automic:
static cv::Mat lll(const cv::Mat&src){
    int col=src.col;
    int row=src.row;
    int max_dim=std::max(col,row);
    cv::Mat result=cv::Mat::zeros(max_dim,max_dim,src.type());
    std::atomic::result=src;
    result.copyTo(result(cv::Rect(0,0,col,row)));
    return result;
}*/
YoloDetector::YoloDetector(const std::string& model_path, Precision precision, bool use_gpu)
{
    // 设置 OpenVINO 运行时的设备
    std::string device = use_gpu ? "GPU.0" : "CPU";
    // 设置模型精度
    ov::Core core;
    std::string precision_str;
    switch (precision) {
        case Precision::FP32:
            precision_str = "f32";
            break;
        case Precision::FP16:
            precision_str = "f16";
            break;
        case Precision::INT8:
            precision_str = "f16";  // INT8不支持，降级为f16
            break;
        default:
            throw std::invalid_argument("Unsupported precision type");
    }
    // 读取模型并编译，用ov的core模块：
    model = core.read_model(model_path);
    compiled_model = core.compile_model(
        model,
        device,
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
        ov::inference_num_threads(4),
        ov::hint::inference_precision(precision_str)//,
        //ov::hint::enable_cpu_pinning(true),
        //ov::hint::enable_hyper_threading(true)
    );
    infer_request = compiled_model.create_infer_request();
    // // 读取模型并编译
    // model = core.read_model(model_path);
    // compiled_model = core.compile_model(model, device);
    // infer_request = compiled_model.create_infer_request();
}

YoloDetections YoloDetector::detect(const cv::Mat& image)
{
    Time::TimeStamp start_time = Time::TimeStamp::now();
    // 1. 对输入图像进行 letterbox 处理
    cv::Mat letterbox_img = letterbox(image);
    // 使用内部定义的输入尺寸进行 blob 处理
    float scale = static_cast<float>(letterbox_img.cols) / INPUT_W;
    cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0,
                                          cv::Size(INPUT_W, INPUT_H), cv::Scalar(), true, false);
    //将图像转换为模型输入所需的blob格式

    // 2. 设置输入 tensor（假设模型只有一个输入）
    auto input_port = compiled_model.input();
    //转张量：
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
    infer_request.set_input_tensor(input_tensor);

    // 3. 执行推理
    infer_request.infer();

    // 4. 获取输出 tensor 并解析结果
    auto output_tensor = infer_request.get_output_tensor(0);
    auto output_shape = output_tensor.get_shape();
    float* data = output_tensor.data<float>();

    // 假设输出 shape 为 {1, dimensions, rows}，例如 dimensions 为 84（4 表示 bboxes，后续为各类分数）
    int dimensions = static_cast<int>(output_shape[1]);
    int rows = static_cast<int>(output_shape[2]);

    // 将输出数据转换为 cv::Mat，便于后续处理
    cv::Mat output_buffer(dimensions, rows, CV_32F, data);
    cv::Mat transposed;
    cv::transpose(output_buffer, transposed); // transposed 大小为 (rows, dimensions)

    // 5. 遍历输出，筛选候选框
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int i = 0; i < transposed.rows; i++) {
        cv::Mat row = transposed.row(i);
        // 前4个元素为 bbox，后面为各类别得分
        cv::Mat scores = row.colRange(4, dimensions);
        cv::Point class_id_point;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);

        if (max_class_score > CONF_THRESHOLD) {
            float cx = row.at<float>(0, 0);
            float cy = row.at<float>(0, 1);
            float w  = row.at<float>(0, 2);
            float h  = row.at<float>(0, 3);

            int left   = static_cast<int>((cx - 0.5f * w) * scale);
            int top    = static_cast<int>((cy - 0.5f * h) * scale);
            int width  = static_cast<int>(w * scale);
            int height = static_cast<int>(h * scale);

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(static_cast<float>(max_class_score));
            class_ids.push_back(class_id_point.x);
        }
    }
    //遍历四个数据，获取置信度高于阈值的得分。

    // 6. 非极大值抑制（NMS）
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, indices);

    // 7. 整理最终检测结果
    YoloDetections detections;
    for (int idx : indices) {
        YoloDetection det;
        det.rect = boxes[idx];
        det.confidence = confidences[idx];
        det.class_id = class_ids[idx];
        det.center = cv::Point2f(det.rect.x + det.rect.width * 0.5f,
                                 det.rect.y + det.rect.height * 0.5f);
        det.area = static_cast<float>(det.rect.area());
        detections.push_back(det);
    }
    //LOG(INFO) << "YoloDetector inference time: " << (Time::TimeStamp::now() - start_time).toSeconds() * 1000 << " ms";


    // 可选：记录调试图像
    //debug_image = letterbox_img;

    return detections;
}

void YoloDetector::setConfThreshold(float threshold)
{
    CONF_THRESHOLD = threshold;
}

void YoloDetector::setNMSThreshold(float threshold)
{
    NMS_THRESHOLD = threshold;
}