#include "Detector/include/ArmorOneStage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

namespace detector
{
    namespace
    {
        constexpr int INPUT_W = 640;
        constexpr int INPUT_H = 640;
        constexpr int NUM_COLORS = 4;
        constexpr int NUM_CLASSES = 9;
        constexpr int OUTPUT_DIM = 8 + 1 + NUM_COLORS + NUM_CLASSES;

        cv::Rect boundingRectFromCorners(const cv::Point2f corners[4])
        {
            std::vector<cv::Point2f> points(corners, corners + 4);
            return cv::boundingRect(points);
        }
    }

    ArmorOneStage::ArmorOneStage(const std::string& model_file)
    {
        initModel(model_file);
    }

    ArmorOneStage::~ArmorOneStage() = default;

    void ArmorOneStage::initModel(const std::string& model_file)
    {
        const std::string device = lyutils::DetectorParam::armor_infer_device.empty()
            ? "CPU"
            : lyutils::DetectorParam::armor_infer_device;

        LOG(INFO) << "Loading 0526 armor model: " << model_file << " on " << device;

        model = core.read_model(model_file);

        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input().preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255.0, 255.0, 255.0});
        ppp.input().model().set_layout("NCHW");
        ppp.output().tensor().set_element_type(ov::element::f32);
        model = ppp.build();

        compiled_model = core.compile_model(
            model,
            device,
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
        infer_request = compiled_model.create_infer_request();
    }

    double ArmorOneStage::sigmoid(double x) const
    {
        if (x > 0.0) {
            return 1.0 / (1.0 + std::exp(-x));
        }
        const double exp_x = std::exp(x);
        return exp_x / (1.0 + exp_x);
    }

    bool ArmorOneStage::shouldKeepColor(int detected_color) const
    {
        if (color_flag == -2) {
            return true;
        }
        if (detected_color == 2 || detected_color == 3) {
            return false;
        }
        if (color_flag == -3) {
            return detected_color == 0 || detected_color == 1;
        }
        return detected_color == color_flag;
    }

    int ArmorOneStage::mapClassToHeroId(int detected_class) const
    {
        // 0526/drone: 0=Base, 1=Hero, 2=Engineer, 3=Inf3, 4=Inf4,
        // 5=Inf5, 6=Outpost, 7=Sentry, 8=Unknown.
        // Hero project expects Outpost=7 and Sentry=6.
        switch (detected_class) {
            case 0: return 6;
            case 1: return 1;
            case 2: return 2;
            case 3: return 3;
            case 4: return 4;
            case 5: return 5;
            case 6: return 7;
            case 7: return 0;
            default: return -1;
        }
    }

    BBoxes ArmorOneStage::detect(const cv::Mat& img)
    {
        BBoxes results;
        if (img.empty()) {
            return results;
        }

        cv::Mat input_img;
        cv::resize(img, input_img, cv::Size(INPUT_W, INPUT_H));

        ov::Shape input_shape = {
            1,
            static_cast<size_t>(INPUT_H),
            static_cast<size_t>(INPUT_W),
            3
        };
        ov::Tensor input_tensor(ov::element::u8, input_shape, input_img.data);
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        ov::Tensor output_tensor = infer_request.get_output_tensor(0);
        const ov::Shape output_shape = output_tensor.get_shape();
        if (output_shape.size() != 3 || static_cast<int>(output_shape[2]) != OUTPUT_DIM) {
            throw std::runtime_error("Unexpected 0526 output shape");
        }

        const int num_anchors = static_cast<int>(output_shape[1]);
        cv::Mat output_buffer(num_anchors, OUTPUT_DIM, CV_32F, output_tensor.data<float>());

        const float scale_x = static_cast<float>(img.cols) / INPUT_W;
        const float scale_y = static_cast<float>(img.rows) / INPUT_H;
        constexpr float offset_x = 0.0f;
        constexpr float offset_y = 0.0f;
        const float conf_threshold = lyutils::DetectorParam::score_threshold;
        const float nms_threshold = lyutils::DetectorParam::nms_threshold;

        BBoxes candidates;
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        for (int i = 0; i < num_anchors; ++i) {
            const float confidence = static_cast<float>(sigmoid(output_buffer.at<float>(i, 8)));
            if (confidence < conf_threshold) {
                continue;
            }

            cv::Point color_id;
            cv::minMaxLoc(output_buffer.row(i).colRange(9, 13), nullptr, nullptr, nullptr, &color_id);
            const int detected_color = color_id.x;
            if (!shouldKeepColor(detected_color)) {
                continue;
            }

            cv::Point class_id;
            cv::minMaxLoc(output_buffer.row(i).colRange(13, 22), nullptr, nullptr, nullptr, &class_id);
            const int hero_class = mapClassToHeroId(class_id.x);
            if (hero_class < 0) {
                continue;
            }

            const cv::Point2f left_top(
                output_buffer.at<float>(i, 0) * scale_x + offset_x,
                output_buffer.at<float>(i, 1) * scale_y + offset_y);
            const cv::Point2f left_bottom(
                output_buffer.at<float>(i, 2) * scale_x + offset_x,
                output_buffer.at<float>(i, 3) * scale_y + offset_y);
            const cv::Point2f right_bottom(
                output_buffer.at<float>(i, 4) * scale_x + offset_x,
                output_buffer.at<float>(i, 5) * scale_y + offset_y);
            const cv::Point2f right_top(
                output_buffer.at<float>(i, 6) * scale_x + offset_x,
                output_buffer.at<float>(i, 7) * scale_y + offset_y);

            BBox bbox;
            bbox.corners[0] = left_top;
            bbox.corners[1] = right_top;
            bbox.corners[2] = right_bottom;
            bbox.corners[3] = left_bottom;
            bbox.center = (bbox.corners[0] + bbox.corners[1] + bbox.corners[2] + bbox.corners[3]) / 4.0f;
            bbox.rect = boundingRectFromCorners(bbox.corners);
            bbox.area = static_cast<float>(bbox.rect.area());

            const float width = static_cast<float>(bbox.rect.width);
            const float height = static_cast<float>(bbox.rect.height);
            const float ratio = std::max(width, height) / std::max(1.0f, std::min(width, height));
            if (ratio < 1.0f || ratio > 6.0f) {
                continue;
            }

            bbox.confidence = confidence;
            bbox.color_id = detected_color;
            bbox.tag_id = hero_class;

            candidates.push_back(bbox);
            boxes.push_back(bbox.rect);
            confidences.push_back(confidence);
        }

        if (candidates.empty()) {
            return results;
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
        results.reserve(indices.size());
        for (int idx : indices) {
            if (idx >= 0 && idx < static_cast<int>(candidates.size())) {
                results.push_back(candidates[idx]);
            }
        }

        return results;
    }
}
