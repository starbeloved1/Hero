#pragma once

#include <iostream>
#include <openvino/openvino.hpp>

#include "Type.hpp"

namespace detector
{
    class ArmorOneStage
    {
    public:
        explicit ArmorOneStage(const std::string& model_file);
        ~ArmorOneStage();

        BBoxes detect(const cv::Mat& img);

        void setColorFlag(int flag_)
        {
            if (flag_ == 0) {
                color_flag = 1;
            } else if (flag_ == 1) {
                color_flag = 0;
            } else if (flag_ == -2) {
                color_flag = -2;
            } else if (flag_ == -3) {
                color_flag = -3;
            } else {
                color_flag = 0;
            }
        }

    private:
        ov::Core core;
        std::shared_ptr<ov::Model> model;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;

        void initModel(const std::string& model_file);
        double sigmoid(double x) const;
        bool shouldKeepColor(int detected_color) const;
        int mapClassToHeroId(int detected_class) const;

        int color_flag = -1;
    };
}
