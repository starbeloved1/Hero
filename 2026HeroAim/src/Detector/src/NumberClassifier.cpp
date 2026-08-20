#include "Detector/include/NumberClassifier.hpp"
#define svm 0
#define net 1
namespace detector
{
    NumberClassifier::NumberClassifier(std::string mode)
    {
        if (mode == "svm")
        {
            loadModel("../src/utils/model/svm_numbers_rbf.xml");
            initHog();
            calcGammaTable(2);
            mode_ = svm;
        }
        else if (mode == "net")
        {
            //TODO 
            mode_ = net;
        }
        else //default: svm
        {
            loadModel("../src/utils/model/svm_numbers_rbf.xml");
            initHog();
            calcGammaTable(2);
            mode_ = svm;
        }
    }

    NumberClassifier::~NumberClassifier()
    {
        delete hog_;
    }

    void NumberClassifier::initHog()
    {
        // 窗口大小,块大小，块步长，cell，180度分为几个区间
        hog_ = new cv::HOGDescriptor(cv::Size(32, 32), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 16);
    }

    void NumberClassifier::calcGammaTable(float gamma)
    {
        for (int i = 0; i < 256; ++i)
        {
            gamma_table[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), gamma) * 255.0f);
        }
        lut_table = cv::Mat(1, 256, CV_8UC1, gamma_table);
    }

    // 加载模型
    bool NumberClassifier::loadModel(const std::string &model_path)
    {
        std::cout << "Loading SVM model from: " << model_path << std::endl;
        try {
            number_svm_model = cv::ml::SVM::load(model_path);
            if (number_svm_model.empty())
            {
                std::cerr << "Open NUMBER svm model ERROR: model is empty" << std::endl;
                is_model_set = false; // 模型不可用
            }
            else
            {
                std::cout << "SVM model loaded successfully!" << std::endl;
                is_model_set = true;
            }
        } catch (const cv::Exception& e) {
            std::cerr << "Failed to load SVM model: " << e.what() << std::endl;
            is_model_set = false;
        }
        return is_model_set;
    }
    //透视变换提取数字区域：
    bool NumberClassifier::affineNumber(const cv::Mat &frame, const std::vector<cv::Point2f> &corners)
    {
        if (mode_ == svm)
        {
            is_get_roi = false;
            static float classify_width_ratio = 0.2f;
            static float classify_height_ratio = 0.5f;

            // 求解完全包围这个框的最大矩形
            cv::Point2f correct_points[4];
            cv::Point2f width_vec = (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
            cv::Point2f height_vec = (corners[3] - corners[0] + corners[2] - corners[1]) / 2;
            correct_points[0] = corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
            correct_points[1] = corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
            correct_points[2] = corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
            correct_points[3] = corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

            int width = getDistance(correct_points[0], correct_points[1]);
            int height = getDistance(correct_points[1], correct_points[2]);
            
            // 检查宽高有效性
            if (width <= 0 || height <= 0) {
                std::cerr << "Invalid width/height: " << width << "x" << height << std::endl;
                std::cerr << "corners: [0]=(" << corners[0].x << "," << corners[0].y << ") "
                          << "[1]=(" << corners[1].x << "," << corners[1].y << ") "
                          << "[2]=(" << corners[2].x << "," << corners[2].y << ") "
                          << "[3]=(" << corners[3].x << "," << corners[3].y << ")" << std::endl;
                std::cerr << "correct_points: [0]=(" << correct_points[0].x << "," << correct_points[0].y << ") "
                          << "[1]=(" << correct_points[1].x << "," << correct_points[1].y << ")" << std::endl;
                return false;
            }
            
            cv::Point2f min_point = cv::Point2f(9999.0f, 9999.0f);
            cv::Point2f max_point = cv::Point2f(0.0f, 0.0f);
            for (int i = 0; i < 4; i++)
            {
                min_point.x = min_point.x < correct_points[i].x ? min_point.x : correct_points[i].x;
                min_point.y = min_point.y < correct_points[i].y ? min_point.y : correct_points[i].y;
                max_point.x = max_point.x > correct_points[i].x ? max_point.x : correct_points[i].x;
                max_point.y = max_point.y > correct_points[i].y ? max_point.y : correct_points[i].y;
            }
            min_point.x = MAX(min_point.x, 0);
            min_point.y = MAX(min_point.y, 0);
            max_point.x = MIN(max_point.x, frame.cols);
            max_point.y = MIN(max_point.y, frame.rows);

            // 检查裁剪区域有效性
            if (max_point.x <= min_point.x || max_point.y <= min_point.y) {
                std::cerr << "Invalid ROI: min(" << min_point.x << "," << min_point.y 
                          << ") max(" << max_point.x << "," << max_point.y << ")" << std::endl;
                return false;
            }

            // 截取
            cv::Mat m_number_roi = frame(cv::Rect(min_point, max_point));

            for (int i = 0; i < 4; i++)
            {
                correct_points[i] -= min_point;
            }

            // 制作重映射对应点
            cv::Point2f remap_points[4];
            remap_points[0] = cv::Point2f(0, 0);
            remap_points[1] = cv::Point2f((int)width, 0);
            remap_points[2] = cv::Point2f((int)width, (int)height);
            remap_points[3] = cv::Point2f(0, (int)height);

            // 进行重映射
            cv::Mat trans_matrix = cv::getPerspectiveTransform(correct_points, remap_points);
            cv::Mat output_roi;
            output_roi.create(cv::Size((int)width, (int)height), CV_8UC3);

            if (m_number_roi.empty() || output_roi.empty())
            {
                std::cerr << "m_number_roi or output_roi empty! m_number_roi: " 
                          << m_number_roi.cols << "x" << m_number_roi.rows
                          << ", output_roi: " << output_roi.cols << "x" << output_roi.rows << std::endl;
                return false;
            }
            cv::warpPerspective(m_number_roi, output_roi, trans_matrix, output_roi.size());

            // //从重映射中取得目标图像
            cv::resize(output_roi, number_roi, cv::Size(32, 32)); // 根据训练的数据大小来判断大小
            is_get_roi = true;

            return is_get_roi;
        }
        else if (mode_ == net)
        {
            //TODO
            //神经网络自带。
            return true;
        }
        return false;
    }

    float NumberClassifier::getDistance(const cv::Point2f &point_1, const cv::Point2f &point_2)
    {
        float x = (point_1 - point_2).x;
        float y = (point_1 - point_2).y;
        return sqrt(x * x + y * y);
    }

    void NumberClassifier::AutoGamma(const cv::Mat &img, cv::Mat &out)
    {
        const int channels = img.channels();
        const int type = img.type();
        /*debug*/
        assert(type == CV_8UC1 || type == CV_8UC3);

        auto mean = cv::mean(img); // 均值
        mean[0] = std::log10(0.4) / std::log10(mean[0] / 255);

        if (channels == 3) // 3channels
        {
            mean[1] = std::log10(0.4) / std::log10(mean[1] / 255);
            mean[2] = std::log10(0.4) / std::log10(mean[2] / 255);

            float mean_end = 0;
            for (int i = 0; i < 3; ++i)
                mean_end += mean[i];

            mean_end /= 3.0;
            for (int i = 0; i < 3; ++i)
                mean[i] = mean_end;
        }
        /*gamma_table*/
        cv::Mat lut(1, 256, img.type());

        if (channels == 1)
        {
            for (int i = 0; i < 256; i++)
            { /*[0,1]*/
                float Y = i * 1.0f / 255.0;
                Y = std::pow(Y, mean[0]);
                lut.at<unsigned char>(0, i) = cv::saturate_cast<unsigned char>(Y * 255);
            }
        }
        else
        {
            for (int i = 0; i < 256; ++i)
            {
                float Y = i * 1.0f / 255.0;
                auto B = cv::saturate_cast<unsigned char>(std::pow(Y, mean[0]) * 255);
                auto G = cv::saturate_cast<unsigned char>(std::pow(Y, mean[1]) * 255);
                auto R = cv::saturate_cast<unsigned char>(std::pow(Y, mean[2]) * 255);

                lut.at<cv::Vec3b>(0, i) = cv::Vec3b(B, G, R);
            }
        }
        cv::LUT(img, lut, out);
    }

    cv::Mat NumberClassifier::AutoGammaCorrect(const cv::Mat image)
    {
        cv::Mat img;
        cv::Mat dst;
        image.copyTo(dst);
        image.copyTo(img);

        cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
        cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
        const int type = img.type();
        // assert(type == CV_8UC1 || type == CV_8UC3);

        std::vector<cv::Mat> channels(3);
        cv::split(img, channels);
        cv::Mat V = channels[2];
        V.convertTo(V, CV_32F);
        double max_v;
        cv::minMaxIdx(V, 0, &max_v);
        V /= 255.0;

        cv::Mat Mean, Sigma;
        cv::meanStdDev(V, Mean, Sigma);

        double mu, sigma; /*均值and方差*/
        bool High_contrast = false;
        bool High_bright = false;
        mu = Mean.at<double>(0);
        sigma = Sigma.at<double>(0);
        // std::cout<<"mu = "<<mu<<"sigma = "<<sigma<<std::endl;

        if (4 * sigma > 0.3333)
        {
            High_contrast = true;
            //    std::cout<<"High_con"<<std::endl;
        }
        if (mu > 0.65)
        {
            High_bright = true;
            //    std::cout<<"High_bri"<<std::endl;
        }
        double gamma, c, Heaviside;

        if (High_contrast)
        {
            gamma = std::exp((1 - (mu + sigma)) / 2.0);
        }
        else
        {
            gamma = -std::log(sigma) / std::log(2);
        }
        // std::cout << gamma << std::endl;

        return pixel(dst, gamma, mu, sigma, High_bright);
    }

    cv::Mat NumberClassifier::pixel(const cv::Mat image, double gamma, double mu, double sigma, int flag)
    {
        double K;
        cv::Mat img;
        image.copyTo(img);
        int rows = image.rows;
        int cols = image.cols;
        int channels = image.channels();
        if (flag)
        {
            for (int i = 0; i < cols; ++i)
            {
                for (int j = 0; j < rows; ++j)
                {
                    if (channels == 3)
                        for (int k = 0; k < channels; ++k)
                        {
                            float pix = float(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
                            img.at<cv::Vec3b>(i, j)[k] = cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
                        }
                    else
                    {
                        float pix = float(img.at<uchar>(i, j)) / 255.0;
                        img.at<uchar>(i, j) = cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < cols; ++i)
            {
                for (int j = 0; j < rows; ++j)
                {
                    if (channels == 3)
                        for (int k = 0; k < channels; ++k)
                        {
                            float pix = float(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
                            double t = std::pow(mu, gamma);
                            K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
                            img.at<cv::Vec3b>(i, j)[k] = cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
                        }
                    else
                    {
                        float pix = float(img.at<uchar>(i, j)) / 255.0;
                        double t = std::pow(mu, gamma);
                        K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
                        img.at<uchar>(i, j) = cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
                    }
                }
            }
        }
        return img;
    }


    /*std::pair<int, double> NumberClassifier::predict(const cv::Mat &frame, const std::vector<cv::Point2f> &corners)
{
    // 提取装甲板数字区域
    // 应用自适应Gamma校正改善图像质量
    // 提取HOG特征描述子
    // 使用SVM模型进行数字分类预测
    // 返回预测的数字类别和置信度
}*/
    std::pair<int, double> NumberClassifier::predict(const cv::Mat &frame, const std::vector<cv::Point2f> &corners)
    {
        if (!is_model_set) {
            std::cerr << "SVM model not loaded!" << std::endl;
            return std::pair<int, double>(-1, 0);
        }
        
        if (!affineNumber(frame, corners) || number_roi.empty())
        {
            std::cerr << "affineNumber failed or number_roi empty. Frame size: " 
                      << frame.cols << "x" << frame.rows 
                      << ", corners[0]: (" << corners[0].x << "," << corners[0].y << ")" << std::endl;
            return std::pair<int, double>(-1, 0);
        }

        if (mode_ == svm)
        {
            // 自适应gamma PLUS
            number_roi = AutoGammaCorrect(number_roi);
            // 普通gamma
            // cv::LUT(number_roi, lut_table, number_roi);

            std::vector<float> hog_descriptors;
            // 对图片提取hog描述子存在hog_descriptors中，hog描述子是向量，不是矩阵
            hog_->compute(number_roi, hog_descriptors, cv::Size(8, 8), cv::Size(0, 0));
            size_t size = hog_descriptors.size();
            cv::Mat descriptors_mat(1, size, CV_32FC1); // 行向量
            for (size_t i = 0; i < hog_descriptors.size(); ++i)
            {
                descriptors_mat.at<float>(0, i) = hog_descriptors[i] * 100;
            }

            // 把提取的本张图片的hog描述子放进svm预测器中进行预测
            number_svm_model->predict(descriptors_mat, class_);
            return std::pair<int, double>((int)class_.at<float>(0), 1);
        }
        else if (mode_ == net)
        {
            //TODO
            return {};
        }
        return {};
    }
}