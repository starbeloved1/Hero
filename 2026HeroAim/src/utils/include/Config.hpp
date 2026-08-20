#ifndef AUTOAIM_CONFIG_HPP
#define AUTOAIM_CONFIG_HPP

#include <string>
#include <fstream>
#include "glog/logging.h"
#include "jsoncpp/json/json.h"

#include "Params.hpp"
#include "../../Driver/include/VideoCapture.h"

using namespace std;
using namespace driver;

namespace lyutils{
    /**
     * @brief: 解析配置文件
     */
    class Config { 
    public:
        Config()  = default;
        explicit Config(const string& path);
        void parse();
        void chooseCameraType(driver::VideoCapture*& video);
    private:
        string json_file_path;
    };
}

#endif //AUTOAIM_CONFIG_H
