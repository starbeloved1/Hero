#ifndef AUTOAIM_AUTOAIM_H
#define AUTOAIM_AUTOAIM_H

#include "Controller/Controller.hpp"
#include "Detector/include/Detector.hpp"
#include "Detector/include/VideoSaver.hpp"
#include "Driver/include/SerialPort.hpp"
#include "Driver/include/VideoCapture.h"
#include "Estimator/include/AntiTop.hpp"
#include "Estimator/include/Estimator.hpp"
#include "Estimator/include/NormalAim.hpp"
#include "Predictor/include/predictor.hpp"
#include "Solver/Solver.hpp"
#include "TCPServer.hpp"
#include "Thread.hpp"
#include "Tracker/Tracker.hpp"
#include "utils/include/Config.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"

#include <string>

using namespace std;
using namespace driver;
using namespace lyutils;

const string confog_file_path = "../src/utils/tools/init.json";

#endif // AUTOAIM_AUTOAIM_HPP
