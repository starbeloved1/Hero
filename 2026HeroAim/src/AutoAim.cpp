#include "AutoAim.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "AntiBase/include/AntiBaseManager.hpp"
#include "AntiBase/include/AntiBaseTransmitter.hpp"
#include "utils/include/Location.hpp"
#include "utils/include/Thread.hpp"
#include "utils/include/Visualizer.hpp"

using namespace lyutils;
using namespace driver;
using namespace tracker;
using namespace solver;
using namespace estimator;
using namespace antibase;

std::atomic<bool> g_running{true};
std::atomic<bool> g_shutdown_done{false};
std::atomic<std::chrono::steady_clock::time_point> g_last_heartbeat{std::chrono::steady_clock::now()};
constexpr int WATCHDOG_TIMEOUT_SEC = 5;
constexpr int WATCHDOG_GRACE_SEC = 2;

namespace {

class FrameProfiler {
    using Clock = std::chrono::steady_clock;

public:
    explicit FrameProfiler(int frame_id)
        : frame_id_(frame_id),
          frame_start_(Clock::now())
    {
    }

    class Scope {
    public:
        Scope(FrameProfiler& profiler, std::string name)
            : profiler_(profiler),
              name_(std::move(name)),
              start_(Clock::now())
        {
        }

        ~Scope()
        {
            if (active_) {
                profiler_.add(name_, elapsedMs(start_, Clock::now()));
            }
        }

        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
        Scope(Scope&& other) noexcept
            : profiler_(other.profiler_),
              name_(std::move(other.name_)),
              start_(other.start_),
              active_(other.active_)
        {
            other.active_ = false;
        }
        Scope& operator=(Scope&&) = delete;

    private:
        FrameProfiler& profiler_;
        std::string name_;
        Clock::time_point start_;
        bool active_ = true;
    };

    Scope scope(std::string name)
    {
        return Scope(*this, std::move(name));
    }

    void add(const std::string& name, double ms)
    {
        records_.push_back({name, ms});
    }

    void log(int mode, int image_cols, int image_rows) const
    {
        // Mode4 已由 [M4 ENC] 输出更有诊断价值的处理耗时，避免重复日志。
        if (mode == 4) {
            return;
        }
        const double total_ms = elapsedMs(frame_start_, Clock::now());
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "[FrameCost] frame=" << frame_id_
            << " mode=" << mode
            << " size=" << image_cols << "x" << image_rows
            << " total=" << total_ms << "ms";

        for (const auto& record : records_) {
            oss << " " << record.name << "=" << record.ms << "ms";
        }

        LOG(INFO) << oss.str();
    }

private:
    struct Record {
        std::string name;
        double ms;
    };

    static double elapsedMs(Clock::time_point start, Clock::time_point end)
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    int frame_id_;
    Clock::time_point frame_start_;
    std::vector<Record> records_;
};

struct CameraRuntime {
    std::unique_ptr<driver::VideoCapture> capture;
    Params_ToVideo params;
    std::thread thread;
};

struct ThreadRefs {
    bool& image_is_update;
    std::mutex& mtx_image;
    std::condition_variable& cond_is_update;
    std::condition_variable& cond_is_process;
};

bool isDualCameraEnabled()
{
#ifdef USE_DAHENG_CAMERA
    return CameraParam::device_type == driver::DaHen && CameraParam::use_base_camera;
#else
    return false;
#endif
}

driver::CameraProfile selectProfileForMode(int mode, bool dual_camera_enabled)
{
    if (mode == 4 && dual_camera_enabled) {
        return driver::CameraProfile::Base;
    }
    if (mode == 1 && CameraParam::use_8mm2) {
        return driver::CameraProfile::Aim8mm2;
    }
    return driver::CameraProfile::Aim8mm;
}

ThreadRefs threadRefsForProfile(driver::CameraProfile profile)
{
    if (profile == driver::CameraProfile::Base) {
        return {Thread::image2_is_update, Thread::mtx_image2, Thread::cond2_is_update, Thread::cond2_is_process};
    }
    if (profile == driver::CameraProfile::Aim8mm2) {
        return {Thread::image3_is_update, Thread::mtx_image3, Thread::cond3_is_update, Thread::cond3_is_process};
    }
    return {Thread::image_is_update, Thread::mtx_image, Thread::cond_is_update, Thread::cond_is_process};
}

CameraRuntime& cameraForProfile(driver::CameraProfile profile,
                                CameraRuntime& aim_camera,
                                CameraRuntime& aim_camera_2,
                                CameraRuntime& base_camera,
                                bool dual_camera_enabled)
{
    if (profile == driver::CameraProfile::Base && dual_camera_enabled) {
        return base_camera;
    }
    if (profile == driver::CameraProfile::Aim8mm2 && CameraParam::use_8mm2) {
        return aim_camera_2;
    }
    return aim_camera;
}

bool startCamera(CameraRuntime& runtime, driver::CameraProfile profile)
{
    driver::VideoCapture* raw_capture = nullptr;
    driver::VideoCapture::chooseCameraType(raw_capture, profile);
    runtime.capture.reset(raw_capture);
    if (!runtime.capture) {
        return false;
    }
    runtime.thread = std::thread(&driver::VideoCapture::startCapture,
                                 runtime.capture.get(),
                                 std::ref(runtime.params));
    return true;
}

bool fetchFrame(CameraRuntime& runtime,
                driver::CameraProfile profile,
                TimeImageData& out_frame,
                int wait_ms)
{
    auto refs = threadRefsForProfile(profile);
    std::unique_lock<std::mutex> lock(refs.mtx_image);
    while (!refs.image_is_update && g_running) {
        auto status = refs.cond_is_update.wait_for(lock, std::chrono::milliseconds(wait_ms));
        if (status == std::cv_status::timeout) {
            break;
        }
    }

    if (!g_running || !refs.image_is_update) {
        return false;
    }

    if (runtime.params.frame_pp == nullptr || *runtime.params.frame_pp == nullptr) {
        refs.image_is_update = false;
        refs.cond_is_process.notify_one();
        return false;
    }

    const Image& image = **runtime.params.frame_pp;
    if (image.mat == nullptr || image.mat->empty()) {
        refs.image_is_update = false;
        refs.cond_is_process.notify_one();
        return false;
    }

    out_frame.image = *(image.mat);
    out_frame.steady_timestamp = image.time_stamp;
    out_frame.timestamp = Time::TimeStamp::now();

    refs.image_is_update = false;
    refs.cond_is_process.notify_one();
    return true;
}

bool fetchFrameForProfile(driver::CameraProfile profile,
                          CameraRuntime& aim_camera,
                          CameraRuntime& aim_camera_2,
                          CameraRuntime& base_camera,
                          bool dual_camera_enabled,
                          TimeImageData& out_frame,
                          int wait_ms)
{
    auto& runtime = cameraForProfile(
        profile, aim_camera, aim_camera_2, base_camera, dual_camera_enabled);
    return fetchFrame(runtime, profile, out_frame, wait_ms);
}

void applyBaseExposureAdjustment(driver::SerialPort& serial_port, CameraRuntime& base_camera)
{
    const int steps = serial_port.consumeExposureAdjustmentSteps();
    if (steps == 0) {
        return;
    }

    constexpr double kExposureStepUs = 500.0;
    if (!base_camera.capture) {
        LOG(WARNING) << "[Base Exposure] ignored " << steps
                     << " step(s): base camera is not enabled";
        return;
    }

    double new_exposure_us = 0.0;
    if (!base_camera.capture->adjustExposureUs(kExposureStepUs * steps, &new_exposure_us)) {
        LOG(WARNING) << "[Base Exposure] adjustment unavailable for active base camera";
        return;
    }
    LOG(INFO) << "[Base Exposure] steps=" << steps
              << " delta_us=" << kExposureStepUs * steps
              << " applied_us=" << new_exposure_us;
}

bool fetchMatchedFrameForMode(detector::Detector& detector,
                              driver::SerialPort& serial_port,
                              CameraRuntime& aim_camera,
                              CameraRuntime& aim_camera_2,
                              CameraRuntime& base_camera,
                              bool dual_camera_enabled,
                              SerialPortData& imu_data,
                              int& current_mode,
                              TimeImageData& frame)
{
    driver::CameraProfile profile = selectProfileForMode(current_mode, dual_camera_enabled);
    if (!fetchFrameForProfile(profile, aim_camera, aim_camera_2, base_camera,
                              dual_camera_enabled, frame, 100)) {
        return false;
    }

    if (!serial_port.findNearestImu(frame.steady_timestamp, imu_data)) {
        std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
        imu_data = SerialParam::recv_data;
    }
    current_mode = detector.determineOperationMode(imu_data.flag);

    const driver::CameraProfile matched_profile = selectProfileForMode(current_mode, dual_camera_enabled);
    if (matched_profile == profile) {
        return true;
    }

    profile = matched_profile;
    if (!fetchFrameForProfile(profile, aim_camera, aim_camera_2, base_camera,
                              dual_camera_enabled, frame, 100)) {
        return false;
    }
    if (!serial_port.findNearestImu(frame.steady_timestamp, imu_data)) {
        std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
        imu_data = SerialParam::recv_data;
    }
    current_mode = detector.determineOperationMode(imu_data.flag);
    return true;
}

bool refreshLatestAimFrameIfNeeded(detector::Detector& detector,
                                   driver::SerialPort& serial_port,
                                   CameraRuntime& aim_camera,
                                   CameraRuntime& aim_camera_2,
                                   CameraRuntime& base_camera,
                                   bool dual_camera_enabled,
                                   SerialPortData& imu_data,
                                   int& current_mode,
                                   TimeImageData& frame)
{
    if (!GlobalParam::AUTOAIM_LATEST_FRAME_PREFER || current_mode == 4) {
        return true;
    }

    const driver::CameraProfile profile = selectProfileForMode(current_mode, dual_camera_enabled);
    auto& active_camera = cameraForProfile(
        profile, aim_camera, aim_camera_2, base_camera, dual_camera_enabled);

    int max_refreshes = std::max(0, GlobalParam::AUTOAIM_MAX_FRAME_REFRESHES);
    int refresh_window_ms = std::max(0, GlobalParam::AUTOAIM_LATEST_FRAME_REFRESH_MS);
    auto refresh_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(refresh_window_ms);

    for (int refreshes = 0; refreshes < max_refreshes && g_running; ++refreshes) {
        auto now = std::chrono::steady_clock::now();
        if (now >= refresh_deadline) {
            break;
        }

        int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(refresh_deadline - now).count());
        if (remain_ms <= 0) {
            break;
        }

        TimeImageData newer_frame;
        if (!fetchFrame(active_camera, profile, newer_frame, remain_ms)) {
            break;
        }
        frame = std::move(newer_frame);
    }

    if (!serial_port.findNearestImu(frame.steady_timestamp, imu_data)) {
        std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
        imu_data = SerialParam::recv_data;
    }
    current_mode = detector.determineOperationMode(imu_data.flag);
    if (selectProfileForMode(current_mode, dual_camera_enabled) != profile) {
        // 刷新窗口内模式切换时不能拿旧相机帧进入新模式，尤其不能让 Mode4
        // 意外处理 8mm 图。按更新后的模式重新取匹配相机帧；失败则本轮不处理。
        return fetchMatchedFrameForMode(detector, serial_port,
                                        aim_camera, aim_camera_2, base_camera,
                                        dual_camera_enabled, imu_data, current_mode, frame);
    }
    return true;
}

Armors buildArmors(const detector::Detections& detections)
{
    Armors armors;
    armors.reserve(detections.size());
    for (const auto& detection : detections) {
        Armor armor;
        armor.center = detection.center;
        armor.corners = detection.corners;
        armor.is_armor = true;
        armor.rect = detection.bounding_rect;
        armor._class = detection.tag_id;
        armors.emplace_back(std::move(armor));
    }
    return armors;
}

void pushOutputFrame(detector::Detector& detector, const cv::Mat& fallback)
{
    if (!detector.visualization_frame.empty()) {
        detector.pushStream(detector.visualization_frame);
    } else {
        detector.pushStream(fallback);
    }
}

void handleNormalAim(detector::Detector& detector,
                     estimator::Estimator& estimator,
                     const detector::DetectResult& detect_result,
                     double delta_t,
                     const SerialPortData& imu_data,
                     driver::SerialPort* serial_port,
                     const cv::Mat& frame_image,
                     int current_mode,
                     FrameProfiler& profiler)
{
    LOG(WARNING) << "  >>>>>>>>>>>>>>>            IN NormalAim            <<<<<<<<<<<<<<<<" << std::endl;

    if (!detect_result.armors.empty()) {
        Armors armors = buildArmors(detect_result.armors);
        {
            auto scope = profiler.scope("estimator");
            estimator.startRun(armors, delta_t, imu_data, serial_port, current_mode);
        }

        if (GlobalParam::STREAM_ENABLE && !detector.visualization_frame.empty()) {
            auto scope = profiler.scope("draw_reprojection");
            detector.drawReprojection(detector.visualization_frame, armors);
        }
    } else {
        LOG(ERROR) << "No Armor Detected in Normal Aim Mode.";
    }

    if (GlobalParam::STREAM_ENABLE && !detector.visualization_frame.empty()) {
        auto scope = profiler.scope("draw_pitch_plot");
        detector.drawMode1PitchPlot(detector.visualization_frame);
    }

    {
        auto scope = profiler.scope("stream");
        pushOutputFrame(detector, frame_image);
    }
}

void handleAntiTop(detector::Detector& detector,
                   estimator::Estimator& estimator,
                   const detector::DetectResult& detect_result,
                   double delta_t,
                   const SerialPortData& imu_data,
                   driver::SerialPort* serial_port,
                   const cv::Mat& frame_image,
                   int current_mode,
                   FrameProfiler& profiler)
{
    LOG(WARNING) << "  >>>>>>>>>>>>>>>            IN AntiTop            <<<<<<<<<<<<<<<<" << std::endl;

    if (!detect_result.armors.empty()) {
        Armors armors = buildArmors(detect_result.armors);
        {
            auto scope = profiler.scope("estimator");
            estimator.startRun(armors, delta_t, imu_data, serial_port, current_mode);
        }

        if (GlobalParam::STREAM_ENABLE && !detector.visualization_frame.empty()) {
            auto scope = profiler.scope("draw_antitop");
            detector.drawReprojection(detector.visualization_frame, armors);
            detector.drawAntiTopStatus(detector.visualization_frame);
        }
    } else {
        LOG(ERROR) << "No Armor Detected in AntiTop Mode.";
        GlobalParam::ANTITOP_SHOOT_FLAG = false;
        if (GlobalParam::STREAM_ENABLE && !detector.visualization_frame.empty()) {
            auto scope = profiler.scope("draw_antitop");
            detector.drawAntiTopStatus(detector.visualization_frame);
        }
    }

    {
        auto scope = profiler.scope("stream");
        pushOutputFrame(detector, frame_image);
    }
}

void handleAutoAim(detector::Detector& detector,
                   tracker::Tracker& tracker,
                   predictor::Predictor& predictor,
                   solver::Solver& solver,
                   controller::Controller& motion_controller,
                   const detector::DetectResult& detect_result,
                   const TimeImageData& frame,
                   const SerialPortData& imu_data,
                   driver::SerialPort* serial_port,
                   const ImuData& imu,
                   FrameProfiler& profiler)
{
    LOG(WARNING) << "  >>>>>>>>>>>>>>>            IN AutoAim            <<<<<<<<<<<<<<<<" << std::endl;

    tracker::TrackResultPairs track_results;
    {
        auto scope = profiler.scope("tracker");
        tracker.merge(detect_result.armors);
        tracker.merge(detect_result.cars);
        track_results = tracker.getTrackResult(frame.timestamp, imu);
    }

    {
        auto scope = profiler.scope("solver");
        solver.solveAndFilterTrackResults(track_results.first, track_results.second, imu);
    }
    {
        auto scope = profiler.scope("predictor_update");
        predictor.update(track_results, frame.timestamp);
    }

    controller::ControlResult result;
    {
        auto scope = profiler.scope("controller");
        motion_controller.setCurrentObservations(track_results.first);
        result = motion_controller.control(imu_data);
    }

    SerialParam::right_clicked = imu_data.right_clicked;
    if (result.target_car_id > 0) {
        auto scope = profiler.scope("serial_write");
        SerialParam::send_data.yaw = result.yaw_setpoint;
        SerialParam::send_data.pitch = result.pitch_setpoint;
        SerialParam::send_data.shootStatus = result.shoot_flag ? 1 : 0;
        SerialParam::send_data.num = static_cast<unsigned char>(result.target_car_id);
        serial_port->writeData(&SerialParam::send_data);
    }

    int predict_ms = static_cast<int>(result.flyTime_s * 1000);
    if (predict_ms < 50) {
        predict_ms = 100;
    }
    predictor::Predictions predictions_debug;
    {
        auto scope = profiler.scope("predictor_predict");
        predictions_debug = predictor.predict(frame.timestamp + std::chrono::milliseconds(predict_ms));
    }

    {
        auto scope = profiler.scope("visualizer_stream");
        cv::Mat& viz_image = !detector.visualization_frame.empty()
            ? detector.visualization_frame
            : const_cast<cv::Mat&>(frame.image);
        PYD viz_imu = static_cast<PYD>(imu);
        lyutils::Visualizer::drawAll(viz_image,
                                     track_results.first,
                                     track_results.second,
                                     predictions_debug,
                                     viz_imu,
                                     result.target_car_id,
                                     result.target_armor_id);
        lyutils::Visualizer::drawControlInfo(viz_image, result);
        detector.pushStream(viz_image);
    }
}

void handleAntiBase(detector::Detector& detector,
                    antibase::AntiBaseManager& antibase_manager,
                    antibase::AntiBaseTransmitter& antibase_transmitter,
                    driver::SerialPort& serial_port,
                    const cv::Mat& frame_image,
                    FrameProfiler& profiler)
{
    antibase::AntiBaseOutput output;
    {
        auto scope = profiler.scope("antibase_process");
        output = antibase_manager.process(frame_image, antibase_transmitter.feedback());
    }
    {
        auto scope = profiler.scope("antibase_send");
        antibase_transmitter.send(output, serial_port);
    }

    {
        auto scope = profiler.scope("stream");
        // output.visual_frame 是与编码器输入完全相同的预处理输出。编码抽帧时
        // 复用最近一帧，确保本地推流不会在原始大图和 320 图之间来回切换。
        static cv::Mat last_pre_encode_frame;
        if (!output.visual_frame.empty()) {
            last_pre_encode_frame = output.visual_frame;
        }
        if (AntiBaseParam::stream_pre_encode_frame && !last_pre_encode_frame.empty()) {
            // /stream 是裸 MJPEG，浏览器会按 JPEG 实际尺寸展示。仅放大调试
            // 画面，不参与 H.264 编码；最近邻可保留 320 图原始像素边界，便于
            // 与 client 解码图对照。
            cv::Mat enlarged_pre_encode_frame;
            cv::resize(last_pre_encode_frame, enlarged_pre_encode_frame,
                       cv::Size(960, 960), 0.0, 0.0, cv::INTER_NEAREST);
            detector.pushStream(enlarged_pre_encode_frame);
        } else {
            detector.pushStream(frame_image);
        }
    }
}

void notifyAllThreads()
{
    Thread::cond_is_process.notify_all();
    Thread::cond2_is_process.notify_all();
    Thread::cond3_is_process.notify_all();
    Thread::cond_is_update.notify_all();
    Thread::cond2_is_update.notify_all();
    Thread::cond3_is_update.notify_all();
}

void requestStop(const char* reason)
{
    LOG(WARNING) << "Requesting shutdown: " << reason;
    g_running = false;
    Thread::should_stop = true;
    notifyAllThreads();
}

void watchdogThread()
{
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!g_running) {
            break;
        }

        auto now = std::chrono::steady_clock::now();
        auto last = g_last_heartbeat.load();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last).count();
        if (elapsed >= WATCHDOG_TIMEOUT_SEC) {
            LOG(ERROR) << "[Watchdog] Main loop blocked for " << elapsed << " seconds.";
            requestStop("watchdog timeout");
            std::this_thread::sleep_for(std::chrono::seconds(WATCHDOG_GRACE_SEC));
            if (!g_shutdown_done) {
                LOG(ERROR) << "[Watchdog] Graceful shutdown timed out. Forcing exit...";
                std::_Exit(2);
            }
            break;
        }
    }
}

void signalHandler(int signum)
{
    LOG(WARNING) << "Interrupt signal (" << signum << ") received, shutting down...";
    requestStop("signal");
}

ImuData toImuData(const SerialPortData& imu_data)
{
    ImuData imu;
    imu.pitch = static_cast<double>(imu_data.pitch);
    imu.yaw = static_cast<double>(imu_data.yaw);
    imu.roll = 0.0;
    return imu;
}

void joinCamera(CameraRuntime& runtime)
{
    if (runtime.thread.joinable()) {
        runtime.thread.join();
    }
}

void stopAndJoinSerial(driver::SerialPort& serial_port, std::thread& serial_thread)
{
    serial_port.stop();
    if (serial_thread.joinable()) {
        serial_thread.join();
    }
}

} // namespace

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    auto log = std::make_unique<Log>();
    log->init(argv[0]);

    auto config = std::make_unique<Config>(confog_file_path);
    config->parse();

    Params_ToSerialPort params_to_serial_port(&SerialParam::recv_data);
    auto serial_port = std::make_unique<driver::SerialPort>(SerialParam::device_name);
    std::thread serial_thread(&driver::SerialPort::read_data,
                              serial_port.get(),
                              std::ref(params_to_serial_port));

    CameraRuntime aim_camera;
    CameraRuntime aim_camera_2;
    CameraRuntime base_camera;
    const bool dual_camera_enabled = isDualCameraEnabled();
    if (!startCamera(aim_camera, driver::CameraProfile::Aim8mm) ||
        (CameraParam::use_8mm2 && !startCamera(aim_camera_2, driver::CameraProfile::Aim8mm2)) ||
        (dual_camera_enabled && !startCamera(base_camera, driver::CameraProfile::Base))) {
        LOG(ERROR) << "Failed to create camera captures";
        requestStop("camera init failed");
        joinCamera(aim_camera);
        joinCamera(aim_camera_2);
        joinCamera(base_camera);
        stopAndJoinSerial(*serial_port, serial_thread);
        g_shutdown_done = true;
        return 1;
    }

    std::thread watchdog_thread(watchdogThread);
    LOG(INFO) << "[Watchdog] Started with " << WATCHDOG_TIMEOUT_SEC << "s timeout";

    auto detector = std::make_unique<detector::Detector>(
        DetectorParam::armor_model_path,
        DetectorParam::car_model_path);
    auto tracker = std::make_unique<tracker::Tracker>();
    auto predictor = std::make_unique<predictor::Predictor>();
    auto solver = std::make_shared<solver::Solver>();
    auto estimator = std::make_unique<estimator::Estimator>(solver);
    auto motion_controller = std::make_unique<controller::Controller>();
    auto antibase_manager = std::make_unique<antibase::AntiBaseManager>();
    // Mode1 与 Mode4 的输入来自不同相机；独立管理器避免背景模型与 H.264
    // 参考帧跨相机串扰。两者仍共用同一个发送器，整包间隔始终由其统一保证。
    auto mode1_antibase_manager = std::make_unique<antibase::AntiBaseManager>();
    auto antibase_transmitter = std::make_unique<antibase::AntiBaseTransmitter>();

    enum class AntiBaseInputSource { None, Mode1Aim, Mode4Base };
    AntiBaseInputSource active_antibase_source = AntiBaseInputSource::None;

    std::unique_ptr<detector::VideoSaver> video_saver;
    if (GlobalParam::SAVE_VIDEO) {
        video_saver = std::make_unique<detector::VideoSaver>();
        video_saver->initSaver();
    }

    location::Location::registerSolver(solver);
    motion_controller->registPredictFunc(
        [&predictor](Time::TimeStamp t) {
            return predictor->predict(t);
        });

    int frame_count = 0;
    while (g_running) {
        g_last_heartbeat.store(std::chrono::steady_clock::now());
        FrameProfiler profiler(frame_count + 1);

        SerialPortData imu_data;
        {
            auto scope = profiler.scope("serial_snapshot");
            std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
            imu_data = SerialParam::recv_data;
        }
        applyBaseExposureAdjustment(*serial_port, base_camera);

        int current_mode = detector->determineOperationMode(imu_data.flag);
        TimeImageData frame;
        {
            auto scope = profiler.scope("fetch_frame");
            if (!fetchMatchedFrameForMode(*detector, *serial_port,
                                          aim_camera, aim_camera_2, base_camera, dual_camera_enabled,
                                          imu_data, current_mode, frame)) {
                continue;
            }
        }

        {
            auto scope = profiler.scope("refresh_frame");
            if (!refreshLatestAimFrameIfNeeded(*detector, *serial_port,
                                                aim_camera, aim_camera_2, base_camera, dual_camera_enabled,
                                                imu_data, current_mode, frame)) {
                continue;
            }
        }

        // 自瞄解算必须与实际取帧相机使用同一套标定。Mode4 为纯 AntiBase
        // 图像链路，setCameraProfile(Base) 保持当前解算标定即可。
        solver->setCameraProfile(selectProfileForMode(current_mode, dual_camera_enabled));

        const double imu_match_delta_ms = imu_data.valid
            ? std::chrono::duration<double, std::milli>(
                imu_data.recv_time > frame.steady_timestamp
                    ? imu_data.recv_time - frame.steady_timestamp
                    : frame.steady_timestamp - imu_data.recv_time).count()
            : -1.0;
        // Mode4 不消费这组 IMU/串口状态；保留链路日志即可。
        const bool log_mode4_main = current_mode != 4;
        if (log_mode4_main) {
            LOG(INFO) << "[Serial Match] mode=" << static_cast<int>(imu_data.flag)
                      << " color=" << static_cast<int>(imu_data.color)
                      << " right_clicked=" << static_cast<int>(imu_data.right_clicked)
                      << " imu_match_ms=" << imu_match_delta_ms;
        }

        if (frame.image.empty()) {
            LOG(ERROR) << "[Main] Received empty frame!";
            continue;
        }
        if (video_saver) {
            auto scope = profiler.scope("save_video");
            video_saver->SaveVideo(frame.image);
        }
        ++frame_count;
        if (current_mode != 4 && frame_count % 100 == 1) {
            LOG(INFO) << "[Main] Frame #" << frame_count
                      << " size: " << frame.image.cols << "x" << frame.image.rows;
        }

        if (log_mode4_main) {
            LOG(INFO) << "[IMU] yaw=" << imu_data.yaw << " pitch=" << imu_data.pitch;
        }
        const int color = GlobalParam::DEBUG_MODE ? GlobalParam::COLOR : imu_data.color;
        const ImuData imu = toImuData(imu_data);

        Time::TimeStamp start = Time::TimeStamp::now();
        detector::DetectResult detect_result;
        if (current_mode != 4) {
            auto scope = profiler.scope("detector");
            detect_result = detector->startdetect(frame.image, current_mode, color);
        }
        frame.timestamp = Time::TimeStamp::now();
        const double delta_t = (frame.timestamp - start).count();

        // 两个管理器各自维护背景/拖影/H.264 参考帧；发送器则是唯一的串口
        // 节拍源。切换时先取消未发出的旧包，再重建新源编码器，使客户端从
        // 新的 SPS/PPS + IDR 开始解码，不能把两台相机的码流拼在一起。
        const AntiBaseInputSource desired_antibase_source = current_mode == 1
            ? AntiBaseInputSource::Mode1Aim
            : (current_mode == 4 ? AntiBaseInputSource::Mode4Base
                                 : AntiBaseInputSource::None);
        if (desired_antibase_source != active_antibase_source) {
            const std::size_t discarded_packets = antibase_transmitter->clearPendingPackets();
            if (desired_antibase_source == AntiBaseInputSource::Mode1Aim) {
                mode1_antibase_manager->resetForSourceSwitch();
            } else if (desired_antibase_source == AntiBaseInputSource::Mode4Base) {
                antibase_manager->resetForSourceSwitch();
            }
            LOG(INFO) << "[AntiBase source] "
                      << (active_antibase_source == AntiBaseInputSource::Mode1Aim ? "mode1_aim" :
                          active_antibase_source == AntiBaseInputSource::Mode4Base ? "mode4_base" : "none")
                      << " -> "
                      << (desired_antibase_source == AntiBaseInputSource::Mode1Aim ? "mode1_aim" :
                          desired_antibase_source == AntiBaseInputSource::Mode4Base ? "mode4_base" : "none")
                      << ", discarded_pending=" << discarded_packets;
            active_antibase_source = desired_antibase_source;
        }

        switch (current_mode) {
            case 1:
                handleNormalAim(*detector, *estimator, detect_result, delta_t,
                                imu_data, serial_port.get(), frame.image, current_mode, profiler);
                {
                    // Mode1 同步复用 Mode4 的预处理、编码、分包和限速发送链路。
                    // 取与本地推流一致的检测可视化图，确保检测框也被编码进画面。
                    const cv::Mat& antibase_input = !detector->visualization_frame.empty()
                        ? detector->visualization_frame
                        : frame.image;
                    handleAntiBase(*detector, *mode1_antibase_manager, *antibase_transmitter,
                                   *serial_port, antibase_input, profiler);
                }
                break;
            case 2:
                handleAntiTop(*detector, *estimator, detect_result, delta_t,
                              imu_data, serial_port.get(), frame.image, current_mode, profiler);
                break;
            case 3:
                handleAutoAim(*detector, *tracker, *predictor, *solver, *motion_controller,
                              detect_result, frame, imu_data, serial_port.get(), imu, profiler);
                break;
            case 4:
                handleAntiBase(*detector, *antibase_manager, *antibase_transmitter,
                               *serial_port, frame.image, profiler);
                break;
            default:
                break;
        }
        profiler.log(current_mode, frame.image.cols, frame.image.rows);
    }

    LOG(INFO) << "Cleaning up resources...";
    requestStop("main loop exit");

    joinCamera(aim_camera);
    joinCamera(aim_camera_2);
    joinCamera(base_camera);
    stopAndJoinSerial(*serial_port, serial_thread);

    g_shutdown_done = true;
    if (watchdog_thread.joinable()) {
        watchdog_thread.join();
    }
    LOG(INFO) << "Shutdown complete.";
    return 0;
}
