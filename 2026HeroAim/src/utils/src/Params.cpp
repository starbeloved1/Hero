#include "../include/Params.hpp"
#include "cmath"
using namespace std;

namespace lyutils{
    int CameraParam::device_type;
    string CameraParam::sn;
    bool CameraParam::use_base_camera = false;
    bool CameraParam::use_8mm2 = false;
    string CameraParam::picture_path;
    string CameraParam::video_path;
    int CameraParam::exposure_time;
    int CameraParam::exposure_min_us;
    int CameraParam::exposure_max_us;
    double CameraParam::acquisition_fps = 25.0;
    float CameraParam::gamma;
    double CameraParam::gain;
    int CameraParam::camera_type;
    double CameraParam::height;
    double CameraParam::width;

    string Camera8mmParam::sn;
    int Camera8mmParam::exposure_time;
    double Camera8mmParam::gain;
    float Camera8mmParam::gamma;
    double Camera8mmParam::fx;
    double Camera8mmParam::fy;
    double Camera8mmParam::u0;
    double Camera8mmParam::v0;
    double Camera8mmParam::k1;
    double Camera8mmParam::k2;
    double Camera8mmParam::k3;
    double Camera8mmParam::p1;
    double Camera8mmParam::p2;
    double Camera8mmParam::trans_x;
    double Camera8mmParam::trans_y;
    double Camera8mmParam::trans_z;
    double Camera8mmParam::pitch;
    double Camera8mmParam::yaw;
    double Camera8mmParam::roll;

    string Camera8mm2Param::sn;
    int Camera8mm2Param::exposure_time;
    double Camera8mm2Param::gain;
    float Camera8mm2Param::gamma;
    double Camera8mm2Param::fx;
    double Camera8mm2Param::fy;
    double Camera8mm2Param::u0;
    double Camera8mm2Param::v0;
    double Camera8mm2Param::k1;
    double Camera8mm2Param::k2;
    double Camera8mm2Param::k3;
    double Camera8mm2Param::p1;
    double Camera8mm2Param::p2;
    double Camera8mm2Param::trans_x;
    double Camera8mm2Param::trans_y;
    double Camera8mm2Param::trans_z;
    double Camera8mm2Param::pitch;
    double Camera8mm2Param::yaw;
    double Camera8mm2Param::roll;

    double LidarParam::trans_x;
    double LidarParam::trans_y;
    double LidarParam::trans_z;
    double LidarParam::pitch;
    double LidarParam::yaw;
    double LidarParam::roll;

    int DetectorParam::color;
    int DetectorParam::thresh;
    int DetectorParam::detect_way;
    float DetectorParam::score_threshold;
    float DetectorParam::nms_threshold = 0.45f;
    string DetectorParam::armor_infer_device = "CPU";
    string DetectorParam::armor_model_path;
    string DetectorParam::car_model_path;

    string SerialParam::device_name;
    SerialPortData SerialParam::recv_data;
    SerialPortWriteData SerialParam::send_data;
    vector<SerialPortData> SerialParam::serial_data_sets(1000);
    std::mutex SerialParam::serial_mutex;
    int SerialParam::set_id = 0;
    int SerialParam::direction = -1;
    cv::Point2f SerialParam::shoot_center;
    bool SerialParam::right_clicked = false;

    double ControllerParam::pic_camera_x;
    double ControllerParam::pic_camera_y;
    bool ControllerParam::mouse_require;
    double ControllerParam::armor_fire_window;
    double ControllerParam::speed_fire_window;
    int ControllerParam::phase_confirm_frames;
    int ControllerParam::speed_confirm_frames;
    double ControllerParam::accel_threshold;
    double ControllerParam::accel_stable_threshold;
    int ControllerParam::accel_stable_frames;

    bool GlobalParam::DEBUG_MODE;
    bool GlobalParam::STREAM_ENABLE;  // Independent stream switch.
    int GlobalParam::COLOR;
    int GlobalParam::BIGID;
    bool GlobalParam::SAVE_VIDEO;
    int GlobalParam::save_step;
    bool GlobalParam::SHOW_THRESH;
    bool GlobalParam::SOCKET;
    double GlobalParam::SHOOT_SPEED;
    int GlobalParam::MODE;
    bool GlobalParam::IS_NUC;
    bool GlobalParam::SHOW_SERIAL;
    bool GlobalParam::AUTOAIM_LATEST_FRAME_PREFER = true;
    int GlobalParam::AUTOAIM_LATEST_FRAME_REFRESH_MS = 4;
    int GlobalParam::AUTOAIM_MAX_FRAME_REFRESHES = 1;
    double GlobalParam::BALLISTIC_DRAG_COEFF = 0.30;
    double GlobalParam::BALLISTIC_GRAVITY = 9.794;
    double GlobalParam::BALLISTIC_AIR_DENSITY = 1.169;
    double GlobalParam::BALLISTIC_BULLET_MASS = 0.041;
    double GlobalParam::BALLISTIC_BULLET_RADIUS = 0.02125;
    double GlobalParam::BALLISTIC_MUZZLE_OFFSET = 0.29;

    // Outpost mode adjustment parameters.
    double OutpostParam::time_bias;
    double OutpostParam::time_bias_inverse;
    double OutpostParam::shoot_distance_offset = 0.0;
    double OutpostParam::tmp_time;

    STATE StateParam::state;

    std::chrono::steady_clock::time_point TimeSystem::time_zero;
    

    float SlamParam::x=0.0;
    float SlamParam::y=0.0;
    float SlamParam::z=0.0;

    bool GlobalParam::IS_CALIBRATED = false;
    std::vector<double> GlobalParam::Z_MAP;
    bool GlobalParam::ANTITOP_IN_SHOOT_ZONE = false;
    int  GlobalParam::ANTITOP_PITCH_STATE = 0;
    int  GlobalParam::ANTITOP_LAST_ZONE_INDEX = -1;
    int  GlobalParam::ANTITOP_DATA_COUNT = 0;
    int  GlobalParam::ANTITOP_AVG_MS = -1;
    bool GlobalParam::ANTITOP_SHOOT_FLAG = false;
    double GlobalParam::ANTITOP_TARGET_Z = 0.0;
    double GlobalParam::ANTITOP_TARGET_PITCH = 0.0;
    bool GlobalParam::ANTITOP_TARGET_VALID = false;
    double GlobalParam::ANTITOP_TRACKING_Z = 0.0;
    bool GlobalParam::ANTITOP_TRACKING_Z_VALID = false;
    uint64_t GlobalParam::ANTITOP_TRACKING_Z_SEQ = 0;

    bool AntiBaseParam::udp_debug_enabled;
    std::string AntiBaseParam::udp_debug_host;
    int AntiBaseParam::udp_debug_port;
    int AntiBaseParam::output_fps;
    bool AntiBaseParam::stream_pre_encode_frame = false;
    int AntiBaseParam::packet_size;
    double AntiBaseParam::max_tx_delay_s;
    double AntiBaseParam::limiter_min_packet_gap_ms = 28.0;
    bool AntiBaseParam::adaptive_bitrate_enabled = true;
    int AntiBaseParam::adaptive_bitrate_min_kbps = 90;
    int AntiBaseParam::adaptive_bitrate_max_kbps = 160;
    int AntiBaseParam::adaptive_bitrate_step_up_kbps = 5;
    int AntiBaseParam::adaptive_bitrate_step_down_kbps = 10;
    double AntiBaseParam::adaptive_bitrate_control_interval_s = 1.0;
    int AntiBaseParam::crop_size;
    int AntiBaseParam::crop_offset_x = 0;
    int AntiBaseParam::output_size;
    bool AntiBaseParam::static_simplify;
    int AntiBaseParam::motion_threshold;
    int AntiBaseParam::motion_erode_px;
    int AntiBaseParam::motion_dilate_px;
    int AntiBaseParam::motion_trail_frames;
    double AntiBaseParam::trail_brightness_gain = 1.0;
    double AntiBaseParam::trail_disable_motion_ratio;
    double AntiBaseParam::trail_resume_motion_ratio = 0.25;
    double AntiBaseParam::motion_ratio_ema_alpha = 0.2;
    double AntiBaseParam::bg_update_alpha;
    double AntiBaseParam::bg_blur_sigma;
    int AntiBaseParam::center_clear_size;
    bool AntiBaseParam::force_monochrome;
    int AntiBaseParam::target_bitrate_kbps;
    bool AntiBaseParam::log_packet_content = true;
    int AntiBaseParam::h264_speed_preset = 9;
    int AntiBaseParam::h264_tune = 4;
    int AntiBaseParam::h264_key_int_max = 30;
    int AntiBaseParam::h264_bframes = 0;
    int AntiBaseParam::h264_rc_lookahead = 0;
    int AntiBaseParam::h264_sync_lookahead = 0;
    bool AntiBaseParam::h264_sliced_threads = true;
    int AntiBaseParam::h264_ref = 1;
    int AntiBaseParam::h264_vbv_buf_capacity = 120;
    std::string AntiBaseParam::h264_option_string =
        "repeat-headers=1:scenecut=0:intra-refresh=0:open-gop=0:aq-mode=1:mbtree=0:force-cfr=1";

    double PredictorParam::init_radius = 0.2;
    int PredictorParam::max_lost_count = 10;
    int PredictorParam::min_consecutive_detections = 3;
    double PredictorParam::low_speed_process_noise_xy = 100.0;
    double PredictorParam::low_speed_process_noise_z = 100.0;
    double PredictorParam::low_speed_process_noise_yaw = 400.0;
    double PredictorParam::middle_speed_process_noise_xy = 100.0;
    double PredictorParam::middle_speed_process_noise_z = 100.0;
    double PredictorParam::middle_speed_process_noise_yaw = 400.0;
    double PredictorParam::high_speed_process_noise_xy = 100.0;
    double PredictorParam::high_speed_process_noise_z = 100.0;
    double PredictorParam::high_speed_process_noise_yaw = 400.0;
    double PredictorParam::middle_speed_angular_velocity_threshold = 2.0;
    double PredictorParam::high_speed_angular_velocity_threshold = 4.0;
    double PredictorParam::measurement_noise_yaw = 0.004;
    double PredictorParam::measurement_noise_pitch = 0.004;
    double PredictorParam::measurement_noise_distance_base = 0.1;
    double PredictorParam::measurement_noise_armor_yaw_base = 0.09;
    double PredictorParam::min_valid_radius = 0.05;
    double PredictorParam::max_valid_radius = 0.5;
}

