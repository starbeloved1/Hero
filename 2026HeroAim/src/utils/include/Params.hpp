#ifndef AUTOAIM_PARAMS_HPP
#define AUTOAIM_PARAMS_HPP

#include <string>
#include <cstdint>
#include <mutex>
#include "SerialPort.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace driver;

namespace lyutils{
    class CameraParam{
    public:
        static int device_type;
        static string sn;
        static bool use_base_camera;
        // true 时仅 Mode1 使用 camera_8mm_2；Mode2/3 始终使用原 camera_8mm。
        static bool use_8mm2;
        static string video_path;
        static string picture_path;
        static int exposure_time;
        static int exposure_min_us;
        static int exposure_max_us;
        // 相机硬件采集频率；0 表示关闭硬件限帧，恢复原连续采集行为。
        static double acquisition_fps;
        static double gain;
        static float gamma;
        static int camera_type;

        static double height;
        static double width;
    };

    class Camera8mmParam{
    public:
        static string sn;
        static int exposure_time;
        static double gain;
        static float gamma;
        static double yaw;
        static double pitch;
        static double roll;

        static double fx; 
        static double fy;
        static double u0;
        static double v0;
        static double k1;
        static double k2;
        static double k3;
        static double p1;
        static double p2;
        static double trans_x;
        static double trans_y;
        static double trans_z;
    };

    // 第二台 8mm 的独立开机参数。标定字段默认与原 8mm 相同，SN 单独配置。
    class Camera8mm2Param{
    public:
        static string sn;
        static int exposure_time;
        static double gain;
        static float gamma;
        static double yaw;
        static double pitch;
        static double roll;
        static double fx;
        static double fy;
        static double u0;
        static double v0;
        static double k1;
        static double k2;
        static double k3;
        static double p1;
        static double p2;
        static double trans_x;
        static double trans_y;
        static double trans_z;
    };

    class LidarParam{
        public:
        static double trans_x;
        static double trans_y;
        static double trans_z;
        static double yaw;
        static double pitch;
        static double roll;
    };

    class DetectorParam{
    public:
        static int color;
        static int thresh;
        static int detect_way;
        static float score_threshold;
        static float nms_threshold;
        static string armor_infer_device;
        static string armor_model_path;
        static string car_model_path;
    };


    class SlamParam{
    public:
        static float x;
        static float y;
        static float z;

    };

    class OutpostParam{
    public:
        static double time_bias;
        static double time_bias_inverse;
        static double center_ratio;
        static double shoot_distance_offset;
        static double tmp_time;
    };


    class SerialParam{
    public:
        static bool enable;
        static string device_name;
        static SerialPortData recv_data;
        static SerialPortWriteData send_data;
        static vector<SerialPortData> serial_data_sets;
        static std::mutex serial_mutex;
        static int set_id;
        static int direction;
        static cv::Point2f shoot_center;
        static bool right_clicked;
    };

    class ControllerParam {
    public:
        static double pic_camera_x;
        static double pic_camera_y;
        static bool mouse_require;
        static double armor_fire_window;   // Low-speed fire window in degrees.
        static double speed_fire_window;   // High-speed fire window in degrees.
        static int phase_confirm_frames;
        static int speed_confirm_frames;
        static double accel_threshold;
        static double accel_stable_threshold;
        static int accel_stable_frames;
    };

    class GlobalParam{
    public:
        static int MODE;
        static int COLOR;
        static int BIGID;
        static bool DEBUG_MODE;
        static bool STREAM_ENABLE;  // Independent stream switch.
        static bool SAVE_VIDEO;
        static bool SHOW_THRESH;
        static int save_step;
        static double SHOOT_SPEED;
        static bool SOCKET;
        static bool IS_NUC;
        static bool SHOW_SERIAL;
        static bool AUTOAIM_LATEST_FRAME_PREFER;
        static int AUTOAIM_LATEST_FRAME_REFRESH_MS;
        static int AUTOAIM_MAX_FRAME_REFRESHES;
        static double BALLISTIC_DRAG_COEFF;
        static double BALLISTIC_GRAVITY;
        static double BALLISTIC_AIR_DENSITY;
        static double BALLISTIC_BULLET_MASS;
        static double BALLISTIC_BULLET_RADIUS;
        static double BALLISTIC_MUZZLE_OFFSET;
        static bool IS_CALIBRATED;
        static std::vector<double> Z_MAP;
        static bool ANTITOP_IN_SHOOT_ZONE;
        static int  ANTITOP_PITCH_STATE;  // 0=UNLOCKED 1=LOCK_COUNTDOWN 2=LOCK_POST_FIRE
        static int  ANTITOP_LAST_ZONE_INDEX;
        static int  ANTITOP_DATA_COUNT;
        static int  ANTITOP_AVG_MS;
        static bool ANTITOP_SHOOT_FLAG;
        static double ANTITOP_TARGET_Z;      // Current z used for anti-top aiming in meters.
        static double ANTITOP_TARGET_PITCH;  // Current pitch command for anti-top target in degrees.
        static bool ANTITOP_TARGET_VALID;    // Whether target z/pitch are valid for visualization.
        static double ANTITOP_TRACKING_Z;      // Current tracked armor z in meters.
        static bool ANTITOP_TRACKING_Z_VALID;  // Whether current z is valid.
        static uint64_t ANTITOP_TRACKING_Z_SEQ; // Monotonic update sequence for visualization.
    };

    class AntiBaseParam {
    public:
        static bool udp_debug_enabled;
        static std::string udp_debug_host;
        static int udp_debug_port;
        static int output_fps;
        // Mode 4 本地调试推流是否显示送入 H.264 前的预处理画面。
        static bool stream_pre_encode_frame;
        static int packet_size;
        static double max_tx_delay_s;
        static double limiter_min_packet_gap_ms;
        static bool adaptive_bitrate_enabled;
        static int adaptive_bitrate_min_kbps;
        static int adaptive_bitrate_max_kbps;
        static int adaptive_bitrate_step_up_kbps;
        static int adaptive_bitrate_step_down_kbps;
        static double adaptive_bitrate_control_interval_s;
        static int crop_size;
        static int crop_offset_x;
        static int output_size;
        static bool static_simplify;
        static int motion_threshold;
        static int motion_erode_px;
        static int motion_dilate_px;
        static int motion_trail_frames;
        static double trail_brightness_gain;
        static double trail_disable_motion_ratio;
        static double trail_resume_motion_ratio;
        static double motion_ratio_ema_alpha;
        static double bg_update_alpha;
        static double bg_blur_sigma;
        static int center_clear_size;
        static bool force_monochrome;
        static int target_bitrate_kbps;
        static bool log_packet_content;
        static int h264_speed_preset;
        static int h264_tune;
        static int h264_key_int_max;
        static int h264_bframes;
        static int h264_rc_lookahead;
        static int h264_sync_lookahead;
        static bool h264_sliced_threads;
        static int h264_ref;
        static int h264_vbv_buf_capacity;
        static std::string h264_option_string;
    };

    class PredictorParam {
    public:
        static double init_radius;
        static int max_lost_count;
        static int min_consecutive_detections;
        static double low_speed_process_noise_xy;
        static double low_speed_process_noise_z;
        static double low_speed_process_noise_yaw;
        static double middle_speed_process_noise_xy;
        static double middle_speed_process_noise_z;
        static double middle_speed_process_noise_yaw;
        static double high_speed_process_noise_xy;
        static double high_speed_process_noise_z;
        static double high_speed_process_noise_yaw;
        static double middle_speed_angular_velocity_threshold;
        static double high_speed_angular_velocity_threshold;
        static double measurement_noise_yaw;
        static double measurement_noise_pitch;
        static double measurement_noise_distance_base;
        static double measurement_noise_armor_yaw_base;
        static double min_valid_radius;
        static double max_valid_radius;
    };

    typedef enum {
        AUTOAIM,
        ANTITOP,
        OUTPOST,
        Half_OUTPOST,
        AUTOAIM_WITH_ROI,
        GREEN_SHOOT
    } STATE;

    typedef enum {
        HERO,
        SENTRY,
        ENGINEER,
    } PRI;

    enum class OperationMode {
        NormalAim = 0x05,
        AntiTop = 0x07,
        AutoAim = 0x09,
        Default = NormalAim
    };

    class StateParam{
    public:
        static STATE state;
    };

    class TimeSystem{
    public:
        static std::chrono::steady_clock::time_point time_zero;
    };
}

#endif //AUTOAIM_PARAMS_HPP

