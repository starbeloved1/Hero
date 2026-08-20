#include "../include/Config.hpp"

using namespace std;
using namespace Json;

namespace lyutils{
    Config::Config(const string &path)
    {
        this->json_file_path = path;
    }

    void Config::parse()
    {
        fstream json_file(json_file_path, std::ios::in);
        LOG_IF(ERROR, !json_file.is_open()) << "can't find json file in " << json_file_path;
        LOG_IF(INFO, json_file.is_open()) << "successfully open json file: " << json_file_path;

        JSONCPP_STRING errs;
        CharReaderBuilder builder;
        Value root;
        bool status = Json::parseFromStream(builder, json_file, &root, &errs);
        LOG_IF(ERROR, !status) << "json file parse error!";
        /*** camera param ***/
        const Value& camera = root["camera"];
        const Value& camera_base = camera["camera_base"];
        const Value& camera_8mm = camera.isMember("camera_8mm")
            ? camera["camera_8mm"]
            : root["camera_8mm"];

        CameraParam::device_type = camera["device_type"].asInt();
        CameraParam::sn = camera_base["sn"].asString();
        CameraParam::use_base_camera = camera.isMember("usebasecamera")
            ? camera["usebasecamera"].asBool()
            : false;
        CameraParam::use_8mm2 = camera.get("use8mm2", false).asBool();
        CameraParam::video_path = camera["video_path"].asString();
        CameraParam::picture_path = camera["picture_path"].asString();
        CameraParam::camera_type = camera["camera_type"].asInt();
        CameraParam::exposure_time = camera_base["exposure_time"].asInt();
        CameraParam::exposure_min_us = camera_base.get("exposure_min_us", 2000).asInt();
        CameraParam::exposure_max_us = camera_base.get("exposure_max_us", 25000).asInt();
        CameraParam::acquisition_fps = camera_base.get(
            "acquisition_fps", CameraParam::acquisition_fps).asDouble();
        CameraParam::gain = camera_base["gain"].asDouble();
        CameraParam::gamma = camera_base["gamma"].asFloat();
        CameraParam::width = camera_base["width"].asDouble();
        CameraParam::height = camera_base["height"].asDouble();


        Camera8mmParam::sn = camera_8mm["sn"].asString();
        Camera8mmParam::exposure_time = camera_8mm["exposure_time"].asInt();
        Camera8mmParam::gain = camera_8mm["gain"].asDouble();
        Camera8mmParam::gamma = camera_8mm["gamma"].asFloat();
        Camera8mmParam::fx = camera_8mm["fx"].asDouble();
        Camera8mmParam::fy = camera_8mm["fy"].asDouble();
        Camera8mmParam::u0 = camera_8mm["u0"].asDouble();
        Camera8mmParam::v0 = camera_8mm["v0"].asDouble();
        Camera8mmParam::k1 = camera_8mm["k1"].asDouble();
        Camera8mmParam::k2 = camera_8mm["k2"].asDouble();
        Camera8mmParam::k3 = camera_8mm["k3"].asDouble();
        Camera8mmParam::p1 = camera_8mm["p1"].asDouble();
        Camera8mmParam::p2 = camera_8mm["p2"].asDouble();
        Camera8mmParam::yaw = camera_8mm["yaw"].asDouble();
        Camera8mmParam::pitch = camera_8mm["pitch"].asDouble();
        Camera8mmParam::roll = camera_8mm["roll"].asDouble();
        Camera8mmParam::trans_x = camera_8mm["trans_x"].asDouble();
        Camera8mmParam::trans_y = camera_8mm["trans_y"].asDouble();
        Camera8mmParam::trans_z = camera_8mm["trans_z"].asDouble();

        // camera_8mm_2 的非 SN 字段默认沿用原 8mm，便于只填写新相机 SN。
        const Value& camera_8mm_2 = camera["camera_8mm_2"];
        Camera8mm2Param::sn = camera_8mm_2.get("sn", "").asString();
        Camera8mm2Param::exposure_time = camera_8mm_2.get(
            "exposure_time", Camera8mmParam::exposure_time).asInt();
        Camera8mm2Param::gain = camera_8mm_2.get("gain", Camera8mmParam::gain).asDouble();
        Camera8mm2Param::gamma = camera_8mm_2.get("gamma", Camera8mmParam::gamma).asFloat();
        Camera8mm2Param::fx = camera_8mm_2.get("fx", Camera8mmParam::fx).asDouble();
        Camera8mm2Param::fy = camera_8mm_2.get("fy", Camera8mmParam::fy).asDouble();
        Camera8mm2Param::u0 = camera_8mm_2.get("u0", Camera8mmParam::u0).asDouble();
        Camera8mm2Param::v0 = camera_8mm_2.get("v0", Camera8mmParam::v0).asDouble();
        Camera8mm2Param::k1 = camera_8mm_2.get("k1", Camera8mmParam::k1).asDouble();
        Camera8mm2Param::k2 = camera_8mm_2.get("k2", Camera8mmParam::k2).asDouble();
        Camera8mm2Param::k3 = camera_8mm_2.get("k3", Camera8mmParam::k3).asDouble();
        Camera8mm2Param::p1 = camera_8mm_2.get("p1", Camera8mmParam::p1).asDouble();
        Camera8mm2Param::p2 = camera_8mm_2.get("p2", Camera8mmParam::p2).asDouble();
        Camera8mm2Param::yaw = camera_8mm_2.get("yaw", Camera8mmParam::yaw).asDouble();
        Camera8mm2Param::pitch = camera_8mm_2.get("pitch", Camera8mmParam::pitch).asDouble();
        Camera8mm2Param::roll = camera_8mm_2.get("roll", Camera8mmParam::roll).asDouble();
        Camera8mm2Param::trans_x = camera_8mm_2.get("trans_x", Camera8mmParam::trans_x).asDouble();
        Camera8mm2Param::trans_y = camera_8mm_2.get("trans_y", Camera8mmParam::trans_y).asDouble();
        Camera8mm2Param::trans_z = camera_8mm_2.get("trans_z", Camera8mmParam::trans_z).asDouble();


        LidarParam::trans_x = root["lidar"]["trans_x"].asDouble();
        LidarParam::trans_y = root["lidar"]["trans_y"].asDouble();
        LidarParam::trans_z = root["lidar"]["trans_z"].asDouble();



        OutpostParam::time_bias = root["outpost"]["time_bias"].asDouble();
        OutpostParam::time_bias_inverse = root["outpost"]["time_bias_inverse"].asDouble();
        OutpostParam::shoot_distance_offset =
            root["outpost"].get("shoot_distance_offset", 0.0).asDouble();
        OutpostParam::tmp_time = root["outpost"]["tmp_time"].asDouble();


        DetectorParam::color = root["detector"]["color"].asInt();
        DetectorParam::thresh = root["detector"]["thresh"].asInt();
        DetectorParam::detect_way = root["detector"]["detect_way"].asInt();
        DetectorParam::score_threshold = root["detector"].isMember("score_threshold")
            ? root["detector"]["score_threshold"].asFloat()
            : 0.65f;
        DetectorParam::nms_threshold = root["detector"].isMember("nms_threshold")
            ? root["detector"]["nms_threshold"].asFloat()
            : 0.45f;
        DetectorParam::armor_infer_device = root["detector"].isMember("armor_infer_device")
            ? root["detector"]["armor_infer_device"].asString()
            : "CPU";
        DetectorParam::armor_model_path = root["detector"]["armor_model_path"].asString();
        DetectorParam::car_model_path = root["detector"]["car_model_path"].asString();
        SerialParam::device_name = root["serialport"]["deviceName"].asString();

        ControllerParam::pic_camera_x = root["controller"]["pic_camera_x"].asDouble();
        ControllerParam::pic_camera_y = root["controller"]["pic_camera_y"].asDouble();
        ControllerParam::mouse_require = root["controller"]["mouse_require"].asBool();
        ControllerParam::armor_fire_window = root["controller"]["armor_fire_window"].asDouble();
        ControllerParam::speed_fire_window = root["controller"]["speed_fire_window"].asDouble();
        ControllerParam::phase_confirm_frames = root["controller"]["phase_confirm_frames"].asInt();
        ControllerParam::speed_confirm_frames = root["controller"]["speed_confirm_frames"].asInt();
        ControllerParam::accel_threshold = root["controller"]["accel_threshold"].asDouble();
        ControllerParam::accel_stable_threshold = root["controller"]["accel_stable_threshold"].asDouble();
        ControllerParam::accel_stable_frames = root["controller"]["accel_stable_frames"].asInt();

        GlobalParam::MODE = root["debug"]["mode"].asInt();
        GlobalParam::COLOR = root["debug"]["color"].asInt();
        GlobalParam::BIGID = root["debug"]["big_id"].asInt();
        GlobalParam::DEBUG_MODE = root["debug"]["enable"].asBool();
        GlobalParam::STREAM_ENABLE = root["debug"]["stream"].asBool();  // Independent stream switch.
        GlobalParam::SAVE_VIDEO = root["debug"]["video"].asBool();
        GlobalParam::SHOW_THRESH = root["debug"]["thresh"].asBool();
        GlobalParam::save_step = root["debug"]["save_step"].asInt();
        GlobalParam::SHOOT_SPEED = root["debug"]["shoot_speed"].asDouble();
        GlobalParam::SOCKET = root["debug"]["socket"].asBool();
        GlobalParam::IS_NUC = root["debug"]["is_nuc"].asBool();
        GlobalParam::SHOW_SERIAL = root["debug"]["show_serial_data"].asBool();
        if (root["debug"].isMember("latest_frame_prefer")) {
            GlobalParam::AUTOAIM_LATEST_FRAME_PREFER =
                root["debug"]["latest_frame_prefer"].asBool();
        }
        if (root["debug"].isMember("latest_frame_refresh_ms")) {
            GlobalParam::AUTOAIM_LATEST_FRAME_REFRESH_MS =
                root["debug"]["latest_frame_refresh_ms"].asInt();
        }
        if (root["debug"].isMember("latest_frame_max_refreshes")) {
            GlobalParam::AUTOAIM_MAX_FRAME_REFRESHES =
                root["debug"]["latest_frame_max_refreshes"].asInt();
        }
        if (root.isMember("ballistics")) {
            const Value& ballistics = root["ballistics"];
            GlobalParam::BALLISTIC_DRAG_COEFF =
                ballistics.get("drag_coeff", GlobalParam::BALLISTIC_DRAG_COEFF).asDouble();
            GlobalParam::BALLISTIC_GRAVITY =
                ballistics.get("gravity", GlobalParam::BALLISTIC_GRAVITY).asDouble();
            GlobalParam::BALLISTIC_AIR_DENSITY =
                ballistics.get("air_density", GlobalParam::BALLISTIC_AIR_DENSITY).asDouble();
            GlobalParam::BALLISTIC_BULLET_MASS =
                ballistics.get("bullet_mass", GlobalParam::BALLISTIC_BULLET_MASS).asDouble();
            GlobalParam::BALLISTIC_BULLET_RADIUS =
                ballistics.get("bullet_radius", GlobalParam::BALLISTIC_BULLET_RADIUS).asDouble();
            GlobalParam::BALLISTIC_MUZZLE_OFFSET =
                ballistics.get("muzzle_offset", GlobalParam::BALLISTIC_MUZZLE_OFFSET).asDouble();
        }

        AntiBaseParam::udp_debug_enabled = root["anti_base"]["udp_debug_enabled"].asBool();
        AntiBaseParam::udp_debug_host = root["anti_base"]["udp_debug_host"].asString();
        AntiBaseParam::udp_debug_port = root["anti_base"]["udp_debug_port"].asInt();
        AntiBaseParam::output_fps = root["anti_base"]["output_fps"].asInt();
        AntiBaseParam::stream_pre_encode_frame = root["anti_base"].get(
            "stream_pre_encode_frame", AntiBaseParam::stream_pre_encode_frame).asBool();
        AntiBaseParam::packet_size = root["anti_base"]["packet_size"].asInt();
        AntiBaseParam::max_tx_delay_s = root["anti_base"]["max_tx_delay_s"].asDouble();
        AntiBaseParam::limiter_min_packet_gap_ms =
            root["anti_base"].get("limiter_min_packet_gap_ms", AntiBaseParam::limiter_min_packet_gap_ms).asDouble();
        AntiBaseParam::adaptive_bitrate_enabled =
            root["anti_base"].get("adaptive_bitrate_enabled", AntiBaseParam::adaptive_bitrate_enabled).asBool();
        AntiBaseParam::adaptive_bitrate_min_kbps =
            root["anti_base"].get("adaptive_bitrate_min_kbps", AntiBaseParam::adaptive_bitrate_min_kbps).asInt();
        AntiBaseParam::adaptive_bitrate_max_kbps =
            root["anti_base"].get("adaptive_bitrate_max_kbps", AntiBaseParam::adaptive_bitrate_max_kbps).asInt();
        AntiBaseParam::adaptive_bitrate_step_up_kbps =
            root["anti_base"].get("adaptive_bitrate_step_up_kbps", AntiBaseParam::adaptive_bitrate_step_up_kbps).asInt();
        AntiBaseParam::adaptive_bitrate_step_down_kbps =
            root["anti_base"].get("adaptive_bitrate_step_down_kbps", AntiBaseParam::adaptive_bitrate_step_down_kbps).asInt();
        AntiBaseParam::adaptive_bitrate_control_interval_s =
            root["anti_base"].get("adaptive_bitrate_control_interval_s", AntiBaseParam::adaptive_bitrate_control_interval_s).asDouble();
        AntiBaseParam::log_packet_content = root["anti_base"].get("log_packet_content", true).asBool();

        AntiBaseParam::crop_size = root["anti_base"]["preprocess"]["crop_size"].asInt();
        AntiBaseParam::crop_offset_x =
            root["anti_base"]["preprocess"].get("crop_offset_x", AntiBaseParam::crop_offset_x).asInt();
        AntiBaseParam::output_size = root["anti_base"]["preprocess"]["output_size"].asInt();
        AntiBaseParam::static_simplify = root["anti_base"]["preprocess"]["static_simplify"].asBool();
        AntiBaseParam::motion_threshold = root["anti_base"]["preprocess"]["motion_threshold"].asInt();
        AntiBaseParam::motion_erode_px = root["anti_base"]["preprocess"]["motion_erode_px"].asInt();
        AntiBaseParam::motion_dilate_px = root["anti_base"]["preprocess"]["motion_dilate_px"].asInt();
        AntiBaseParam::motion_trail_frames = root["anti_base"]["preprocess"]["motion_trail_frames"].asInt();
        AntiBaseParam::trail_brightness_gain = root["anti_base"]["preprocess"].get(
            "trail_brightness_gain", AntiBaseParam::trail_brightness_gain).asDouble();
        AntiBaseParam::trail_disable_motion_ratio =
            root["anti_base"]["preprocess"]["trail_disable_motion_ratio"].asDouble();
        AntiBaseParam::trail_resume_motion_ratio =
            root["anti_base"]["preprocess"].get(
                "trail_resume_motion_ratio", AntiBaseParam::trail_resume_motion_ratio).asDouble();
        AntiBaseParam::motion_ratio_ema_alpha =
            root["anti_base"]["preprocess"].get(
                "motion_ratio_ema_alpha", AntiBaseParam::motion_ratio_ema_alpha).asDouble();
        AntiBaseParam::bg_update_alpha = root["anti_base"]["preprocess"]["bg_update_alpha"].asDouble();
        AntiBaseParam::bg_blur_sigma = root["anti_base"]["preprocess"]["bg_blur_sigma"].asDouble();
        AntiBaseParam::center_clear_size = root["anti_base"]["preprocess"]["center_clear_size"].asInt();
        AntiBaseParam::force_monochrome = root["anti_base"]["preprocess"]["force_monochrome"].asBool();
        AntiBaseParam::target_bitrate_kbps = root["anti_base"]["preprocess"]["target_bitrate_kbps"].asInt();

        if (root["anti_base"].isMember("h264")) {
            const Value& h264 = root["anti_base"]["h264"];
            AntiBaseParam::h264_speed_preset =
                h264.get("speed_preset", AntiBaseParam::h264_speed_preset).asInt();
            AntiBaseParam::h264_tune =
                h264.get("tune", AntiBaseParam::h264_tune).asInt();
            AntiBaseParam::h264_key_int_max =
                h264.get("key_int_max", AntiBaseParam::h264_key_int_max).asInt();
            AntiBaseParam::h264_bframes =
                h264.get("bframes", AntiBaseParam::h264_bframes).asInt();
            AntiBaseParam::h264_rc_lookahead =
                h264.get("rc_lookahead", AntiBaseParam::h264_rc_lookahead).asInt();
            AntiBaseParam::h264_sync_lookahead =
                h264.get("sync_lookahead", AntiBaseParam::h264_sync_lookahead).asInt();
            AntiBaseParam::h264_sliced_threads =
                h264.get("sliced_threads", AntiBaseParam::h264_sliced_threads).asBool();
            AntiBaseParam::h264_ref =
                h264.get("ref", AntiBaseParam::h264_ref).asInt();
            AntiBaseParam::h264_vbv_buf_capacity =
                h264.get("vbv_buf_capacity", AntiBaseParam::h264_vbv_buf_capacity).asInt();
            AntiBaseParam::h264_option_string =
                h264.get("option_string", AntiBaseParam::h264_option_string).asString();
        }

        if (root.isMember("predictor")) {
            const Value& predictor = root["predictor"];
            PredictorParam::init_radius =
                predictor.get("init_radius", PredictorParam::init_radius).asDouble();
            PredictorParam::max_lost_count =
                predictor.get("max_lost_count", PredictorParam::max_lost_count).asInt();
            PredictorParam::min_consecutive_detections =
                predictor.get("min_consecutive_detections", PredictorParam::min_consecutive_detections).asInt();
            PredictorParam::low_speed_process_noise_xy =
                predictor.get("low_speed_process_noise_xy", PredictorParam::low_speed_process_noise_xy).asDouble();
            PredictorParam::low_speed_process_noise_z =
                predictor.get("low_speed_process_noise_z", PredictorParam::low_speed_process_noise_z).asDouble();
            PredictorParam::low_speed_process_noise_yaw =
                predictor.get("low_speed_process_noise_yaw", PredictorParam::low_speed_process_noise_yaw).asDouble();
            PredictorParam::middle_speed_process_noise_xy =
                predictor.get("middle_speed_process_noise_xy", PredictorParam::middle_speed_process_noise_xy).asDouble();
            PredictorParam::middle_speed_process_noise_z =
                predictor.get("middle_speed_process_noise_z", PredictorParam::middle_speed_process_noise_z).asDouble();
            PredictorParam::middle_speed_process_noise_yaw =
                predictor.get("middle_speed_process_noise_yaw", PredictorParam::middle_speed_process_noise_yaw).asDouble();
            PredictorParam::high_speed_process_noise_xy =
                predictor.get("high_speed_process_noise_xy", PredictorParam::high_speed_process_noise_xy).asDouble();
            PredictorParam::high_speed_process_noise_z =
                predictor.get("high_speed_process_noise_z", PredictorParam::high_speed_process_noise_z).asDouble();
            PredictorParam::high_speed_process_noise_yaw =
                predictor.get("high_speed_process_noise_yaw", PredictorParam::high_speed_process_noise_yaw).asDouble();
            PredictorParam::middle_speed_angular_velocity_threshold =
                predictor.get("middle_speed_angular_velocity_threshold", PredictorParam::middle_speed_angular_velocity_threshold).asDouble();
            PredictorParam::high_speed_angular_velocity_threshold =
                predictor.get("high_speed_angular_velocity_threshold", PredictorParam::high_speed_angular_velocity_threshold).asDouble();
            PredictorParam::measurement_noise_yaw =
                predictor.get("measurement_noise_yaw", PredictorParam::measurement_noise_yaw).asDouble();
            PredictorParam::measurement_noise_pitch =
                predictor.get("measurement_noise_pitch", PredictorParam::measurement_noise_pitch).asDouble();
            PredictorParam::measurement_noise_distance_base =
                predictor.get("measurement_noise_distance_base", PredictorParam::measurement_noise_distance_base).asDouble();
            PredictorParam::measurement_noise_armor_yaw_base =
                predictor.get("measurement_noise_armor_yaw_base", PredictorParam::measurement_noise_armor_yaw_base).asDouble();
            PredictorParam::min_valid_radius =
                predictor.get("min_valid_radius", PredictorParam::min_valid_radius).asDouble();
            PredictorParam::max_valid_radius =
                predictor.get("max_valid_radius", PredictorParam::max_valid_radius).asDouble();
            LOG(INFO) << "Predictor parameters loaded from config file";
        } else {
            LOG(WARNING) << "Predictor parameters not found in config, using defaults";
        }

        json_file.close();
    }
}

