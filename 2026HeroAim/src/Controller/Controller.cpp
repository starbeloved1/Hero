#include "Controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "Log.hpp"
#include "Params.hpp"

using namespace controller;
using namespace lyutils;

auto controller::createController() -> std::shared_ptr<controller::Controller>
{
    return std::make_unique<controller::Controller>();
}

// 实现无参构造函数
Controller::Controller()
{
    // 从静态类读取参数
    this->mouse_require = ControllerParam::mouse_require;
    this->pic_camera_x = ControllerParam::pic_camera_x;
    this->pic_camera_y = ControllerParam::pic_camera_y;
    this->bullet_speed = GlobalParam::SHOOT_SPEED;
    
    // 开火判据参数
    this->armor_fire_window = ControllerParam::armor_fire_window;
    this->speed_fire_window = ControllerParam::speed_fire_window;
    this->phase_confirm_frames = ControllerParam::phase_confirm_frames;
    this->speed_confirm_frames = ControllerParam::speed_confirm_frames;

    // 加速度判断参数
    this->accel_threshold = ControllerParam::accel_threshold;
    this->accel_stable_threshold = ControllerParam::accel_stable_threshold;
    this->accel_stable_frames = ControllerParam::accel_stable_frames;

    // 系统延迟：与 AntiTop 的 tmp_time 含义一致（链路延迟：串口、拨弹盘、机械响应）
    this->system_delay = OutpostParam::tmp_time / 1000.0;

    LOG(INFO) << "[Controller] Init: bullet_speed=" << bullet_speed
              << " system_delay=" << system_delay << "s";
}

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

void Controller::setCurrentObservations(const tracker::TrackResults& trackResults)
{
    current_obs_xyz.clear();
    for (const auto& tr : trackResults)
    {
        // key = car_id * 10 + armor_id，存储Solver解算的完整xyz
        int key = tr.car_id * 10 + tr.armor_id;
        XYZ xyz = tr.location.xyz_imu;  // 隐式转换 XYZIMUProxy -> XYZ
        current_obs_xyz[key] = xyz;
    }
}

static bool thetaInRange(double theta_deg, double range_deg)
{
    theta_deg = std::remainder(theta_deg, 360.0);
    if(std::abs(theta_deg) < range_deg)
        return true;
    else
        return false;
} 

bool Controller::judgeAimNew(bool request)
{
    aim_new = false;
    if(((!aiming) && request) || (aiming && (!request)))
    {
        accumulate_aim_request++;
    }
    else
    {
        accumulate_aim_request = 0;
    }
    if(accumulate_aim_request > waitFrame)
    {
        accumulate_aim_request = 0;
        aiming = request;
        if(aiming)
            aim_new = true;
    }
    return aim_new;
}

ControlResult Controller::control(const SerialPortData& imu_data)
{
    const bool right_pressed = imu_data.right_clicked && !last_right_clicked;
    last_right_clicked = imu_data.right_clicked;

    if(right_pressed)
    {
        LOG(INFO) << "[Ctrl] Manual target reset by right click";
        aim_armor_id = std::make_pair(-1, -1);
        phase_ready_count = 0;
        flyTime = std::chrono::duration<double>(0.0);
        first_shoot = true;
        high_accel_mode = false;
        accel_stable_count = 0;
        aiming = false;
        aim_new = false;
        accumulate_aim_request = 0;
    }

    ControlResult result;
    // ======= 初始化 result：使用当前 IMU 值，后续 Stage 5 会更新为自瞄计算值 =======
    const double current_yaw_deg = imu_data.yaw;
    const double current_pitch_deg = imu_data.pitch;
    result.yaw_setpoint = current_yaw_deg;
    result.pitch_setpoint = current_pitch_deg;
    result.pitch_actual_want = current_pitch_deg;
    result.yaw_actual_want = current_yaw_deg;
    result.shoot_flag = false;
    
    auto now = Time::TimeStamp::now();

    // =========== Stage 1: 粗略预测 -> 选目标车 ===========
    Predictions predictions = predictFunc(now + flyTime);
    if (predictions.empty())
    {
        LOG(WARNING) << "[Ctrl] No prediction";
        return result;
    }

    // 匹配上一帧车辆，找不到则选最近
    auto car_it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& p) { return p.id == aim_armor_id.first; });

    if (car_it == predictions.end())
    {
        double min_dist_sq = std::numeric_limits<double>::max();
        for (auto it = predictions.begin(); it != predictions.end(); ++it)
        {
            double d2 = it->center.x * it->center.x + it->center.y * it->center.y;
            if (d2 < min_dist_sq)
            {
                min_dist_sq = d2;
                car_it = it;
            }
        }
        if (car_it == predictions.end())
        {
            LOG(WARNING) << "[Ctrl] Stage1: no car found in " << predictions.size() << " predictions";
            return result;
        }
        // 切换车辆
        if (aim_armor_id.first != -1 && aim_armor_id.first != car_it->id)
        {
            LOG(INFO) << "[Ctrl] Car switched from " << aim_armor_id.first << " to " << car_it->id;
        }
        aim_armor_id.first = car_it->id;
    }

    // =========== Stage 2: 精确预测（飞行时间 + 系统延迟） ==========
    double horiz_dist = std::sqrt(car_it->center.x * car_it->center.x +
                                   car_it->center.y * car_it->center.y);
    if (horiz_dist > 0.1)
    {
        flyTime = std::chrono::duration<double>(horiz_dist / bullet_speed + system_delay);
        predictions = predictFunc(now + flyTime);
        car_it = std::find_if(predictions.begin(), predictions.end(),
            [&](const auto& p) { return p.id == aim_armor_id.first; });
        if (car_it == predictions.end())
        {
            LOG(WARNING) << "[Ctrl] Stage2: car " << aim_armor_id.first << " lost after flyTime=" << flyTime.count() << "s, predictions=" << predictions.size();
            return result;
        }
    }

    // =========== Stage 3: 找法向量最接近朝向我们的装甲板 ===========
    double best_armor_angle = std::numeric_limits<double>::max();
    int best_armor_id = -1;

    // 从车辆中心指向我们的角度
    double see_angle = std::atan2(car_it->center.y, car_it->center.x);
    
    for (const auto& armor : car_it->armors)
    {
        if (armor.status != Armor::AVAILABLE)
            continue;
        // 装甲板法向量与“指向我们”方向的夹角（0°表示正对我们）
        double armor_angle = std::remainder(armor.theta + M_PI / 2 - see_angle, 2.0 * M_PI);
        if (std::abs(armor_angle) < std::abs(best_armor_angle))
        {
            best_armor_angle = armor_angle;
            best_armor_id = armor.id;
        }
    }
    if (best_armor_id < 0)
    {
        LOG(WARNING) << "[Ctrl] Stage3: no AVAILABLE armor in car " << car_it->id;
        return result;
    }
    aim_armor_id.second = best_armor_id;

    // =========== Stage 4: 用弹道飞行时间再迭代一次 ===========
    {
        const auto& target_armor = car_it->armors[best_armor_id];
        double pitch_tmp = 0.0, yaw_tmp = 0.0, btime_tmp = 0.0;
        // 弹道解算：用车辆中心 xy + 选中装甲板 z（只算飞行时间，不用 yaw）
        if (calcPitchYaw(pitch_tmp, yaw_tmp, btime_tmp, car_it->center.x, car_it->center.y, target_armor.center.z))
        {
            // 用精确飞行时间刷新预测
            flyTime = std::chrono::duration<double>(btime_tmp + system_delay);
            predictions = predictFunc(now + flyTime);
            car_it = std::find_if(predictions.begin(), predictions.end(),
                [&](const auto& p) { return p.id == aim_armor_id.first; });
            if (car_it != predictions.end())
            {
                // 重新找最优装甲板（用局部变量，失败时保留 Stage 3 结果）
                double s4_best_angle = std::numeric_limits<double>::max();
                int s4_best_id = -1;
                double see_angle_s4 = std::atan2(car_it->center.y, car_it->center.x);
                for (const auto& armor : car_it->armors)
                {
                    if (armor.status != Armor::AVAILABLE)
                        continue;
                    double armor_angle = std::remainder(armor.theta + M_PI / 2 - see_angle_s4, 2.0 * M_PI);
                    if (std::abs(armor_angle) < std::abs(s4_best_angle))
                    {
                        s4_best_angle = armor_angle;
                        s4_best_id = armor.id;
                    }
                }
                if (s4_best_id >= 0)
                {
                    best_armor_angle = s4_best_angle;
                    best_armor_id = s4_best_id;
                    aim_armor_id.second = s4_best_id;
                }
            }
            else
            {
                // Stage 4 预测后车辆消失，回退到 Stage 2 的 predictions
                LOG(WARNING) << "[Ctrl] Stage4: car " << aim_armor_id.first << " lost after btime predict, trying fallback";
                predictions = predictFunc(now + flyTime);
                car_it = std::find_if(predictions.begin(), predictions.end(),
                    [&](const auto& p) { return p.id == aim_armor_id.first; });
                if (car_it == predictions.end())
                {
                    LOG(WARNING) << "[Ctrl] Stage4: fallback failed, predictions=" << predictions.size();
                    return result;
                }
            }
        }
    }

    // =========== Stage 4.5: 用刷新后的装甲板坐标最终计算pitch ===========
    // 直接使用EKF预测的xyz坐标
    double pitch = 0.0, yaw = 0.0, btime = 0.0;
    double center_yaw_rad = 0.0;  // 车辆中心 yaw 角（弧度）
    bool accel_block_fire = false;  // 加速度过大时禁止开火
    bool use_observation_aim = false;  // 是否使用观测装甲板瞄准（而非预测车辆中心）
    {
        const auto& final_armor = car_it->armors[best_armor_id];
        int car_id = aim_armor_id.first;
        
        // 使用EKF预测的xyz（默认）
        double target_x = final_armor.center.x;
        double target_y = final_armor.center.y;
        double target_z = final_armor.center.z;
        
        // 车辆中心xyz（用于yaw计算和日志）
        double center_x = car_it->center.x;
        double center_y = car_it->center.y;
        double center_z = car_it->center.z;
        
        // =========== Stage 4.6: 加速度判断 ===========
        // 计算当前加速度大小
        double accel_magnitude = std::sqrt(car_it->ax * car_it->ax + car_it->ay * car_it->ay);
        
        // 判断是否进入/退出高加速度模式
        if (accel_magnitude > accel_threshold) {
            // 加速度超过阈值，进入高加速度模式
            high_accel_mode = true;
            accel_stable_count = 0;
            LOG(WARNING) << "[Ctrl] HIGH_ACCEL detected: " << accel_magnitude << " m/s² > " << accel_threshold;
        } else if (high_accel_mode) {
            // 已在高加速度模式，检查是否稳定
            if (accel_magnitude < accel_stable_threshold) {
                accel_stable_count++;
                if (accel_stable_count >= accel_stable_frames) {
                    // 连续多帧加速度稳定，退出高加速度模式
                    high_accel_mode = false;
                    accel_stable_count = 0;
                    LOG(INFO) << "[Ctrl] Accel stabilized, resuming prediction mode";
                }
            } else {
                accel_stable_count = 0;
            }
        }
        
        // 高加速度模式：使用观测坐标而非预测坐标（现在不会进高加速度模式）
        if (high_accel_mode) {
            int obs_key = car_id * 10 + best_armor_id;
            auto obs_it = current_obs_xyz.find(obs_key);
            if (obs_it != current_obs_xyz.end()) {
                // 使用当前帧观测坐标
                target_x = obs_it->second.x;
                target_y = obs_it->second.y;
                target_z = obs_it->second.z;
                LOG(INFO) << "[Ctrl] HIGH_ACCEL mode: using observation ("
                          << target_x << "," << target_y << "," << target_z << ")";
            } else {
                LOG(WARNING) << "[Ctrl] HIGH_ACCEL mode: no observation for key=" << obs_key;
            }
            accel_block_fire = true;  // 禁止开火
            use_observation_aim = true;
        }
        
        LOG(INFO) << "Target: car=" << car_id << " armor=" << best_armor_id
                  << " armor_xyz=(" << target_x << "," << target_y << "," << target_z << ")"
                  << " a=" << accel_magnitude << "m/s²";
        
        double ekf_omega = car_it->omega / (2.0 * M_PI);  // EKF的omega(rps)
        LOG(INFO) << " Center：xyz=(" << center_x << "," << center_y << "," << center_z << ")"
                  << " EKF=" << ekf_omega << "rps"
                  << " r1=" << car_it->r1 << " r2=" << car_it->r2;
        
        // 用车辆中心 xy 计算 yaw 角
        center_yaw_rad = std::atan2(center_y, center_x);
        
        if (!calcPitchYaw(pitch, yaw, btime, target_x, target_y, target_z))
        {
            LOG(WARNING) << "[Ctrl] Stage4.5: calcPitchYaw failed";
            return result;
        }
    }

    // =========== Stage 5: 输出 ===========
    double center_yaw_deg = center_yaw_rad * 180.0 / M_PI;
    center_yaw_deg = current_yaw_deg + std::remainder(center_yaw_deg - current_yaw_deg, 360.0);
    double pitch_deg = pitch * 180.0 / M_PI;
    
    result.yaw_setpoint = center_yaw_deg;
    result.yaw_actual_want = center_yaw_deg;
    result.pitch_setpoint = pitch_deg;  
    result.pitch_actual_want = pitch_deg;

    // =========== Stage 6: 开火判据（自适应参数） ==========
    // 根据目标车辆转速omega动态调整开火窗口和确认帧数
    const double abs_omega = std::abs(car_it->omega);
    const double omega_threshold = 4.0;  // rad/s，高/低速分界
    double dynamic_fire_window;
    int dynamic_confirm_frames;
    
    if (abs_omega > omega_threshold) {
        // 高速模式：放宽窗口，减少确认帧（装甲板正对时间窗口短）
        dynamic_fire_window = speed_fire_window * M_PI / 180.0;  
        dynamic_confirm_frames = speed_confirm_frames;
    } else {
        // 低速/静止模式：使用默认严格参数
        dynamic_fire_window = armor_fire_window * M_PI / 180.0;  // 默认10°
        dynamic_confirm_frames = phase_confirm_frames;  // 默认5帧
    }

    // 条件1: 装甲板法向量朝向角度在开火窗口内
    // 条件2: 连续对准若干帧
    const bool armor_aligned = std::abs(best_armor_angle) < dynamic_fire_window;

    // 所有条件都满足时才累计计数器
    if (armor_aligned)
        phase_ready_count++;
    else
        phase_ready_count = 0;

    // 开火条件：对准 + 确认帧数达标 + 非高加速度模式；不再依赖右键持续按住
    if (armor_aligned && phase_ready_count >= dynamic_confirm_frames && !accel_block_fire)
    {
        result.shoot_flag = true;
        last_shoot_time = now;
        first_shoot = false;
        phase_ready_count = 0;
        LOG(WARNING) << "           SHOOT!!!!!!!!";
    }

    // 输出选中目标信息供可视化使用
    result.target_car_id = aim_armor_id.first;
    result.target_armor_id = best_armor_id;
    result.flyTime_s = static_cast<float>(flyTime.count());
    
    // 填充可视化数据
    result.vx = static_cast<float>(car_it->vx);
    result.vy = static_cast<float>(car_it->vy);
    result.ax = static_cast<float>(car_it->ax);
    result.ay = static_cast<float>(car_it->ay);
    result.armor_angle_deg = static_cast<float>(best_armor_angle * 180.0 / M_PI);
    result.phase_ready_cnt = phase_ready_count;
    result.phase_confirm_frames = dynamic_confirm_frames;
    result.accel_threshold = static_cast<float>(accel_threshold);
    result.high_accel_mode = high_accel_mode;

    LOG(INFO) << "[Ctrl] armor_angle=" << (best_armor_angle * 180.0 / M_PI)
              << "° aligned=" << armor_aligned
              << " cnt=" << phase_ready_count << "/" << dynamic_confirm_frames
              << " fire_win=" << (dynamic_fire_window * 180.0 / M_PI) << "°"
              << " shoot=" << (int)result.shoot_flag
              << " accel_block=" << accel_block_fire;
    
    LOG(INFO) << "                                 send yaw: " << result.yaw_setpoint
              << "  send pitch: " << result.pitch_setpoint;
    LOG(INFO) << "                                 recv yaw: " << imu_data.yaw
              << "  recv pitch: " << imu_data.pitch;

    return result;
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time,
                              double target_x, double target_y, double target_z){
    // FLU坐标系：target_x=前，target_y=左，target_z=上
    const double dz = target_z;
    const double distance = sqrt(target_x * target_x + target_y * target_y);
    if (distance < 0.01) {
        LOG(WARNING) << "calcPitchYaw: target too close, dist=" << distance;
        return false;
    }
    // K = 0.5 * Cd * Rho * A / m，截面积 A = π*r²
    // 空气阻力系数：42mm弹丸实测值，偏高则减小，偏低则增大
    const double drag_coeff = GlobalParam::BALLISTIC_DRAG_COEFF;
    const double air_density = GlobalParam::BALLISTIC_AIR_DENSITY;
    const double bullet_radius = GlobalParam::BALLISTIC_BULLET_RADIUS;
    const double bullet_mass = GlobalParam::BALLISTIC_BULLET_MASS;
    const double gravity = GlobalParam::BALLISTIC_GRAVITY;
    const double muzzle_offset = GlobalParam::BALLISTIC_MUZZLE_OFFSET;
    double k1 = drag_coeff * air_density *
                (M_PI * bullet_radius * bullet_radius) / 2.0 / bullet_mass;
    double theta = atan(dz / distance);
    double delta_z = 0.0;
    // 枪口在pitch轴x方向前移0.3m，需修正弹道起点
    for (int i = 0; i < max_iter; i++)
    {
        // 每步根据当前仰角theta修正枪口到目标的水平距离和高度差
        double h_muzzle = distance - muzzle_offset * cos(theta);
        double dz_muzzle = dz - muzzle_offset * sin(theta);
        double t = (exp(k1 * h_muzzle) - 1) / (k1 * bullet_speed * cos(theta));
        // 考虑空气阻力的水平飞行时间
        // 标准弹道方程：z = v0*sin(theta)*t - 0.5*g*t^2
        double calc_z = bullet_speed * sin(theta) * t - 0.5 * gravity * t * t;
        delta_z = dz_muzzle - calc_z; 
        if (fabs(delta_z) < tol)
        {
            time = t;
            break;
        }
        // 牛顿迭代导数：df/dtheta = v0*cos(theta)*t + (v0*sin(theta) - g*t) * t * tan(theta)
        double dt_dtheta = t * tan(theta);
        double df_dtheta = bullet_speed * cos(theta) * t 
                         + (bullet_speed * sin(theta) - gravity * t) * dt_dtheta;
        theta += delta_z / df_dtheta;  // 注意符号：delta_z = target - calc, 需要增大theta
    }

    if(fabs(delta_z) > tol)
    {
        LOG(WARNING) << "calcPitchYaw failed to converge, delta_z=" << delta_z;
        return false;
    }
    else
    {
        pitch = theta;
        // FLU坐标系：target_x=前，target_y=左，target_z=上
        target_x = (target_x == 0 ? 1e-6 : target_x);
        yaw = atan2(target_y, target_x);
        return true;
    }
} 
