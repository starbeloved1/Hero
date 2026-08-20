#include "Estimator/include/AntiTop.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"
#include <numeric>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>
#include <map>

namespace estimator {

namespace {
constexpr double NO_PITCH_CONTROL_SENTINEL = 9999.0;
}

// ================================================================
//  构造 & 状态重置
// ================================================================

AntiTop::AntiTop(solver::Solver* solver) : solver_(solver) {}

void AntiTop::resetCalibrationState(bool reset_direction) {
    GlobalParam::IS_CALIBRATED = false;
    GlobalParam::Z_MAP.clear();
    GlobalParam::ANTITOP_TRACKING_Z_VALID = false;
    GlobalParam::ANTITOP_TARGET_Z = 0.0;
    GlobalParam::ANTITOP_TARGET_PITCH = 0.0;
    GlobalParam::ANTITOP_TARGET_VALID = false;
    GlobalParam::ANTITOP_LAST_ZONE_INDEX = -1;
    center_xy_window.clear();
    calib_z_samples.clear();
    recent_z_buffer.clear();
    target_aim_z = 0.0;
    last_zone_index = -1;
    shootCenter.z = 0.0;
    if (reset_direction) {
        Direction = -1;
        SerialParam::direction = Direction;
        x_diff_history.clear();
    }
}

void AntiTop::resetRuntimeState(bool reset_direction) {
    resetCalibrationState(reset_direction);
    zone_period_history.clear();
    avg_ms = -1;
    has_last_enter_time = false;
    in_shoot_zone = false;
    pitch_state = PitchLockState::FITTING;
    fire_delay_ms = 0;
    shootCenter = Point3d(0, 0, 0);
}

void AntiTop::updateZCalibration() {
    if (GlobalParam::IS_CALIBRATED)
        return;

    if (center_xy_window.size() < Z_CALIB_START_CENTER_SAMPLES)
        return;

    if (!std::isfinite(tracking_armor.z)) {
        LOG(WARNING) << "CALIB: skip invalid z=" << tracking_armor.z;
        return;
    }

    calib_z_samples.push_back(tracking_armor.z);
    if (calib_z_samples.size() > Z_CALIB_MAX_SAMPLES)
        calib_z_samples.erase(calib_z_samples.begin());

    if (calib_z_samples.size() < Z_CALIB_MIN_SAMPLES) {
        LOG(INFO) << "CALIB: collecting z samples"
                  << " count=" << calib_z_samples.size()
                  << "/" << Z_CALIB_MIN_SAMPLES
                  << " z=" << tracking_armor.z;
        return;
    }

    tryBuildZMapFromSamples();
}

bool AntiTop::tryBuildZMapFromSamples() {
    if (GlobalParam::IS_CALIBRATED ||
        calib_z_samples.size() < Z_CALIB_MIN_SAMPLES)
        return false;

    std::map<int, size_t> histogram;
    for (double z : calib_z_samples) {
        if (!std::isfinite(z))
            continue;
        const int bin = static_cast<int>(std::floor(z / Z_HIST_BIN_SIZE));
        ++histogram[bin];
    }

    struct Peak {
        int bin;
        size_t count;
    };

    std::vector<Peak> peaks;
    for (const auto& kv : histogram)
        peaks.push_back({kv.first, kv.second});

    std::sort(peaks.begin(), peaks.end(),
              [](const Peak& a, const Peak& b) {
                  if (a.count != b.count)
                      return a.count > b.count;
                  return a.bin < b.bin;
              });
    if (peaks.size() > 40)
        peaks.resize(40);

    std::vector<double> selected_peaks;
    size_t best_score = 0;
    for (size_t i = 0; i < peaks.size(); ++i) {
        for (size_t j = i + 1; j < peaks.size(); ++j) {
            for (size_t k = j + 1; k < peaks.size(); ++k) {
                std::vector<double> candidate = {
                    (static_cast<double>(peaks[i].bin) + 0.5) * Z_HIST_BIN_SIZE,
                    (static_cast<double>(peaks[j].bin) + 0.5) * Z_HIST_BIN_SIZE,
                    (static_cast<double>(peaks[k].bin) + 0.5) * Z_HIST_BIN_SIZE
                };
                std::sort(candidate.begin(), candidate.end());

                const double gap0 = candidate[1] - candidate[0];
                const double gap1 = candidate[2] - candidate[1];
                if (gap0 < Z_LAYER_GAP_MIN || gap0 > Z_LAYER_GAP_MAX ||
                    gap1 < Z_LAYER_GAP_MIN || gap1 > Z_LAYER_GAP_MAX)
                    continue;

                const size_t score = peaks[i].count + peaks[j].count + peaks[k].count;
                if (score > best_score) {
                    best_score = score;
                    selected_peaks = candidate;
                }
            }
        }
    }

    if (selected_peaks.size() < 3) {
        LOG(INFO) << "CALIB: waiting for 3 z peaks"
                  << " samples=" << calib_z_samples.size()
                  << " peaks=" << selected_peaks.size();
        return false;
    }

    std::sort(selected_peaks.begin(), selected_peaks.end());
    for (size_t i = 1; i < selected_peaks.size(); ++i) {
        const double gap = selected_peaks[i] - selected_peaks[i - 1];
        if (gap < Z_LAYER_GAP_MIN || gap > Z_LAYER_GAP_MAX) {
            LOG(WARNING) << "CALIB: reject peak gap"
                         << " prev=" << selected_peaks[i - 1]
                         << " next=" << selected_peaks[i]
                         << " gap=" << gap
                         << " min=" << Z_LAYER_GAP_MIN
                         << " max=" << Z_LAYER_GAP_MAX;
            return false;
        }
    }

    std::vector<std::vector<double>> layers(3);
    for (double z : calib_z_samples) {
        int best_idx = -1;
        double best_diff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < selected_peaks.size(); ++i) {
            const double diff = std::abs(z - selected_peaks[i]);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = static_cast<int>(i);
            }
        }
        if (best_idx >= 0 && best_diff <= Z_CALIB_MATCH_THRESHOLD)
            layers[best_idx].push_back(z);
    }

    std::vector<double> zmap;
    for (size_t i = 0; i < layers.size(); ++i) {
        if (layers[i].size() < Z_LAYER_MIN_SAMPLES) {
            LOG(WARNING) << "CALIB: reject weak layer"
                         << " idx=" << i
                         << " count=" << layers[i].size()
                         << " min_count=" << Z_LAYER_MIN_SAMPLES;
            return false;
        }

        std::sort(layers[i].begin(), layers[i].end());
        zmap.push_back(layers[i][layers[i].size() / 2]);
    }

    std::sort(zmap.begin(), zmap.end());
    for (size_t i = 1; i < zmap.size(); ++i) {
        const double gap = zmap[i] - zmap[i - 1];
        if (gap < Z_LAYER_GAP_MIN || gap > Z_LAYER_GAP_MAX) {
            LOG(WARNING) << "CALIB: reject zmap gap"
                         << " prev=" << zmap[i - 1]
                         << " next=" << zmap[i]
                         << " gap=" << gap
                         << " min=" << Z_LAYER_GAP_MIN
                         << " max=" << Z_LAYER_GAP_MAX;
            return false;
        }
    }

    GlobalParam::Z_MAP = zmap;
    GlobalParam::IS_CALIBRATED = true;
    target_aim_z = zmap[LOWEST_Z_INDEX];
    shootCenter.z = target_aim_z;
    pitch_state = PitchLockState::LOCKED_AIM;

    LOG(WARNING) << "CALIBRATION COMPLETE!"
                 << " z0=" << zmap[0]
                 << " z1=" << zmap[1]
                 << " z2=" << zmap[2]
                 << " samples=" << calib_z_samples.size()
                 << " layer_counts="
                 << layers[0].size() << "/"
                 << layers[1].size() << "/"
                 << layers[2].size();
    calib_z_samples.clear();
    return true;
}

// ================================================================
//  主入口
// ================================================================

bool AntiTop::startAim(vector<Armor>& armors, const SerialPortData& imu_data,
                        SerialPort* serial_port, bool is_init)
{
    GlobalParam::ANTITOP_SHOOT_FLAG = false;
    GlobalParam::ANTITOP_TARGET_VALID = false;

    if (is_init) {
        LOG(INFO) << "----------INIT_ANTITOP----------";
        resetRuntimeState(true);
    }

    bool result = false;
    if (!armors.empty()) {
        try {
            result = processFrame(armors, imu_data, serial_port);
        } catch (const std::exception& e) {
            LOG(ERROR) << "AntiTop error: " << e.what();
        }
    }

    // 同步可视化状态
    GlobalParam::ANTITOP_IN_SHOOT_ZONE = in_shoot_zone;
    GlobalParam::ANTITOP_PITCH_STATE = static_cast<int>(pitch_state);
    GlobalParam::ANTITOP_AVG_MS = avg_ms;
    GlobalParam::ANTITOP_SHOOT_FLAG = result && (GlobalParam::ANTITOP_SHOOT_FLAG ||
                                                 SerialParam::send_data.shootStatus == 1);
    return result;
}

void AntiTop::judgeFireCondition(const SerialPortData& imu_data) {
    last_right_clicked = imu_data.right_clicked;
}

// ================================================================
//  帧处理流水线（线性流程，每一步依赖上一步的结果）
//
//   selectTarget -> updateTracking -> updateZoneState -> executeControl
//        选目标        跟踪+方向+中心        zone进出+标定      状态机+串口
// ================================================================

bool AntiTop::processFrame(vector<Armor>& armors, const SerialPortData& imu_data,
                            SerialPort* serial_port)
{
    if (!selectTarget(armors)) {
        LOG(WARNING) << "Exit processFrame: selectTarget failed";
        return false;
    }

    if (!updateTracking()) {
        LOG(INFO) << "SkipCenter: processFrame exit because updateTracking returned false";
        return false;
    }

    updateZoneState();

    LOG(WARNING) << "Omega: " << avg_ms;

    executeControl(imu_data, serial_port);

    return true;
}

// ================================================================
//  目标选择
// ================================================================

bool AntiTop::selectTarget(std::vector<Armor>& armors) {
    if (armors.empty()) {
        GlobalParam::ANTITOP_TRACKING_Z_VALID = false;
        LOG(WARNING) << "Exit selectTarget: empty armors";
        return false;
    }

    // 严格选择前哨战装甲板（class=7），world 坐标已在 PreProcessArmor 中解算
    std::vector<Armor> candidates;
    for (const auto& a : armors) {
        if (a._class == 7)
            candidates.push_back(a);
    }

    if (candidates.empty()) {
        GlobalParam::ANTITOP_TRACKING_Z_VALID = false;
        LOG(WARNING) << "Exit selectTarget: class 7 armor not found, armors_size=" << armors.size();
        return false;
    }

    if (last_armor.is_armor) {
        tracking_armor = *std::min_element(
            candidates.begin(), candidates.end(),
            [&](const Armor& a, const Armor& b) {
                return calcDiff(a, last_armor) < calcDiff(b, last_armor);
            });
    } else {
        tracking_armor = candidates.front();
    }

    SerialParam::send_data.num = tracking_armor._class;

    return true;
}

// ================================================================
//  跟踪维护
// ================================================================

bool AntiTop::updateTracking() {
    bool success = false;

    GlobalParam::ANTITOP_TRACKING_Z = tracking_armor.z;
    GlobalParam::ANTITOP_TRACKING_Z_VALID = true;
    ++GlobalParam::ANTITOP_TRACKING_Z_SEQ;

    const bool has_last = last_armor.is_armor;
    const bool armor_jump = has_last
        && cv::norm(tracking_armor.center - last_armor.center) >= ARMOR_PIXEL_DISTANCE_THRESHOLD;

    if (armor_jump) {
        LOG(WARNING) << "SkipFrame: armor jump pixel_diff="
                     << cv::norm(tracking_armor.center - last_armor.center)
                     << " threshold=" << ARMOR_PIXEL_DISTANCE_THRESHOLD;
        x_diff_history.clear();
        last_armor = tracking_armor;
        GlobalParam::ANTITOP_TRACKING_Z_VALID = false;
        return false;
    } else if (has_last) {
        updateDirection();
    }

    if (std::isfinite(tracking_armor.z)) {
        recent_z_buffer.push_back(tracking_armor.z);
        if (recent_z_buffer.size() > ZONE_Z_MEDIAN_FRAMES)
            recent_z_buffer.erase(recent_z_buffer.begin());
    }

    center_xy_window.emplace_back(static_cast<float>(tracking_armor.x),
                                  static_cast<float>(tracking_armor.y));
    if (center_xy_window.size() > MAX_WINDOW_SIZE)
        center_xy_window.erase(center_xy_window.begin());
    updateZCalibration();

    success = updateRotationCenter();
    last_armor = tracking_armor;
    return success;
}

// ================================================================
//  方向判定
// ================================================================

bool AntiTop::updateDirection() {
    if (Direction != -1) return true;
    if (!tracking_armor.is_armor || !last_armor.is_armor) return false;

    const Point2f cur_proj = solver_->reproject(
        Point3f(tracking_armor.x, tracking_armor.y, tracking_armor.z));
    const Point2f prev_proj = solver_->reproject(
        Point3f(last_armor.x, last_armor.y, last_armor.z));
    const float x_diff = cur_proj.x - prev_proj.x;

    // 动态阈值：历史均值 x10，用于过滤异常跳变
    float threshold = 0.0f;
    if (!x_diff_history.empty()) {
        float sum_abs = std::accumulate(x_diff_history.begin(), x_diff_history.end(), 0.0f,
            [](float acc, float v) { return acc + std::abs(v); });
        threshold = 10.0f * (sum_abs / x_diff_history.size());
    }

    if (x_diff_history.empty() || std::abs(x_diff) <= threshold) {
        x_diff_history.push_back(x_diff);
        if (x_diff_history.size() > MAX_DIRE_SIZE + 1)
            x_diff_history.erase(x_diff_history.begin());
    } else {
        x_diff_history.clear();
    }

    if (x_diff_history.size() == MAX_DIRE_SIZE) {
        const float avg = std::accumulate(x_diff_history.begin(), x_diff_history.end(), 0.0f)
                          / static_cast<float>(MAX_DIRE_SIZE);
        if (avg < -DIRECTION_THRESHOLD)       { Direction = 1; LOG(INFO) << "Direction: CW"; }
        else if (avg > DIRECTION_THRESHOLD)    { Direction = 0; LOG(INFO) << "Direction: CCW"; }
        else                                   { Direction = -1; x_diff_history.clear(); LOG(INFO) << "Direction: UNKNOWN"; }
        SerialParam::direction = Direction;
    }
    return true;
}

// ================================================================
//  旋转中心计算
// ================================================================

bool AntiTop::updateRotationCenter() {
    if (center_xy_window.size() < 10) {
        LOG(INFO) << "SkipCenter: insufficient samples count=" << center_xy_window.size();
        return false;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto& sample : center_xy_window) {
        sum_x += sample.x;
        sum_y += sample.y;
    }

    shootCenter.x = sum_x / static_cast<double>(center_xy_window.size());
    shootCenter.y = sum_y / static_cast<double>(center_xy_window.size());

    SerialParam::shoot_center = solver_->reproject(shootCenter);
    const size_t show_count = std::min(center_xy_window.size(),
                                       static_cast<size_t>(std::numeric_limits<int>::max()));
    GlobalParam::ANTITOP_DATA_COUNT = static_cast<int>(show_count);
    LOG(INFO) << "RotationCenter: xyz=(" << shootCenter.x << ","
              << shootCenter.y << "," << shootCenter.z << ")"
              << " window=" << center_xy_window.size()
              << " armor_xy=(" << tracking_armor.x << "," << tracking_armor.y << ")";
    return true;
}

// ================================================================
//  Z 值标定
// ================================================================


void AntiTop::updateZoneState() {
    // 周期统计只依赖中心窗口；开火仍然必须等待 z 标定完成。
    if (center_xy_window.size() < 80)
        return;

    auto now = std::chrono::steady_clock::now();
    const Point2f center2D = solver_->reproject(shootCenter);
    const Point2f armor2D = solver_->reproject(
        Point3d(tracking_armor.x, tracking_armor.y, tracking_armor.z));

    const float x_dist = std::abs(center2D.x - armor2D.x);
    LOG(INFO) << "Pixel: " << x_dist;

    const bool now_in_zone = (x_dist < ENTER_ZONE_THRESHOLD);
    const bool zone_rising = (now_in_zone && !in_shoot_zone);

    // 周期记录
    if (zone_rising) {
        if (has_last_enter_time) {
            auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_enter_time);
            if (period.count() >= MIN_TIME_INTERVAL && period.count() <= MAX_TIME_INTERVAL) {
                zone_period_history.push_back(period);
                if (zone_period_history.size() > MAX_PERIOD_HISTORY)
                    zone_period_history.erase(zone_period_history.begin());
            } else {
                LOG(WARNING) << "Reset period history: invalid interval="
                             << period.count()
                             << " min=" << MIN_TIME_INTERVAL
                             << " max=" << MAX_TIME_INTERVAL;
                zone_period_history.clear();
                avg_ms = -1;
                has_last_enter_time = false;
            }
        }
        if (!has_last_enter_time) {
            last_enter_time = now;
            has_last_enter_time = true;
        } else {
            last_enter_time = now;
        }
        avg_ms = calculateAvgPeriodMs();
    }

    // 滞回更新 zone 状态
    if (now_in_zone)
        in_shoot_zone = true;
    else if (x_dist > OUT_ZONE_THRESHOLD) {
        in_shoot_zone = false;
    }

    // 标定完成后的 zone 上升沿处理
    if (zone_rising && GlobalParam::IS_CALIBRATED) {
        const bool zone_matched = observeZoneOrder();
        if (zone_matched && pitch_state == PitchLockState::LOCKED_AIM)
            tryInitiateFiring(now);
    }
}

bool AntiTop::observeZoneOrder() {
    if (recent_z_buffer.size() < ZONE_Z_MEDIAN_FRAMES) {
        last_zone_index = -2;
        GlobalParam::ANTITOP_LAST_ZONE_INDEX = last_zone_index;
        LOG(WARNING) << "ZoneOrder: insufficient z median samples"
                     << " count=" << recent_z_buffer.size()
                     << "/" << ZONE_Z_MEDIAN_FRAMES;
        return false;
    }

    const double z = recentZMedian();
    double best = std::numeric_limits<double>::max();
    const int matched = matchZLayer(z, &best);
    if (matched < 0) {
        last_zone_index = -2;
        GlobalParam::ANTITOP_LAST_ZONE_INDEX = last_zone_index;
        LOG(WARNING) << "ZoneOrder: z median match failed"
                     << " z_median=" << z
                     << " best_diff=" << best
                     << " threshold=" << Z_RUNTIME_MATCH_THRESHOLD;
        return false;
    }

    last_zone_index = matched;
    GlobalParam::ANTITOP_LAST_ZONE_INDEX = last_zone_index;
    if (matched == LOWEST_Z_INDEX)
        updateTargetAimZ(z);
    LOG(INFO) << "ZoneOrder: matched"
              << " idx=" << last_zone_index
              << " z_median=" << z
              << " diff=" << best
              << " threshold=" << Z_RUNTIME_MATCH_THRESHOLD;
    return true;
}

double AntiTop::recentZMedian() const {
    if (recent_z_buffer.empty())
        return std::numeric_limits<double>::quiet_NaN();

    std::vector<double> values = recent_z_buffer;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

int AntiTop::matchZLayer(double observed_z, double* out_diff) const {
    double best = std::numeric_limits<double>::max();
    int matched = -1;

    if (!std::isfinite(observed_z) || GlobalParam::Z_MAP.size() != 3) {
        if (out_diff)
            *out_diff = std::numeric_limits<double>::quiet_NaN();
        return -1;
    }

    for (size_t i = 0; i < GlobalParam::Z_MAP.size(); ++i) {
        const double d = std::abs(GlobalParam::Z_MAP[i] - observed_z);
        if (d < best) {
            best = d;
            matched = static_cast<int>(i);
        }
    }

    if (out_diff)
        *out_diff = best;
    return best <= Z_RUNTIME_MATCH_THRESHOLD ? matched : -1;
}

void AntiTop::updateTargetAimZ(double observed_z) {
    if (!std::isfinite(observed_z))
        return;

    target_aim_z = observed_z;
    shootCenter.z = target_aim_z;

    LOG(INFO) << "TargetAimZ: observed=" << observed_z
              << " target=" << target_aim_z;
}

double AntiTop::getTargetAimZ() const {
    if (std::isfinite(target_aim_z) && target_aim_z != 0.0)
        return target_aim_z;

    if (GlobalParam::Z_MAP.size() > LOWEST_Z_INDEX &&
        std::isfinite(GlobalParam::Z_MAP[LOWEST_Z_INDEX]))
        return GlobalParam::Z_MAP[LOWEST_Z_INDEX];

    return tracking_armor.z;
}

int AntiTop::calculateAvgPeriodMs() const {
    const size_t n = zone_period_history.size();
    if (n < 2)
        return -1;

    const size_t count = std::min<size_t>(3, n);
    int sum = 0;
    for (size_t i = 0; i < count; ++i)
        sum += static_cast<int>(zone_period_history[n - 1 - i].count());

    return sum / static_cast<int>(count);
}

// ================================================================
//  射击发起（zone 上升沿 + UNLOCKED 时一次性计算并锁定 pitch）
// ================================================================

void AntiTop::tryInitiateFiring(std::chrono::steady_clock::time_point now) {
    (void)now;

    if (pitch_state != PitchLockState::LOCKED_AIM)
        return;

    if (last_zone_index != LOWEST_Z_INDEX)
        return;

    if (avg_ms <= 0)
        return;

    if (!std::isfinite(getTargetAimZ()))
        return;

    int bias_ms = 0;
    if (Direction == 1) bias_ms = static_cast<int>(time_bias);
    else if (Direction == 0) bias_ms = static_cast<int>(time_bias_inverse);

    const double tz = getTargetAimZ();
    double fly_s = 0.0;
    calculateBallistics(Point3d(shootCenter.x, shootCenter.y, tz), &fly_s);
    if (!std::isfinite(fly_s))
        return;

    const double delay = 3 * avg_ms - tmp_time - fly_s * 1000.0 + bias_ms;
    if (!std::isfinite(delay))
        return;

    fire_delay_ms = std::max(0, static_cast<int>(std::lround(delay)));
    fire_start_time = std::chrono::steady_clock::now();
    pitch_state = PitchLockState::FIRE_COUNTDOWN;

    LOG(WARNING) << "FIRE COUNTDOWN START"
                 << " zone_idx=" << last_zone_index
                 << " target_z=" << tz
                 << " period=" << avg_ms << "ms"
                 << " delay=" << fire_delay_ms << "ms";
}

// ================================================================
//  弹道计算
// ================================================================

Angle_t AntiTop::calculateBallistics(Point3d target, double* out_flyTime) {
    Angle_t result;

    // FLU: X 前，Y 左，Z 上，yaw 正值为左转
    target.x = (target.x == 0 ? 1e-6 : target.x);
    result.yaw = atan2(target.y, target.x);
    const double h_dist_raw = sqrt(target.x * target.x + target.y * target.y);
    double h_dist = h_dist_raw - OutpostParam::shoot_distance_offset;
    h_dist = std::max(h_dist, 1e-6);
    double dz = target.z;
    double theta = atan(dz / h_dist);
    double ft = 0.0;
    const double drag_coeff = GlobalParam::BALLISTIC_DRAG_COEFF;
    const double air_density = GlobalParam::BALLISTIC_AIR_DENSITY;
    const double bullet_radius = GlobalParam::BALLISTIC_BULLET_RADIUS;
    const double bullet_mass = GlobalParam::BALLISTIC_BULLET_MASS;
    const double k1 = drag_coeff * air_density *
                      (M_PI * bullet_radius * bullet_radius) / 2.0 / bullet_mass;
    const double gravity = GlobalParam::BALLISTIC_GRAVITY;
    const double muzzle_offset = GlobalParam::BALLISTIC_MUZZLE_OFFSET;

    for (int i = 0; i < 100; i++) {
        double h_muz = h_dist - muzzle_offset * cos(theta);
        double dz_muz = dz - muzzle_offset * sin(theta);
        ft = (exp(k1 * h_muz) - 1) / (k1 * speed * cos(theta));
        double calc_z = speed * sin(theta) * ft - 0.5 * gravity * ft * ft;
        double delta = dz_muz - calc_z;
        if (fabs(delta) < 1e-6) break;
        double dt_dtheta = ft * tan(theta);
        double df_dtheta = speed * cos(theta) * ft
                         + (speed * sin(theta) - gravity * ft) * dt_dtheta;
        theta += delta / df_dtheta;
    }

    if (out_flyTime) *out_flyTime = ft;
    result.pitch = theta / M_PI * ANGLE_FULL_CIRCLE;
    return result;
}

Angle_t AntiTop::getShootCenter(const SerialPortData& imu_data) {
    (void)imu_data;
    return calculateBallistics(shootCenter);
}

// ================================================================
//  控制输出（状态机 + 串口发送）
//
//  每帧只调用一次 calculateBallistics 计算旋转中心瞄准角：
//  - Yaw:   始终跟踪旋转中心
//  - Pitch: 标定后只使用最低装甲板高度
//  - Shoot: COUNTDOWN 到时置 1
// ================================================================

void AntiTop::executeControl(const SerialPortData& imu_data, SerialPort* serial_port) {
    const double aim_z = GlobalParam::IS_CALIBRATED
        ? getTargetAimZ()
        : tracking_armor.z;

    // yaw 始终跟踪旋转中心 XY；pitch 只使用最低装甲板高度
    const Angle_t aim = calculateBallistics(
        Point3d(shootCenter.x, shootCenter.y, aim_z));
    GlobalParam::ANTITOP_TARGET_Z = aim_z;
    GlobalParam::ANTITOP_TARGET_PITCH = aim.pitch;
    GlobalParam::ANTITOP_TARGET_VALID = std::isfinite(aim_z) && std::isfinite(aim.pitch);

    // Yaw: 转换为角度并归一化到 IMU 近邻
    double yaw_cd = aim.yaw / M_PI * 180.0;
    while (std::abs(yaw_cd - imu_data.yaw) > ANGLE_FULL_CIRCLE / 2) {
        if (yaw_cd - imu_data.yaw >= ANGLE_FULL_CIRCLE / 2)
            yaw_cd -= ANGLE_FULL_CIRCLE;
        else
            yaw_cd += ANGLE_FULL_CIRCLE;
    }
    SerialParam::send_data.yaw = yaw_cd;

    // Pitch & Shoot: 拟合前跟随当前云台，拟合后持续写最低层 pitch
    auto now = std::chrono::steady_clock::now();

    switch (pitch_state) {
    case PitchLockState::FITTING:
        LOG(WARNING) << "PitchCase: FITTING";
        SerialParam::send_data.pitch = imu_data.pitch;
        SerialParam::send_data.shootStatus = 0;
        break;

    case PitchLockState::LOCKED_AIM:
        LOG(WARNING) << "PitchCase: LOCKED_AIM";
        SerialParam::send_data.pitch = aim.pitch;
        SerialParam::send_data.shootStatus = 0;
        break;

    case PitchLockState::FIRE_COUNTDOWN: {
        LOG(WARNING) << "PitchCase: FIRE_COUNTDOWN";
        SerialParam::send_data.pitch = aim.pitch;
        const int elapsed = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - fire_start_time).count());
        if (elapsed >= fire_delay_ms) {
            const double target_z = getTargetAimZ();
            const Angle_t fire_angle = calculateBallistics(
                Point3d(shootCenter.x, shootCenter.y, target_z));
            if (!std::isfinite(fire_angle.pitch)) {
                LOG(ERROR) << "Invalid fire pitch, unlock";
                pitch_state = PitchLockState::LOCKED_AIM;
                SerialParam::send_data.pitch = imu_data.pitch;
                SerialParam::send_data.shootStatus = 0;
                break;
            }
            SerialParam::send_data.pitch = fire_angle.pitch;
            SerialParam::send_data.shootStatus = 1;
            GlobalParam::ANTITOP_SHOOT_FLAG = true;
            pitch_state = PitchLockState::LOCKED_AIM;
            LOG(WARNING) << "FIRE!"
                         << " target_z=" << target_z
                         << " pitch=" << fire_angle.pitch;
        } else {
            SerialParam::send_data.shootStatus = 0;
            LOG(INFO) << "COUNTDOWN: " << (fire_delay_ms - elapsed) << "ms";
        }
        break;
    }
    }

    // 拟合前后都持续发送；开火帧额外带 shoot=1
    if (GlobalParam::IS_CALIBRATED && SerialParam::send_data.shootStatus == 1) {
        LOG(WARNING) << "================ ANTITOP FIRE COMMAND SENT ================";
    }

    serial_port->writeData(&SerialParam::send_data);
}

} // namespace estimator
