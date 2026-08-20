#include "Estimator/include/NormalAim.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Params.hpp"
#include <algorithm>
#include <cmath>
#include "utils/include/EigenCompat.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace cv;

namespace estimator{
    namespace {
        double imageCenterDistance(const Armor& armor) {
            const double dx = armor.rect.x + armor.rect.width * 0.5 - IMAGE_CENTER_X;
            const double dy = armor.rect.y + armor.rect.height * 0.5 - IMAGE_CENTER_Y;
            return dx * dx + dy * dy;
        }
    }

    bool NormalAim::startAim(vector<Armor> &armors, double delta_t, const SerialPortData& imu_data,
                             SerialPort *SerialPort_) {

        candidates.clear();
        judgeFireCondition(imu_data);

        if (armors.empty()) {
            LOG(INFO) << "Armor Detection Empty";
            return false;
        }

        if (!selectTarget(armors)) {
            LOG(INFO) << "No Target Detected";
            return false;
        }

        if (!mainTainStatus()) {
            LOG(INFO) << "Target Lost";
            return false;
        }

        try {
            auto shoot_center = getShootCenter(imu_data);
            sendControlCommand(imu_data, std::move(shoot_center));
        } catch (const std::exception& e) {
            LOG(ERROR) << "Aiming process failed: " << e.what();
            return false;
        }

        return true;
    }

    bool NormalAim::selectTarget(std::vector<Armor>& armors){
        if (last_armor.is_armor) {
            std::vector<Armor> last_class_candidates;
            for (const auto& a : armors) {
                if (a._class == last_armor._class)
                    last_class_candidates.push_back(a);
            }

            if (!last_class_candidates.empty()) {
                auto best_candidate = std::min_element(last_class_candidates.begin(), last_class_candidates.end(),
                [&](const Armor &a, const Armor &b) {
                    return calcDiff(a, last_armor) < calcDiff(b, last_armor);
                });

                const double target_diff = calcDiff(*best_candidate, last_armor);
                if (target_diff <= ARMOR_DISTANCE_THRESHOLD) {
                    current_armor = *best_candidate;
                    current_armor.is_armor = true;
                    top_pri = current_armor._class;
                    candidates = std::move(last_class_candidates);
                    lost_cnt = 0;
                    SerialParam::send_data.num = top_pri;
                    LOG(INFO) << "Keep target: " << top_pri << " diff=" << target_diff;
                    return true;
                }

                if (lost_cnt < MAX_LOST_FRAME) {
                    current_armor = last_armor;
                    current_armor.is_armor = true;
                    top_pri = current_armor._class;
                    candidates.push_back(current_armor);
                    lost_cnt++;
                    SerialParam::send_data.num = top_pri;
                    LOG(INFO) << "Keep target after jump: " << top_pri
                              << " diff=" << target_diff
                              << " lost_cnt=" << lost_cnt << "/" << MAX_LOST_FRAME;
                    return true;
                }
            } else if (lost_cnt < MAX_LOST_FRAME) {
                current_armor = last_armor;
                current_armor.is_armor = true;
                top_pri = current_armor._class;
                candidates.push_back(current_armor);
                lost_cnt++;
                SerialParam::send_data.num = top_pri;
                LOG(INFO) << "Keep lost target: " << top_pri
                          << " lost_cnt=" << lost_cnt << "/" << MAX_LOST_FRAME;
                return true;
            }
        }

        sort(armors.begin(), armors.end(), [](const Armor &a, const Armor &b) -> bool {
            return imageCenterDistance(a) < imageCenterDistance(b);
        });

        top_pri = armors.at(0)._class;
        LOG(INFO) << "Choose target: " << top_pri << endl;
        SerialParam::send_data.num = top_pri;

        for (const auto &a: armors) {
            if (a._class == top_pri)
                candidates.push_back(a);
        }

        if (candidates.empty()) {
            is_first_shoot = true;
            return false;
        }

        current_armor = candidates.front();
        current_armor.is_armor = true;
        lost_cnt = 0;

        return true;
    }

    bool NormalAim::mainTainStatus(){
        if (!current_armor.is_armor)
            return false;
        last_armor = current_armor;
        last_armor.is_armor = true;
        is_first_shoot = false;
        return true;
    }

    Angle_t NormalAim::getShootCenter(const SerialPortData& imu_data){
        Angle_t tmp_angle;
        Point3d shoot_armor = Point3d(current_armor.x,current_armor.y,current_armor.z);

        shoot_armor.x = (shoot_armor.x == 0 ? 1e-6 : shoot_armor.x);
        tmp_angle.yaw = atan2(shoot_armor.y, shoot_armor.x);
        tmp_angle.distance = sqrt(shoot_armor.x * shoot_armor.x + shoot_armor.y * shoot_armor.y);
        double dz = shoot_armor.z;
        tmp_angle.pitch = atan(dz / tmp_angle.distance);
        double theta = correctShootCenter(tmp_angle.pitch, dz, tmp_angle.distance);
        float tmp_pitch = theta / M_PI * ANGLE_FULL_CIRCLE ;
        Angle_t correct_angle = {tmp_pitch, tmp_angle.yaw, tmp_angle.time, tmp_angle.distance};
        return correct_angle;

    }

    double NormalAim::correctShootCenter(double theta, double dz, double center_distance){
        double delta_z;
        const double drag_coeff = GlobalParam::BALLISTIC_DRAG_COEFF;
        const double air_density = GlobalParam::BALLISTIC_AIR_DENSITY;
        const double bullet_radius = GlobalParam::BALLISTIC_BULLET_RADIUS;
        const double bullet_mass = GlobalParam::BALLISTIC_BULLET_MASS;
        const double k1 = drag_coeff * air_density *
                          (M_PI * bullet_radius * bullet_radius) / 2.0 / bullet_mass;
        if(candidates[0].is_big_armor) center_distance = center_distance / BIG_ARMOR_DISTANCE_FACTOR;
        const double gravity = GlobalParam::BALLISTIC_GRAVITY;
        const double muzzle_offset = GlobalParam::BALLISTIC_MUZZLE_OFFSET;
        for (int i = 0; i < 100; i++) {
            double h_muzzle = center_distance - muzzle_offset * cos(theta);
            double dz_muzzle = dz - muzzle_offset * sin(theta);
            double t = (pow(2.718281828, k1 * h_muzzle) - 1) / (k1 * speed * cos(theta));
            double calc_z = speed * sin(theta) * t - 0.5 * gravity * t * t;
            delta_z = dz_muzzle - calc_z;
            if (fabs(delta_z) < 0.000001)
                break;
            double dt_dtheta = t * tan(theta);
            double df_dtheta = speed * cos(theta) * t + (speed * sin(theta) - gravity * t) * dt_dtheta;
            theta += delta_z / df_dtheta;
        }
        return theta;
    }

    void NormalAim::judgeFireCondition(const SerialPortData& imu_data){
        right_clicked = imu_data.right_clicked;
        if ((last_right_clicked == 0 && right_clicked == 1))
            is_first_shoot = true;
        last_right_clicked = right_clicked;
    }

}
