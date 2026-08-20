#pragma once
#include <cstdint>

namespace controller
{
    struct ControlResult
    {
        uint8_t shoot_flag;//0,1,2
        float pitch_setpoint;//
        float yaw_setpoint;//定义的拟合发射点的中心
        float pitch_actual_want;
        float yaw_actual_want;//相机返回值的坐标中心
        int target_car_id = -1;    // Controller选中的目标车辆ID
        int target_armor_id = -1;  // Controller选中的装甲板ID (0-3)
        float flyTime_s = 0.1f;    // 预测时间(s)：弹道飞行时间+系统延迟，供可视化使用
        
        // 可视化数据
        float vx = 0.0f;           // 车辆X轴速度 (m/s)
        float vy = 0.0f;           // 车辆Y轴速度 (m/s)
        float ax = 0.0f;           // 车辆X轴加速度 (m/s²)
        float ay = 0.0f;           // 车辆Y轴加速度 (m/s²)
        float armor_angle_deg = 0.0f;  // 目标装甲板朝向角 (度)
        int phase_ready_cnt = 0;       // 对准帧数
        int phase_confirm_frames = 5;  // 确认帧数阈值
        float accel_threshold = 5.0f;  // 加速度阈值 (m/s²)
        bool high_accel_mode = false;  // 高加速度模式标志
    };
}