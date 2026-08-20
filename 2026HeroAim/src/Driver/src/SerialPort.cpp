#include "Driver/include/SerialPort.hpp"
#include "utils/include/Params.hpp"
#include "utils/include/Log.hpp"
#include "utils/include/Thread.hpp"
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <thread>
#include <exception>
#include <limits>
#include <iomanip>
#include <sstream>

using namespace lyutils;

namespace driver
{
    namespace
    {
        std::string bytesToHex(const uint8_t* data, int len)
        {
            if (data == nullptr || len <= 0) {
                return "";
            }

            std::ostringstream oss;
            oss << std::hex << std::uppercase << std::setfill('0');
            for (int i = 0; i < len; ++i) {
                if (i > 0) {
                    oss << ' ';
                }
                oss << std::setw(2) << static_cast<int>(data[i]);
            }
            return oss.str();
        }
    }

    // 固定的从串口读取数据
    void SerialPort::read_data(Params_ToSerialPort& params)
    {
        (void)params;
        while (!Thread::should_stop)
        {
            try
            {
                SerialPortData data;
                this->readData(&data);
            }
            catch (...)
            {
                DLOG(ERROR) << "catch an error in SerialPort::read_data";
                break;
            }
            usleep(1000);                   // 规定的发送频率
        }
        return;
    }

    SerialPort::SerialPort()
    {
        new (this) SerialPort("/dev/ttyACM0");
    }

    SerialPort::SerialPort(const string &port_name)
    {
        // 本地视频/Mode4 回归不应依赖物理串口。保留原有 virtual 读写路径，
        // 仅在配置明确写为 virtual/mock 时启用，真实车端仍按原逻辑打开设备。
        if (port_name == "virtual" || port_name == "mock") {
            use_virtual_port_ = true;
            virtual_port_ = new VirtualSerialPort();
            LOG(INFO) << "Using virtual serial port for local test.";
            return;
        }
        try
        {
            LOG(INFO)<<"Try to connect to: "<<port_name<<endl;
            // 创建串口对象
            this->boost_port_ = new boost::asio::serial_port(_io_service, port_name);
            // 设置波特率
            this->boost_port_->set_option(boost::asio::serial_port::baud_rate(1000000));

            // 流量控制
            this->boost_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
            // 奇偶校验
            this->boost_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
            // 1位停止位
            this->boost_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
            // 8位数据位
            this->boost_port_->set_option(boost::asio::serial_port::character_size(8));
            // 串口发送数据tmp
            _data_tmp = (uint8_t *)malloc((size_t)_data_len);
            pingpong = (uint8_t *)malloc((size_t)_data_len * 2);
        }
        catch (...)
        {
            LOG(ERROR) << "create serial port object error! ";
            string port_name_back = "/dev/ttyACM1";
            try {
                LOG(INFO)<<"Try to connect to: "<<port_name_back<<endl;
                // 创建串口对象
                this->boost_port_ = new boost::asio::serial_port(_io_service, port_name_back);
                // 设置波特率
                this->boost_port_->set_option(boost::asio::serial_port::baud_rate(1000000));
                // 流量控制
                this->boost_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
                // 奇偶校验
                this->boost_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                // 1位停止位
                this->boost_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                // 8位数据位
                this->boost_port_->set_option(boost::asio::serial_port::character_size(8));
                // 串口发送数据tmp
                _data_tmp = (uint8_t *)malloc((size_t)_data_len);
                pingpong = (uint8_t *)malloc((size_t)_data_len * 2);
            }
            catch (...)
            {
                LOG(FATAL) << "create serial port error! " << port_name
                           << " and " << port_name_back
                           << " both failed. Exit without virtual serial port.";
            }
        }
    }

    SerialPort::~SerialPort()
    {
        stop();
        free(_data_tmp);
        free(pingpong);
        if(use_virtual_port_){
            // 虚拟模式下无需删除
        }else if(boost_port_){
            delete boost_port_;
        }
    }

    void SerialPort::stop()
    {
        if (use_virtual_port_) {
            return;
        }
        if (boost_port_) {
            boost::system::error_code ignored;
            boost_port_->cancel(ignored);
            boost_port_->close(ignored);
        }
    }

    // boost底层从串口读取数据
    void SerialPort::serialPortRead(uint8_t *msg, uint8_t max_len)
    {
        if (use_virtual_port_) {
            // 虚拟模式下不读取，直接返回
            memset(msg, 0, max_len);
            usleep(1000); // 模拟读取延迟
            return;
        }
        try
        {
            boost::asio::read(*boost_port_, boost::asio::buffer(msg, max_len), _err);
        }
        catch (...)
        {
            LOG(ERROR) << "readData from serial port error! " << _err.message();
        }
    }

    // boost底层向串口写入数据
    void SerialPort::serialPortWrite(uint8_t *msg, int len)
    {
        try
        {
            if (GlobalParam::SHOW_SERIAL) {
                static uint64_t tx_log_counter = 0;
                const bool is_antibase_chunk = len == 64 && msg != nullptr && msg[0] == '#';
                if (!is_antibase_chunk && ++tx_log_counter % 10 == 0) {
                    LOG(INFO) << "[Serial TX] len=" << len << " data=" << bytesToHex(msg, len);
                }
            }
            if (use_virtual_port_) {
                // virtual_port_->write(boost::asio::buffer(msg, len));
                //LOG(INFO) << "write to virtual serial port: " ;
            } else {
                boost::asio::write(*boost_port_, boost::asio::buffer(msg, len));
            }
        }
        catch (const std::exception& e)
        {
            LOG(FATAL) << "write to serial port error! " << e.what();
        }
        catch (...)
        {
            LOG(FATAL) << "write to serial port error! unknown exception";
        }
    }

    uint16_t SerialPort::getCRC16(const uint8_t *data, int len)
    {
        if (!data || len <= 0)
            return 0xFFFF;

        uint16_t crc = 0xFFFF;
        while (len--)
        {
            crc = (CRC16_TAB[(crc ^ (*data++)) & 0xFF]) ^ (crc >> 8);
        }

        return crc;
    }

    void SerialPort::appendCRC16(uint8_t *msg, int len)
    {
        if (!msg || len <= 2)
            return;

        const uint16_t crc = getCRC16(msg, len - 2);
        msg[len - 2] = static_cast<uint8_t>(crc & 0x00FF);
        msg[len - 1] = static_cast<uint8_t>((crc >> 8) & 0x00FF);
    }

    bool SerialPort::verifyCRC16(const uint8_t *data, int len)
    {
        if (!data || len <= 2)
            return false;

        const uint16_t expected = getCRC16(data, len - 2);
        return data[len - 2] == static_cast<uint8_t>(expected & 0x00FF) &&
               data[len - 1] == static_cast<uint8_t>((expected >> 8) & 0x00FF);
    }

    void SerialPort::recordExposureAdjustment(const SerialPortData& data)
    {
        const bool up = data.up != 0;
        const bool down = data.down != 0;
        // 同时置位不产生调整；仅记录 0→1，避免高电平持续时每个串口包都加减。
        if (!up && down && !last_down_) {
            pending_exposure_adjustment_steps_.fetch_sub(1, std::memory_order_relaxed);
        } else if (up && !down && !last_up_) {
            pending_exposure_adjustment_steps_.fetch_add(1, std::memory_order_relaxed);
        }
        last_up_ = up;
        last_down_ = down;
    }

    int SerialPort::consumeExposureAdjustmentSteps() noexcept
    {
        return pending_exposure_adjustment_steps_.exchange(0, std::memory_order_relaxed);
    }

    // 从串口读取imu数据
    // IMU 安装误差补偿角度（IMU 向下倾斜安装）
    // 可通过此参数微调 Z 坐标一致性
    // 理论值 2.0°，实际可能需要微调
    constexpr double IMU_PITCH_OFFSET_DEG = 0;
    
    void SerialPort::readData(SerialPortData *imu_data)
    {
        bool decoded = false;
        if (use_virtual_port_) {
            *imu_data = SerialPortData();

            imu_data->startflag = '!';
            imu_data->flag = 0x05;
            imu_data->pitch = 0.0f;
            imu_data->yaw = 0.0f;
            imu_data->up = 0;
            imu_data->down = 0;
            imu_data->color = 0;
            imu_data->right_clicked = 0;
            decoded = true;
        } else {
            serialPortRead(_data_tmp, _data_len);
            memcpy(pingpong + _data_len, _data_tmp, _data_len);
            for (int start_bit = 0; start_bit < _data_len; start_bit++)
            {
                if (pingpong[start_bit] == '!')
                {
                    memcpy(_data_tmp, pingpong + start_bit, _data_len);
                    if (verifyCRC16(_data_tmp, _data_len))
                    {
                        imu_data->startflag = _data_tmp[0];
                        imu_data->flag = _data_tmp[1];
                        memcpy(&imu_data->pitch, _data_tmp + 2, sizeof(float));
                        memcpy(&imu_data->yaw, _data_tmp + 6, sizeof(float));

                        while (imu_data->yaw > 180.0f) imu_data->yaw -= 360.0f;
                        while (imu_data->yaw < -180.0f) imu_data->yaw += 360.0f;

                        // 16B: !, flag, pitch, yaw, up, down, color, right_clicked, CRC16。
                        imu_data->up = _data_tmp[10];
                        imu_data->down = _data_tmp[11];
                        imu_data->color = _data_tmp[12];
                        imu_data->right_clicked = bool(_data_tmp[13]);
                        imu_data->crc16 = static_cast<uint16_t>(_data_tmp[14]) |
                                          (static_cast<uint16_t>(_data_tmp[15]) << 8);

                        imu_data->pitch = static_cast<float>(imu_data->pitch + IMU_PITCH_OFFSET_DEG);
                        decoded = true;

                        break;
                    }
                }
            }
            memcpy(pingpong, pingpong + _data_len, _data_len);
        }

        if (decoded) {
            recordExposureAdjustment(*imu_data);
            imu_data->recv_time = std::chrono::steady_clock::now();
            imu_data->valid = true;

            std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
            SerialParam::recv_data = *imu_data;
            if (!SerialParam::serial_data_sets.empty()) {
                SerialParam::serial_data_sets[SerialParam::set_id] = *imu_data;
                SerialParam::set_id = (SerialParam::set_id + 1) %
                                      static_cast<int>(SerialParam::serial_data_sets.size());
            }
        }
    }

    
    void SerialPort::writeData(SerialPortWriteData *_data_write)
    {
        msg[0] = '!';
        msg[1] = 0x05;

        const float raw_pitch = static_cast<float>(_data_write->pitch - IMU_PITCH_OFFSET_DEG);
        memcpy(msg + 2, &raw_pitch, sizeof(float));
        memcpy(msg + 6, &_data_write->yaw, sizeof(float));

        msg[10] = _data_write->shootStatus;
        msg[11] = _data_write->num;

        appendCRC16(msg, _data_len_write);
        _data_write->startflag = msg[0];
        _data_write->crc16 = static_cast<uint16_t>(msg[12]) |
                             (static_cast<uint16_t>(msg[13]) << 8);

        LOG(INFO) << "[Serial Send] mode=" << static_cast<int>(msg[1])
                  << " yaw=" << _data_write->yaw
                  << " pitch=" << _data_write->pitch
                  << " shootStatus=" << static_cast<int>(_data_write->shootStatus)
                  << " num=" << static_cast<int>(_data_write->num)
                  << " crc16=" << _data_write->crc16;
        serialPortWrite(msg, _data_len_write);
    }

    bool SerialPort::findNearestImu(const std::chrono::steady_clock::time_point& frame_time, SerialPortData& out)
    {
        std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
        bool found = false;
        auto best_delta = std::chrono::steady_clock::duration::max();

        for (const auto& data : SerialParam::serial_data_sets) {
            if (!data.valid) {
                continue;
            }

            auto delta = data.recv_time > frame_time ? data.recv_time - frame_time
                                                     : frame_time - data.recv_time;
            if (!found || delta < best_delta) {
                best_delta = delta;
                out = data;
                found = true;
            }
        }

        if (!found && SerialParam::recv_data.valid) {
            out = SerialParam::recv_data;
            found = true;
        }

        return found;
    }

    
    void SerialPort::updateImuData(SerialPortData& data, double angle_threshold) 
    {
        std::lock_guard<std::mutex> lock(SerialParam::serial_mutex);
        if (abs(SerialParam::recv_data.yaw - data.yaw) < angle_threshold  // 改为使用参数data
        && abs(SerialParam::recv_data.pitch - data.pitch) < angle_threshold)
        {
            data = SerialParam::recv_data; // 同步数据
        }
    }

    // ===== AntiBase CRC计算函数实现 =====
    // CRC8计算（帧头校验）
    // CRC16计算（整帧校验）
    uint16_t calculateCRC16_Frame(const uint8_t* data, uint16_t length) {
        uint16_t crc = 0xFFFF;
        while (length--) {
            crc = (CRC16_TAB[(crc ^ (*data++)) & 0xFF]) ^ (crc >> 8);
        }
        return crc;
    }

}
