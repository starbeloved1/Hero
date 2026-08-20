#pragma once
#ifndef AUTOAIM_SERIALPORT_HPP
#define AUTOAIM_SERIALPORT_HPP

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <iostream>

#include "Log.hpp"

using namespace std;

namespace driver{

    // 根据串口flag确定操作模式，返回1/2/3
    int determineOperationMode(uint8_t flag);

    // ===== CRC校验表 =====
    // AntiBase CRC16查表（RM协议）
    const uint16_t CRC16_TAB[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
    };

    // CRC计算函数声明
    uint16_t calculateCRC16_Frame(const uint8_t* data, uint16_t length);

    // 读取数据
    struct SerialPortData{
        unsigned char startflag = 0;
        unsigned char flag = 0;
        float yaw = 0.0f;
        float pitch = 0.0f;
        uint8_t up = 0;
        uint8_t down = 0;
        unsigned char color = 0;
        bool right_clicked = false;
        uint16_t crc16 = 0;
        std::chrono::steady_clock::time_point recv_time;
        bool valid = false;

        SerialPortData(){

        }
        SerialPortData(const SerialPortData& data) {
            this->startflag = data.startflag;
            this->yaw = data.yaw;
            this->pitch = data.pitch;
            this->flag = data.flag;
            this->up = data.up;
            this->down = data.down;
            this->color = data.color;
            this->right_clicked = data.right_clicked;
            this->crc16 = data.crc16;
            this->recv_time = data.recv_time;
            this->valid = data.valid;
        }
    };

    struct SerialPortWriteData{
        unsigned char startflag = 0;
        float yaw = 0.0f;
        float pitch = 0.0f;
        unsigned char shootStatus = 0;
        unsigned char num = 0;
        uint16_t crc16 = 0;

        SerialPortWriteData(){

        }
        SerialPortWriteData(SerialPortWriteData *pData) {
        }
    };

    struct Params_ToSerialPort{
        SerialPortData* data;
        Params_ToSerialPort(){
        }
        Params_ToSerialPort(SerialPortData* recv){
            data = recv;
        }
    };


    class VirtualSerialPort {
    public:
        VirtualSerialPort() = default;

        bool is_open() const { return true; }
        void close() {}
    };

    class SerialPort {
    public:
        explicit SerialPort();
        explicit SerialPort(const string& port_name);
        ~SerialPort();
        void stop();
        void read_data(Params_ToSerialPort& params);
        void readData(SerialPortData *imu_data);
        void writeData(SerialPortWriteData *_data_write);
        bool findNearestImu(const std::chrono::steady_clock::time_point& frame_time, SerialPortData& out);
        void updateImuData(SerialPortData& data, double angle_threshold);
        void serialPortWrite(uint8_t* msg, int len);  // AntiBase需要的public接口
        // 返回自上次调用以来 up/down 的净上升沿次数：+ 为增加曝光，- 为减少曝光。
        int consumeExposureAdjustmentSteps() noexcept;

    private:
        boost::asio::serial_port* boost_port_ = nullptr;
        VirtualSerialPort* virtual_port_ = nullptr;
        bool use_virtual_port_ = false;
        
        boost::asio::io_service _io_service;

        static constexpr int PC_RECV_FRAME_SIZE = 16;
        static constexpr int PC_WRITE_FRAME_SIZE = 14;

        int _data_len = PC_RECV_FRAME_SIZE;
        int _data_len_write = PC_WRITE_FRAME_SIZE;
        unsigned char msg[PC_WRITE_FRAME_SIZE];

        uint8_t* _data_tmp = nullptr;
        uint8_t* pingpong = nullptr;
        Params_ToSerialPort thread_params;
        boost::system::error_code _err;

        SerialPortData stm32;
        bool BeginTime_Flag = false;
        std::atomic<int> pending_exposure_adjustment_steps_{0};
        bool last_up_ = false;
        bool last_down_ = false;

        void serialPortRead(uint8_t* msg, uint8_t max_len);

        void appendCRC16(uint8_t* msg, int len);

        uint16_t getCRC16(const uint8_t *data, int len);

        bool verifyCRC16(const uint8_t *data, int len);
        void recordExposureAdjustment(const SerialPortData& data);
    };

}

#endif //AUTOAIM_SERIALPORT_HPP
