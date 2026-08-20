#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP


#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>


namespace { 
using namespace std::literals::chrono_literals;
// 时间差
class TimeDiff
{
public:
    TimeDiff() : duration(0ns) {}
    TimeDiff(std::chrono::nanoseconds duration) : duration(duration) {}

    template <typename T>
    T get() const {
        return std::chrono::duration_cast<T>(duration);
    }
    double count() const {
        return std::chrono::duration<double, std::milli>(duration).count();
    }//返回毫秒数的时间差。
    double toSeconds() const {
        // auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        // return static_cast<double>(us) / 1'000'000;
        return std::chrono::duration<double>(duration).count();
    }
    operator std::chrono::nanoseconds() const {
        return duration;
    }

    operator std::string() const {
        //show xxx.xxx ms
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration) % 1000;
        std::stringstream ss;
        ss << milliseconds.count() << '.' << std::setw(3) << std::setfill('0') << microseconds.count() << " ms";
        return ss.str();
    }

    // SLOW FUNCTION
    friend std::ostream& operator<<(std::ostream& os, const TimeDiff& td) {
        // according to the specific value to depend the unit
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(td.duration);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(td.duration) % 1000;
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(td.duration) % 1000;
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(td.duration) % 1000;
        if (microseconds.count() == 0) {
            os << nanoseconds.count() << " ns";
        }
        else if (milliseconds.count() == 0) {
            os << microseconds.count() << '.' << std::setw(3) << std::setfill('0') << nanoseconds.count() << " us";
        }
        else if (seconds.count() == 0) {
            os << milliseconds.count() << '.' << std::setw(3) << std::setfill('0') << microseconds.count() << " ms";
        }
        else if (seconds.count() < 1000) {
            os << seconds.count() << '.' << std::setw(3) << std::setfill('0') << milliseconds.count() << " s";
        }
        else {
            os << seconds.count() << " s";
        }
        return os;
    }

#pragma region 重载所有比较运算符
    template <typename T>
    bool operator<(const T& other) const {
        return duration < std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }

    template <typename T>
    bool operator<=(const T& other) const {
        return duration <= std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }

    template <typename T>
    bool operator>(const T& other) const {
        return duration > std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }

    template <typename T>
    bool operator>=(const T& other) const {
        return duration >= std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }

    template <typename T>
    bool operator==(const T& other) const {
        return duration == std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }

    template <typename T>
    bool operator!=(const T& other) const {
        return duration != std::chrono::duration_cast<std::chrono::nanoseconds>(other);
    }
#pragma endregion

private:
    std::chrono::nanoseconds duration;
};

#pragma region 时间差比较运算符重载

template <typename T>
bool operator<(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) < rhs.get<std::chrono::nanoseconds>();
}

template <typename T>
bool operator<=(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) <= rhs.get<std::chrono::nanoseconds>();
}

template <typename T>
bool operator>(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) > rhs.get<std::chrono::nanoseconds>();
}

template <typename T>
bool operator>=(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) >= rhs.get<std::chrono::nanoseconds>();
}

template <typename T>
bool operator==(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) == rhs.get<std::chrono::nanoseconds>();
}

template <typename T>
bool operator!=(const T& lhs, const TimeDiff& rhs) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(lhs) != rhs.get<std::chrono::nanoseconds>();
}
#pragma endregion

}

namespace Time{
using namespace std::literals::chrono_literals;
// 时间戳
class TimeStamp {
public:
    TimeStamp() : time_point(std::chrono::system_clock::now()) {}
    TimeStamp(const std::chrono::system_clock::time_point& time_point) : time_point(time_point) {}

    // 获取当前时间
    static TimeStamp now() {
        return TimeStamp();
    }

    void reset() {
        time_point = std::chrono::system_clock::now();
    }

    std::string toYearString() const {
        std::time_t time = std::chrono::system_clock::to_time_t(time_point);
        std::tm tm = *std::localtime(&time);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    std::string toString() const {
        std::time_t time = std::chrono::system_clock::to_time_t(time_point);
        std::tm tm = *std::localtime(&time);
        std::stringstream ss;
        ss << std::put_time(&tm, "%H:%M:%S");

        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch()) % 1000;
        ss << ':' << std::setw(3) << std::setfill('0') << milliseconds.count();

        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_point.time_since_epoch()) % 1000;
        ss << '.' << std::setw(3) << std::setfill('0') << microseconds.count();

        return ss.str();
    }
    // 从字符串解析时间戳 (格式: "HH:MM:SS:MMM.μμμ")
    void fromString(const std::string& str) {
        // 获取当前日期，只替换时间部分
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        
        // 解析小时:分钟:秒
        int hour = 0, minute = 0, second = 0, millisecond = 0, microsecond = 0;
        
        std::stringstream ss(str);
        char delimiter;
        
        // 格式: "HH:MM:SS:MMM.μμμ"
        ss >> hour >> delimiter; // 读取小时和第一个冒号
        if (delimiter != ':') throw std::runtime_error("Invalid time format");
        
        ss >> minute >> delimiter; // 读取分钟和第二个冒号
        if (delimiter != ':') throw std::runtime_error("Invalid time format");
        
        ss >> second >> delimiter; // 读取秒和第三个冒号
        if (delimiter != ':') throw std::runtime_error("Invalid time format");
        
        ss >> millisecond >> delimiter; // 读取毫秒和点号
        if (delimiter != '.') throw std::runtime_error("Invalid time format");
        
        ss >> microsecond; // 读取微秒
        
        // 创建tm结构体
        std::tm time_info = now_tm;
        time_info.tm_hour = hour;
        time_info.tm_min = minute;
        time_info.tm_sec = second;
        
        // 转换为time_t
        std::time_t time_seconds = std::mktime(&time_info);
        
        // 创建时间点
        auto timepoint = std::chrono::system_clock::from_time_t(time_seconds);
        
        // 添加毫秒和微秒
        timepoint += std::chrono::milliseconds(millisecond);
        timepoint += std::chrono::microseconds(microsecond);
        
        // 设置时间点
        time_point = timepoint;
    }

    //min() return the smallest possible time_point
    static TimeStamp min() {
        return TimeStamp(std::chrono::system_clock::time_point::min());
    }

    operator std::string() const {
        return toString();
    }

    friend std::ostream& operator<<(std::ostream& os, const TimeStamp& ts) {
        os << ts.toString();
        return os;
    }

#pragma region 重载compare运算符
    // 重载-运算符
    TimeDiff operator-(const TimeStamp& other) const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time_point - other.time_point);
    }

    TimeStamp operator+(const std::chrono::duration<double>& duration) const {
        return TimeStamp(time_point + std::chrono::duration_cast<std::chrono::nanoseconds>(duration));
    }
    TimeStamp operator+(double seconds) const {
        return *this + std::chrono::duration<double>(seconds);
    }
    // 重载>运算符
    bool operator>(const TimeStamp& other) const {
        return time_point > other.time_point;
    }
    // 重载>=运算符
    bool operator>=(const TimeStamp& other) const {
        return time_point >= other.time_point;
    }
    // 重载<运算符
    bool operator<(const TimeStamp& other) const {
        return time_point < other.time_point;
    }
    // 重载<=运算符
    bool operator<=(const TimeStamp& other) const {
        return time_point <= other.time_point;
    }
#pragma endregion

private:
    std::chrono::system_clock::time_point time_point;
};


}

#endif // TIMESTAMP_HPP