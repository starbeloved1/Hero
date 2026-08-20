#ifndef AUTOAIM_THREAD_HPP
#define AUTOAIM_THREAD_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

using namespace std;

namespace lyutils
{

    class Thread
    {
    public:
        // 程序停止标志
        static atomic<bool> should_stop;
        
        // 表示图片是否已经更新
        static bool image_is_update;
        static bool image2_is_update;
        static bool image3_is_update;

        // 条件变量
        // 用于在 Detector 和 VideoCapture 之间控制同步关系, 明白即可
        static condition_variable cond_is_update;
        static condition_variable cond2_is_update;
        static condition_variable cond3_is_update;
        static condition_variable cond_is_process;
        static condition_variable cond2_is_process;
        static condition_variable cond3_is_process;

        // 静态互斥锁
        // static mutex mtx;
        static mutex mtx_image;     // 相机读取 以及 detector的互斥锁
        static mutex mtx_image2;
        static mutex mtx_image3;
        static mutex mtx_video;     // 保存视频 以及 detector的互斥锁
        static mutex mtx_video2;
    };
}

#endif //AUTOAIM_THREAD_H
