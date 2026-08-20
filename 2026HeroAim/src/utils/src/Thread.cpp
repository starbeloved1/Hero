#include "Thread.hpp"

namespace lyutils{
    atomic<bool> Thread::should_stop{false};
    bool Thread::image_is_update = false;
    bool Thread::image2_is_update = false;
    bool Thread::image3_is_update = false;

    condition_variable Thread::cond_is_update;
    condition_variable Thread::cond2_is_update;
    condition_variable Thread::cond3_is_update;
    condition_variable Thread::cond_is_process;
    condition_variable Thread::cond2_is_process;
    condition_variable Thread::cond3_is_process;

    // mutex Thread::mtx;
    mutex Thread::mtx_image;
    mutex Thread::mtx_image2;
    mutex Thread::mtx_image3;
    mutex Thread::mtx_video;
    mutex Thread::mtx_video2;
}
