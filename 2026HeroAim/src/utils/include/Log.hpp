#ifndef AUTOAIM_LOG_HPP
#define AUTOAIM_LOG_HPP

#include "glog/logging.h"

#define DEBUG_MODE(op) (DLOG_IF(INFO, (op, false)))

namespace lyutils
{
    class Log
    {
    public:
        Log() = default;
        ~Log();
        explicit Log(const char *argv0);
        void init(const char *argv0);
    };
} 

#endif //AUTOAIM_LOG_H
