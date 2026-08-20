#include "Log.hpp"

#include <filesystem>
#include <iostream>
#include <system_error>

using namespace google;

namespace lyutils {
namespace {

std::filesystem::path resolveExecutableDir(const char* argv0)
{
#ifdef __linux__
    {
        std::error_code ec;
        const auto exe_path = std::filesystem::read_symlink("/proc/self/exe", ec);
        if (!ec && !exe_path.empty()) {
            return exe_path.parent_path();
        }
    }
#endif

    std::error_code argv_ec;
    const auto argv_path = std::filesystem::absolute(argv0 == nullptr ? "" : argv0, argv_ec);
    if (!argv_ec && !argv_path.empty()) {
        return argv_path.parent_path();
    }

    std::error_code cwd_ec;
    return std::filesystem::current_path(cwd_ec);
}

void configureLog(const char* argv0)
{
    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;

    InitGoogleLogging(argv0);

    const auto executable_dir = resolveExecutableDir(argv0);
    const auto log_dir = executable_dir.parent_path() / "logs";

    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        std::cerr << "[Log] Failed to create log directory: " << log_dir
                  << ", error: " << ec.message() << std::endl;
    }

    const std::string info_prefix = (log_dir / "INFO_").string();
    const std::string warning_prefix = (log_dir / "WARNING_").string();
    const std::string error_prefix = (log_dir / "ERROR_").string();
    const std::string fatal_prefix = (log_dir / "FATAL_").string();

    SetLogDestination(INFO, info_prefix.c_str());
    SetLogDestination(WARNING, warning_prefix.c_str());
    SetLogDestination(ERROR, error_prefix.c_str());
    SetLogDestination(FATAL, fatal_prefix.c_str());
    SetLogFilenameExtension("log");
}

} // namespace

Log::Log(const char* argv0)
{
    configureLog(argv0);
}

void Log::init(const char* argv0)
{
    configureLog(argv0);
}

Log::~Log()
{
    ShutdownGoogleLogging();
}
} // namespace lyutils
