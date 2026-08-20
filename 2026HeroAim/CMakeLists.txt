cmake_minimum_required(VERSION 3.10)
project(HERO_26AutoAim)

# ==========================================
# 1. 基础配置
# ==========================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 生成 compile_commands.json 供 IDE 使用

# 设置编译类型 (Debug/Release)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# ==========================================
# 2. 查找依赖库
# (1) OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# (2) Eigen3 (矩阵运算)
find_package(Eigen3 3.3 REQUIRED)

# (4) Sophus (李代数库)
# 注意：Sophus 通常需要源码安装，如果找不到请确认安装路径
find_package(Sophus REQUIRED)

# 兼容不同 Sophus 安装导出的目标名/变量（对齐 2026HeroAim-master-1）
set(SOPHUS_LINK_TARGET "")
set(SOPHUS_INCLUDE_HINTS "")

if(TARGET Sophus::Sophus)
    set(SOPHUS_LINK_TARGET Sophus::Sophus)
elseif(TARGET Sophus)
    set(SOPHUS_LINK_TARGET Sophus)
elseif(DEFINED Sophus_LIBRARIES AND NOT "${Sophus_LIBRARIES}" STREQUAL "")
    set(SOPHUS_LINK_TARGET ${Sophus_LIBRARIES})
endif()

if(DEFINED Sophus_INCLUDE_DIRS)
    list(APPEND SOPHUS_INCLUDE_HINTS ${Sophus_INCLUDE_DIRS})
endif()
if(DEFINED Sophus_INCLUDE_DIR)
    list(APPEND SOPHUS_INCLUDE_HINTS ${Sophus_INCLUDE_DIR})
endif()

find_path(SOPHUS_HEADER_ROOT
    NAMES sophus/se3.hpp
    PATHS
        ${SOPHUS_INCLUDE_HINTS}
        /usr/include
        /usr/local/include
        $ENV{HOME}/.local/include
)

if(SOPHUS_HEADER_ROOT)
    list(APPEND SOPHUS_INCLUDE_HINTS ${SOPHUS_HEADER_ROOT})
    list(REMOVE_DUPLICATES SOPHUS_INCLUDE_HINTS)
    message(STATUS "Sophus include path: ${SOPHUS_HEADER_ROOT}")
endif()

# (5) Boost (串口通信与线程)
find_package(Boost REQUIRED COMPONENTS system thread)

# (6) Glog (日志库)
find_package(glog REQUIRED)

# (7) OpenVINO (神经网络推理)
find_package(OpenVINO REQUIRED)
message(STATUS "OpenVINO version: ${OpenVINO_VERSION}")

# (8) JsonCpp (JSON解析)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# (8.1) PoseLib (PnPL + RANSAC + bundle refinement)
find_package(PoseLib REQUIRED)
message(STATUS "Found PoseLib, using PoseLib PnPL solver")

# (8.2) cpp-httplib fallback
find_package(cpp-httplib QUIET CONFIG)
find_library(CPPHTTPLIB_LIB NAMES cpp-httplib httplib)
pkg_check_modules(CPPHTTPLIB QUIET cpp-httplib)

# (9) Daheng camera SDK, optional.
find_library(GXIAPI_LIB NAMES gxiapi libgxiapi PATHS /usr/lib /usr/local/lib)
find_library(DXIMAGEPROC_LIB NAMES dximageproc libdximageproc PATHS /usr/lib /usr/local/lib)

if(GXIAPI_LIB)
    message(STATUS "Found Daheng Camera Lib: ${GXIAPI_LIB}")
    set(USE_DAHENG_CAMERA ON)
    if(NOT DXIMAGEPROC_LIB)
        set(DXIMAGEPROC_LIB "")
    endif()
else()
    message(WARNING "⚠️ 未找到大恒相机SDK，将禁用大恒相机功能")
    set(USE_DAHENG_CAMERA OFF)
    set(GXIAPI_LIB "")
    set(DXIMAGEPROC_LIB "")
endif()

# (10) GStreamer + gst-app.
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.4 gstreamer-app-1.0>=1.4)
message(STATUS "Found GStreamer: ${GSTREAMER_VERSION}")

# ==========================================
# 3. 包含头文件路径
# 核心：将 src 设为根 include 路径，以支持 "utils/include/..." 写法
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/utils/include
    ${PROJECT_SOURCE_DIR}/src/Driver/include
    ${PROJECT_SOURCE_DIR}/src/Predictor/include
    ${PROJECT_SOURCE_DIR}/src/Detector/include
    ${PROJECT_SOURCE_DIR}/src/Tracker
    ${PROJECT_SOURCE_DIR}/src/AntiBase/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${SOPHUS_INCLUDE_HINTS}
    ${JSONCPP_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
)

# ==========================================
# 4. 自动搜集源文件
# 递归查找 src 目录下所有的 .cpp 文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 移除可能存在的临时文件或测试文件（可选）
# list(FILTER SOURCES EXCLUDE REGEX "test_.*\\.cpp")
list(FILTER SOURCES EXCLUDE REGEX ".*[/\\\\]Detector[/\\\\]src[/\\\\]NumberClassifier\\.cpp$")

# ==========================================
# 5. 生成可执行文件
add_executable(AutoAim ${SOURCES})
target_compile_options(AutoAim PRIVATE -U_res -Ures)

# ==========================================
# 6. 链接库

# 若 Sophus::Sophus 不存在，构造兼容目标
if(NOT TARGET Sophus::Sophus)
    if(TARGET Sophus)
        add_library(Sophus::Sophus INTERFACE IMPORTED)
        target_link_libraries(Sophus::Sophus INTERFACE Sophus)
        if(SOPHUS_INCLUDE_HINTS)
            target_include_directories(Sophus::Sophus INTERFACE ${SOPHUS_INCLUDE_HINTS})
        endif()
        message(STATUS "Create alias target Sophus::Sophus -> Sophus")
    elseif(DEFINED Sophus_LIBRARIES AND NOT "${Sophus_LIBRARIES}" STREQUAL "")
        add_library(Sophus::Sophus INTERFACE IMPORTED)
        target_link_libraries(Sophus::Sophus INTERFACE ${Sophus_LIBRARIES})
        if(SOPHUS_INCLUDE_HINTS)
            target_include_directories(Sophus::Sophus INTERFACE ${SOPHUS_INCLUDE_HINTS})
        endif()
        message(STATUS "Create interface target Sophus::Sophus from Sophus_LIBRARIES")
    elseif(SOPHUS_HEADER_ROOT)
        add_library(Sophus::Sophus INTERFACE IMPORTED)
        target_include_directories(Sophus::Sophus INTERFACE ${SOPHUS_INCLUDE_HINTS})
        message(STATUS "Create header-only target Sophus::Sophus from include path")
    else()
        message(FATAL_ERROR "未找到可用的 Sophus 目标/库/头文件，无法兼容 Sophus::Sophus")
    endif()
endif()

target_link_libraries(AutoAim
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Sophus::Sophus  # 或 ${Sophus_LIBRARIES}，取决于 Sophus 版本
    Boost::system
    Boost::thread
    glog::glog
    openvino::runtime
    ${JSONCPP_LIBRARIES}
    pthread         # 系统线程库
    ${GSTREAMER_LIBRARIES}
)

target_link_libraries(AutoAim PoseLib::PoseLib)
target_compile_definitions(AutoAim PRIVATE HAVE_POSELIB)

# 对齐 2026HeroAim-master-1：cpp-httplib 链接兜底
if(TARGET cpp-httplib::cpp-httplib)
    target_link_libraries(AutoAim cpp-httplib::cpp-httplib)
    message(STATUS "cpp-httplib link target: cpp-httplib::cpp-httplib")
elseif(TARGET httplib::httplib)
    target_link_libraries(AutoAim httplib::httplib)
    message(STATUS "cpp-httplib link target: httplib::httplib")
elseif(CPPHTTPLIB_LIB)
    target_link_libraries(AutoAim ${CPPHTTPLIB_LIB})
    message(STATUS "cpp-httplib link library: ${CPPHTTPLIB_LIB}")
elseif(CPPHTTPLIB_FOUND)
    target_include_directories(AutoAim PRIVATE ${CPPHTTPLIB_INCLUDE_DIRS})
    target_link_libraries(AutoAim ${CPPHTTPLIB_LIBRARIES})
    target_link_directories(AutoAim PRIVATE ${CPPHTTPLIB_LIBRARY_DIRS})
    message(STATUS "cpp-httplib link via pkg-config: ${CPPHTTPLIB_LIBRARIES}")
else()
    message(STATUS "cpp-httplib library not found, fallback to vendored/header-only httplib")
endif()

# 如果找到大恒相机SDK，链接相关库
if(USE_DAHENG_CAMERA)
    target_link_libraries(AutoAim ${GXIAPI_LIB} ${DXIMAGEPROC_LIB})
    target_compile_definitions(AutoAim PRIVATE USE_DAHENG_CAMERA)
endif()
# AntiBase compile flag for GStreamer H264 encoder.
target_compile_definitions(AutoAim PRIVATE HAVE_GSTREAMER_X264)
# ==========================================
# 7. 附加设置 (WSL 调试建议)
# ==========================================
if(EXISTS "/usr/lib/wsl/lib")
    message(STATUS "Detected WSL environment.")
endif()
