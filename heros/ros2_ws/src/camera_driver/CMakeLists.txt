cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(DAHENG_SDK_ROOT "/opt/galaxy_sdk" CACHE PATH "Galaxy SDK 安装目录")
find_path(DAHENG_INCLUDE_DIR GxIAPI.h HINTS "${DAHENG_SDK_ROOT}/inc")
find_library(DAHENG_GXIAPI_LIBRARY NAMES gxiapi)
if(NOT DAHENG_INCLUDE_DIR OR NOT DAHENG_GXIAPI_LIBRARY)
  message(FATAL_ERROR "未找到 Galaxy C/C++ SDK；请安装 SDK 并设置 DAHENG_SDK_ROOT")
endif()

add_library(daheng_camera src/daheng_camera.cpp)
target_compile_features(daheng_camera PUBLIC cxx_std_17)
target_include_directories(daheng_camera PUBLIC include "${DAHENG_INCLUDE_DIR}")
target_link_libraries(daheng_camera PUBLIC "${DAHENG_GXIAPI_LIBRARY}")

add_library(camera_source src/camera_source.cpp)
target_compile_features(camera_source PUBLIC cxx_std_17)
target_include_directories(camera_source PUBLIC include)

add_executable(camera_driver_node src/main.cpp src/camera_driver_node.cpp)
target_compile_features(camera_driver_node PUBLIC cxx_std_17)
target_include_directories(camera_driver_node PRIVATE include)
target_link_libraries(camera_driver_node daheng_camera camera_source ${OpenCV_LIBS})
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_source test/test_camera_source.cpp)
  target_link_libraries(test_camera_source camera_source)
endif()

install(TARGETS daheng_camera camera_source camera_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
