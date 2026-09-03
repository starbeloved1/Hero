cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(DEFINED ENV{OpenVINO_DIR} AND NOT DEFINED OpenVINO_DIR)
  set(OpenVINO_DIR "$ENV{OpenVINO_DIR}" CACHE PATH "OpenVINO CMake 配置目录")
endif()
find_package(OpenVINO REQUIRED)

add_library(armor_utils src/armor_utils.cpp)
target_compile_features(armor_utils PUBLIC cxx_std_17)
target_include_directories(armor_utils PUBLIC include)

add_library(armor_detector src/armor_detector.cpp)
target_compile_features(armor_detector PUBLIC cxx_std_17)
target_include_directories(armor_detector PUBLIC include)
target_link_libraries(armor_detector PUBLIC armor_utils openvino::runtime ${OpenCV_LIBS})

add_library(armor_visualizer src/armor_visualizer.cpp)
target_compile_features(armor_visualizer PUBLIC cxx_std_17)
target_include_directories(armor_visualizer PUBLIC include)
target_link_libraries(armor_visualizer PUBLIC ${OpenCV_LIBS})

add_executable(armor_detector_node src/main.cpp src/armor_detector_node.cpp)
target_compile_features(armor_detector_node PUBLIC cxx_std_17)
target_include_directories(armor_detector_node PRIVATE include)
target_link_libraries(armor_detector_node armor_detector armor_visualizer)
ament_target_dependencies(armor_detector_node
  ament_index_cpp
  cv_bridge
  geometry_msgs
  hero_msgs
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_utils test/test_armor_utils.cpp)
  target_link_libraries(test_armor_utils armor_utils)
  ament_add_gtest(test_armor_visualizer test/test_armor_visualizer.cpp)
  target_link_libraries(test_armor_visualizer armor_visualizer)
  target_include_directories(test_armor_visualizer PRIVATE include)
endif()

install(TARGETS armor_utils armor_detector armor_visualizer armor_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
