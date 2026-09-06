cmake_minimum_required(VERSION 3.8)
project(hero_antibase)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

add_library(antibase_processor src/antibase_processor.cpp)
target_compile_features(antibase_processor PUBLIC cxx_std_17)
target_include_directories(antibase_processor PUBLIC include)
target_link_libraries(antibase_processor PUBLIC ${OpenCV_LIBS} PkgConfig::GSTREAMER)

add_executable(hero_antibase_node
  src/main.cpp
  src/hero_antibase_node.cpp
)
target_compile_features(hero_antibase_node PUBLIC cxx_std_17)
target_include_directories(hero_antibase_node PRIVATE include)
target_link_libraries(hero_antibase_node antibase_processor)
ament_target_dependencies(hero_antibase_node cv_bridge hero_msgs rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_antibase_processor test/test_antibase_processor.cpp)
  target_link_libraries(test_antibase_processor antibase_processor)
  target_include_directories(test_antibase_processor PRIVATE include)
endif()

install(TARGETS antibase_processor hero_antibase_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
