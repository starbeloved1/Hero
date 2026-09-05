cmake_minimum_required(VERSION 3.8)
project(aim_antitop)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(aim_antitop_core src/antitop_tracker.cpp src/antitop_aimer.cpp src/antitop_controller.cpp)
target_compile_features(aim_antitop_core PUBLIC cxx_std_17)
target_include_directories(aim_antitop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aim_antitop_core PUBLIC Eigen3::Eigen aim_core::aim_core)

add_executable(aim_antitop_node src/main.cpp src/aim_antitop_node.cpp)
target_compile_features(aim_antitop_node PUBLIC cxx_std_17)
target_include_directories(aim_antitop_node PRIVATE include)
target_link_libraries(aim_antitop_node aim_antitop_core ${OpenCV_LIBS})
ament_target_dependencies(aim_antitop_node aim_core hero_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

install(TARGETS aim_antitop_core aim_antitop_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_antitop_tracker test/test_antitop_tracker.cpp)
  target_link_libraries(test_antitop_tracker aim_antitop_core)
  target_include_directories(test_antitop_tracker PRIVATE include)
  ament_add_gtest(test_antitop_aimer test/test_antitop_aimer.cpp)
  target_link_libraries(test_antitop_aimer aim_antitop_core)
  target_include_directories(test_antitop_aimer PRIVATE include)
  ament_add_gtest(test_antitop_controller test/test_antitop_controller.cpp)
  target_link_libraries(test_antitop_controller aim_antitop_core)
  target_include_directories(test_antitop_controller PRIVATE include)
endif()

ament_package()
