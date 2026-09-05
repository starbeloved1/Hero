cmake_minimum_required(VERSION 3.8)
project(aim_auto)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(aim_auto_core src/auto_aimer.cpp)
target_compile_features(aim_auto_core PUBLIC cxx_std_17)
target_include_directories(aim_auto_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aim_auto_core PUBLIC Eigen3::Eigen aim_core::aim_core)

add_executable(aim_auto_node src/main.cpp src/aim_auto_node.cpp)
target_compile_features(aim_auto_node PUBLIC cxx_std_17)
target_include_directories(aim_auto_node PRIVATE include)
target_link_libraries(aim_auto_node aim_auto_core)
ament_target_dependencies(aim_auto_node aim_core hero_msgs rclcpp visualization_msgs)

install(TARGETS aim_auto_core aim_auto_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_auto_aimer test/test_auto_aimer.cpp)
  target_link_libraries(test_auto_aimer aim_auto_core)
  target_include_directories(test_auto_aimer PRIVATE include)
endif()

ament_package()
