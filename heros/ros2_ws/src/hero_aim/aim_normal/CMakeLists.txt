cmake_minimum_required(VERSION 3.8)
project(aim_normal)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aim_core REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(aim_normal_core src/armor_selector.cpp src/normal_aimer.cpp)
target_compile_features(aim_normal_core PUBLIC cxx_std_17)
target_include_directories(aim_normal_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aim_normal_core aim_core::aim_core)

add_executable(aim_normal_node src/aim_normal_node.cpp src/main.cpp)
target_compile_features(aim_normal_node PUBLIC cxx_std_17)
target_include_directories(aim_normal_node PRIVATE include)
target_link_libraries(aim_normal_node aim_normal_core)
ament_target_dependencies(aim_normal_node aim_core hero_msgs rclcpp)

install(TARGETS aim_normal_core aim_normal_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_selector test/test_armor_selector.cpp)
  target_link_libraries(test_armor_selector aim_normal_core)
  target_include_directories(test_armor_selector PRIVATE include)
  ament_add_gtest(test_normal_aimer test/test_normal_aimer.cpp)
  target_link_libraries(test_normal_aimer aim_normal_core)
  target_include_directories(test_normal_aimer PRIVATE include)
endif()

ament_package()
