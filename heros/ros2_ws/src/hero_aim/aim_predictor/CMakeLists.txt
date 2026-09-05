cmake_minimum_required(VERSION 3.8)
project(aim_predictor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(target_tracker src/target_tracker.cpp)
target_compile_features(target_tracker PUBLIC cxx_std_17)
target_include_directories(target_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(target_tracker PUBLIC Eigen3::Eigen)

add_executable(aim_predictor_node src/main.cpp src/aim_predictor_node.cpp)
target_compile_features(aim_predictor_node PUBLIC cxx_std_17)
target_include_directories(aim_predictor_node PRIVATE include)
target_link_libraries(aim_predictor_node target_tracker)
ament_target_dependencies(aim_predictor_node geometry_msgs hero_msgs rclcpp tf2 tf2_ros visualization_msgs)

install(TARGETS target_tracker aim_predictor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_target_tracker test/test_target_tracker.cpp)
  target_link_libraries(test_target_tracker target_tracker)
  target_include_directories(test_target_tracker PRIVATE include)
endif()

ament_package()
