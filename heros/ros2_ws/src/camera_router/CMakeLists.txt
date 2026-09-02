cmake_minimum_required(VERSION 3.8)
project(camera_router)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(camera_profile src/camera_profile.cpp)
target_compile_features(camera_profile PUBLIC cxx_std_17)
target_include_directories(camera_profile PUBLIC include)
ament_target_dependencies(camera_profile hero_msgs)

add_executable(camera_router_node
  src/main.cpp
  src/camera_router_node.cpp
)
target_compile_features(camera_router_node PUBLIC cxx_std_17)
target_include_directories(camera_router_node PRIVATE include)
target_link_libraries(camera_router_node camera_profile)
ament_target_dependencies(camera_router_node hero_msgs rclcpp sensor_msgs)

install(TARGETS camera_profile camera_router_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_profile test/test_camera_profile.cpp)
  target_link_libraries(test_camera_profile camera_profile)
  target_include_directories(test_camera_profile PRIVATE include)
  ament_target_dependencies(test_camera_profile hero_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(hero_msgs)
ament_package()
