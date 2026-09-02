cmake_minimum_required(VERSION 3.8)
project(gimbal_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(serial_protocol src/serial_protocol.cpp)
target_compile_features(serial_protocol PUBLIC cxx_std_17)
target_include_directories(serial_protocol PUBLIC include)

add_executable(gimbal_driver_node
  src/serial_port.cpp
  src/gimbal_driver_node.cpp
  src/main.cpp
)
target_compile_features(gimbal_driver_node PUBLIC cxx_std_17)
target_include_directories(gimbal_driver_node PRIVATE include)
target_link_libraries(gimbal_driver_node serial_protocol Boost::system)
ament_target_dependencies(gimbal_driver_node hero_msgs rclcpp)

install(TARGETS serial_protocol gimbal_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  target_link_libraries(test_serial_protocol serial_protocol)
  target_include_directories(test_serial_protocol PRIVATE include)
endif()

ament_package()
