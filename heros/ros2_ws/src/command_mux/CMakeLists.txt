cmake_minimum_required(VERSION 3.8)
project(command_mux)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(candidate_selector src/candidate_selector.cpp)
target_compile_features(candidate_selector PUBLIC cxx_std_17)
target_include_directories(candidate_selector PUBLIC include)
ament_target_dependencies(candidate_selector hero_msgs)

add_executable(command_mux_node
  src/command_mux_node.cpp
  src/main.cpp
)
target_compile_features(command_mux_node PUBLIC cxx_std_17)
target_include_directories(command_mux_node PRIVATE include)
target_link_libraries(command_mux_node candidate_selector)
ament_target_dependencies(command_mux_node hero_msgs rclcpp)

install(TARGETS candidate_selector command_mux_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_candidate_selector test/test_candidate_selector.cpp)
  target_link_libraries(test_candidate_selector candidate_selector)
  target_include_directories(test_candidate_selector PRIVATE include)
  ament_target_dependencies(test_candidate_selector hero_msgs)
endif()

ament_package()
