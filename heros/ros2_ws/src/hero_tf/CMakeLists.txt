cmake_minimum_required(VERSION 3.8)
project(hero_tf)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(hero_tf_math src/transform_utils.cpp)
target_compile_features(hero_tf_math PUBLIC cxx_std_17)
target_include_directories(hero_tf_math PUBLIC include)
ament_target_dependencies(hero_tf_math tf2)

add_executable(hero_tf_node src/hero_tf_node.cpp)
target_compile_features(hero_tf_node PUBLIC cxx_std_17)
target_include_directories(hero_tf_node PRIVATE include)
target_link_libraries(hero_tf_node hero_tf_math)
ament_target_dependencies(hero_tf_node geometry_msgs hero_msgs rclcpp tf2 tf2_ros)

install(TARGETS hero_tf_math hero_tf_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transform_utils test/test_transform_utils.cpp)
  target_link_libraries(test_transform_utils hero_tf_math)
  target_include_directories(test_transform_utils PRIVATE include)
  ament_target_dependencies(test_transform_utils tf2)
endif()

ament_export_include_directories(include)
ament_export_dependencies(tf2)
ament_package()
