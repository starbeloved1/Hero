cmake_minimum_required(VERSION 3.8)
project(armor_solver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hero_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(armor_pnp src/armor_pnp.cpp)
target_compile_features(armor_pnp PUBLIC cxx_std_17)
target_include_directories(armor_pnp PUBLIC include)
target_link_libraries(armor_pnp PUBLIC ${OpenCV_LIBS})

add_executable(armor_solver_node src/main.cpp src/armor_solver_node.cpp)
target_compile_features(armor_solver_node PUBLIC cxx_std_17)
target_include_directories(armor_solver_node PRIVATE include)
target_link_libraries(armor_solver_node armor_pnp)
ament_target_dependencies(armor_solver_node
  geometry_msgs
  hero_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_armor_pnp test/test_armor_pnp.cpp)
  target_link_libraries(test_armor_pnp armor_pnp)
  target_include_directories(test_armor_pnp PRIVATE include)
endif()

install(TARGETS armor_pnp armor_solver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
