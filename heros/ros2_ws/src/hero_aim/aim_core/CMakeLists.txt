cmake_minimum_required(VERSION 3.8)
project(aim_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(aim_core
  src/angle_utils.cpp
  src/ballistics.cpp
)
target_compile_features(aim_core PUBLIC cxx_std_17)
target_include_directories(aim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS aim_core EXPORT export_aim_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_angle_utils test/test_angle_utils.cpp)
  target_link_libraries(test_angle_utils aim_core)
  ament_add_gtest(test_ballistics test/test_ballistics.cpp)
  target_link_libraries(test_ballistics aim_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_aim_core HAS_LIBRARY_TARGET)
ament_package()
