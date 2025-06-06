cmake_minimum_required(VERSION 3.8)
project(crawling_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)

# 헤더 경로 추가
include_directories(
  include
  include/crawling_robot_controller
  include/crawling_robot_controller/serial
  include/crawling_robot_controller/serial/impl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

add_library(serial_lib
  src/serial/serial.cc
  src/serial/unix.cc
)

target_include_directories(serial_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(position_control src/position_control.cpp)

target_link_libraries(position_control
  serial_lib
)

ament_target_dependencies(position_control
  rclcpp
  std_msgs
  dynamixel_sdk
  geometry_msgs
)

install(TARGETS
  position_control
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
