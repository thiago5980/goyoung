cmake_minimum_required(VERSION 3.8)
project(goyoung_tcp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(goyoung_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(ROSDEPENDENCIES
  rclcpp
  std_msgs
  goyoung_msgs
)

add_executable(tcp_server src/main.cpp)
ament_target_dependencies(tcp_server ${ROSDEPENDENCIES})
install(TARGETS tcp_server DESTINATION lib/${PROJECT_NAME})

ament_package()
