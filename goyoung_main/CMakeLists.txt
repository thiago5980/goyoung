cmake_minimum_required(VERSION 3.8)
project(goyoung_main)

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

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

add_executable(goyoung_main src/main.cpp)
ament_target_dependencies(goyoung_main ${ROSDEPENDENCIES})
install(TARGETS goyoung_main DESTINATION lib/${PROJECT_NAME})

ament_package()
