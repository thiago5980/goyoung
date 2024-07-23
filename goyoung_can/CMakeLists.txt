cmake_minimum_required(VERSION 3.8)
project(goyoung_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCANBASIC_ROOT ${CMAKE_SOURCE_DIR}/../../../../pcanbasic)
include_directories(${PCANBASIC_ROOT}/include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(goyoung_msgs REQUIRED)
find_library(PCANBasic_LIB NAMES PCANBasic PATHS $ENV{PCAN_LIB_PATH})

link_directories(${PCANBASIC_ROOT}/lib)
include_directories(include)


set(ROSDEPENDENCIES
  goyoung_msgs
  rclcpp
  std_msgs
)


add_executable(motor_can_node src/main.cpp)
target_link_libraries(motor_can_node pcanbasic)
ament_target_dependencies(motor_can_node ${ROSDEPENDENCIES})
install(TARGETS motor_can_node DESTINATION lib/${PROJECT_NAME})

ament_package()
