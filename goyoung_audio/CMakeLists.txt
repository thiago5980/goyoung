cmake_minimum_required(VERSION 3.8)
project(goyoung_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(goyoung_msgs REQUIRED)
find_package(SFML 2.5 REQUIRED audio graphics window system)

include_directories(include)


set(ROSDEPENDENCIES
  goyoung_msgs
  rclcpp
  std_msgs
)


add_executable(goyoung_audio src/main.cpp)
target_link_libraries(goyoung_audio sfml-audio sfml-graphics sfml-window sfml-system)
ament_target_dependencies(goyoung_audio ${ROSDEPENDENCIES})
install(TARGETS goyoung_audio DESTINATION lib/${PROJECT_NAME})



ament_package()
