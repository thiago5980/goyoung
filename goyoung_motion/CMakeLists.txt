cmake_minimum_required(VERSION 3.8)
project(goyoung_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(goyoung_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(ROSDEPDEPENDENCIES
  goyoung_msgs
  rclcpp
  std_msgs
)

add_executable(motion_record src/motion_record.cpp)
ament_target_dependencies(motion_record ${ROSDEPDEPENDENCIES})
target_link_libraries(motion_record ${YAML_CPP_LIBRARIES})
install(TARGETS 
  motion_record
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(motion_execute src/motion_execute.cpp)
ament_target_dependencies(motion_execute ${ROSDEPDEPENDENCIES})
target_link_libraries(motion_execute ${YAML_CPP_LIBRARIES})
install(TARGETS 
  motion_execute
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
