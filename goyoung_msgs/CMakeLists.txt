cmake_minimum_required(VERSION 3.8)
project(goyoung_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_file
  "msg/Motor.msg"
  "msg/Robot.msg"
  "msg/Checkpoint.msg"
  "msg/Savefile.msg"
  "msg/Mode.msg"
  "msg/Motionpath.msg"
  "msg/Shutdown.msg"
)

set(srv_file
  "srv/Motionexecute.srv"
  "srv/Motortorque.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_file} ${srv_file}
  DEPENDENCIES 
    builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
