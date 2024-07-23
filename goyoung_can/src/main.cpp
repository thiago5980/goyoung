#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_can/motor_can_execution.cpp"

using namespace std::chrono_literals;
using namespace std::chrono;

// sudo modprobe peak_usb
// sudo ip link set can0 up type can bitrate 1000000
// sudo ifconfig can0 up

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanNode>());
    rclcpp::shutdown();
    return 0;
}
