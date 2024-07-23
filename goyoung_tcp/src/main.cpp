#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_tcp/tcp_execution.cpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpNode>());
    rclcpp::shutdown();
    return 0;
}