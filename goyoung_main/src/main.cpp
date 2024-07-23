#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "goyoung_main/goyoung_execute.cpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoyoungExecute>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
