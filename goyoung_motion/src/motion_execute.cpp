#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/robot.hpp"

#include "goyoung_motion/motion_execute.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionExecute>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}