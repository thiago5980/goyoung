#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_audio/goyoung_audio_execute.cpp"

using namespace std::chrono_literals;
using namespace std::chrono;

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioNode>());
    rclcpp::shutdown();
    return 0;
}
