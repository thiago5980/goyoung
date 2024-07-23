#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <SFML/Audio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

class AudioNode : public rclcpp::Node
{
public:
    AudioNode() : Node("audio_node")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        sub_mode = this->create_subscription<goyoung_msgs::msg::Mode>("robot_motion_mode", qos_profile, std::bind(&AudioNode::mode_callback, this, _1));
        audio_exe();
    }
    ~AudioNode(){}

private:
    rclcpp::Subscription<goyoung_msgs::msg::Mode>::SharedPtr sub_mode;
    void mode_callback(const goyoung_msgs::msg::Mode::SharedPtr msg) // 태블릿에서 넘어온 mode에 따라 로봇의 음성 파일의 실행
    {
        if (first_flag)
            first_flag = false;
        else
            end_flag = true;

        if (msg->mode == 0)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/stay.wav";
        }
        else if (msg->mode == 1)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/hello.wav";
        }
        else if (msg->mode == 2)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/song.wav";
        }
        else if (msg->mode == 3)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/iot.wav";

        }
        else if (msg->mode == 4)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/bio.wav";
            
        }
        else if (msg->mode == 5)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/interia.wav";
            
        }
        else if (msg->mode == 6)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/beauty.wav";
            
        }
        else if (msg->mode == 7)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/cook.wav";
            
        }
        else if (msg->mode == 8)
        {
            this->file_name = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_audio/data/bread.wav";
        }

        start_flag = true;
    }

    std::string file_name = "";
    bool first_flag = true;
    
    bool start_flag = false;
    bool end_flag = false;
    
    sf::Music music;

    void audio_play(std::string file_name)
    {
        if (music.getStatus() == sf::Music::Stopped)
        {
            end_flag = false;
        }
        
        if (!music.openFromFile(file_name.c_str())) { 
            std::cout << "Could not open file" << std::endl;
            return;
        }

        music.play();

        // 음악이 끝날 때까지 대기
        while (music.getStatus() == sf::Music::Playing && rclcpp::ok())  
        {
            sf::sleep(sf::milliseconds(100));
            std::cout << "Playing..." << std::endl;
            if (end_flag)
            {
                music.stop();
                end_flag = false;
                break;
            }
            rclcpp::spin_some(this->get_node_base_interface());
        }

        std::cout << "Playback finished." << std::endl;
    }

    
    void audio_exe() // 해당 함수의 경우 로봇에 장착되어 있는 코드는 변경되어 있으니 유의 바람
    {
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            if (start_flag)
            {
                start_flag = false;
                audio_play(file_name); // 저장된 file_name에 따라 음성 파일을 실행
            }
        }
    }
};