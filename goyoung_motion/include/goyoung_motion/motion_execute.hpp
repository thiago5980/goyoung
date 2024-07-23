#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/robot.hpp"
#include "goyoung_msgs/msg/savefile.hpp"
#include "goyoung_msgs/msg/motionpath.hpp"
#include "goyoung_msgs/srv/motionexecute.hpp"
#include "goyoung_msgs/msg/checkpoint.hpp"
#include "std_msgs/msg/int8.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

///// motor num //////
///// chest : 5    ///
///// left 1 : 3   ///
///// left 2 : 6   ///
///// left 3 : 4   ///
///// left 4 : 9   ///
///// right 1 : 7  ///
///// right 2 : 11 ///
///// right 3 : 10 ///
///// right 4 : 8  ///
//////////////////////



class MotionExecute : public rclcpp::Node
{
public:
    MotionExecute()
    : Node("motion_execute")
    {
        motor_subscription_ = this->create_subscription<goyoung_msgs::msg::Robot>(  // motor state를 받아오는 subscriber
          "initial_motor_state",
          rclcpp::SensorDataQoS(),
          std::bind(&MotionExecute::motor_cb_message, this, _1));
        motion_publisher_ = this->create_publisher<goyoung_msgs::msg::Robot>("start_motion", rclcpp::SensorDataQoS());  // motor state를 publish하는 publisher
        motion_service_ = this->create_service<goyoung_msgs::srv::Motionexecute>(   // motion 실행을 위한 service
          "motion_path",
          std::bind(&MotionExecute::motion_cb_message, this, _1, _2));
        end_pose_subscriber_ = this->create_subscription<goyoung_msgs::msg::Checkpoint>(    // end pose를 받아오는 subscriber (현재는 사용하지 않음)
          "end_pose",
          rclcpp::SensorDataQoS(),
          std::bind(&MotionExecute::end_pose_cb_message, this, _1));
    }
private:
    void end_pose_cb_message(const goyoung_msgs::msg::Checkpoint::SharedPtr msg)
    {
        end_pose = true;
        // RCLCPP_INFO(this->get_logger(), "end_pose");
    }
    
    void motor_cb_message(const goyoung_msgs::msg::Robot::SharedPtr msg)    // motor state를 받아오는 subscriber callback
    {
        this->robot_state_ = *msg;
        is_start = true;
    }
    void motion_cb_message(const goyoung_msgs::srv::Motionexecute::Request::SharedPtr request,  // motion 실행을 위한 service callback
                           goyoung_msgs::srv::Motionexecute::Response::SharedPtr response)
    {
        // RCLCPP_INFO(this->get_logger(), "motion start");
        std::cout << "motion start" << std::endl;
        this->readMotionfile(request->file);    // motion file을 읽어오는 함수(모션의 각 state와 각 state의 시간을 읽어옴-형식은 yaml)
        if (is_start == false)
        {
            std::cerr << "Error: initial_motor_state is empty" << std::endl;
            return;
        }

        else
        {
            auto send = this->motion_start(request->file);   // motion 실행 함수
            RCLCPP_INFO(this->get_logger(), "motion end");
            record_state.clear();
            start_time.clear();
            end_time.clear();
            is_start = false;
            response->success = send;
        }
    }

    bool motion_start(std::string filename)
    {
        if (record_state.empty() || start_time.empty() || end_time.empty())
        {
            std::cerr << "Error: record_state or start_time or end_time is empty" << std::endl;
            return false;
        }
        std::chrono::milliseconds interval(1000 / TIME_HZ);

        auto start = std::chrono::steady_clock::now();
        auto send_motion = goyoung_msgs::msg::Robot();

        // 처음 모션 실행 전 위치 값과 이후 모션 실행 값의 차이를 이용하여 각 모터의 속도를 계산
        send_motion.left.m1_v = static_cast<int16_t>((abs(record_state[0].left.m1 - this->robot_state_.left.m1))/end_time[0]);
        send_motion.left.m2_v = static_cast<int16_t>((abs(record_state[0].left.m2 - this->robot_state_.left.m2))/end_time[0]);
        send_motion.left.m3_v = static_cast<int16_t>((abs(record_state[0].left.m3 - this->robot_state_.left.m3))/end_time[0]);
        send_motion.left.m4_v = static_cast<int16_t>((abs(record_state[0].left.m4 - this->robot_state_.left.m4))/end_time[0]);
        send_motion.right.m1_v = static_cast<int16_t>((abs(record_state[0].right.m1 - this->robot_state_.right.m1))/end_time[0]);
        send_motion.right.m2_v = static_cast<int16_t>((abs(record_state[0].right.m2 - this->robot_state_.right.m2))/end_time[0]);
        send_motion.right.m3_v = static_cast<int16_t>((abs(record_state[0].right.m3 - this->robot_state_.right.m3))/end_time[0]);
        send_motion.right.m4_v = static_cast<int16_t>((abs(record_state[0].right.m4 - this->robot_state_.right.m4))/end_time[0]);
        send_motion.chest_v = static_cast<int16_t>((abs(record_state[0].chest - this->robot_state_.chest))/end_time[0]);
        send_motion.left.m1 = record_state[0].left.m1;
        send_motion.left.m2 = record_state[0].left.m2;
        send_motion.left.m3 = record_state[0].left.m3;
        send_motion.left.m4 = record_state[0].left.m4;
        send_motion.right.m1 = record_state[0].right.m1;
        send_motion.right.m2 = record_state[0].right.m2;
        send_motion.right.m3 = record_state[0].right.m3;
        send_motion.right.m4 = record_state[0].right.m4;
        send_motion.chest = record_state[0].chest;
        send_motion.num = record_state[0].num;
        this->motion_publisher_->publish(send_motion); // 모터 속도와 위치값을 publish(CAN 통신을 통해 모터를 제어하는 노드에 전달)
        while ((std::chrono::steady_clock::now() - start < std::chrono::milliseconds(static_cast<int>(end_time[0] * 1000))) && rclcpp::ok()) // 해당 시간동안 기다
        {
            rclcpp::sleep_for(std::chrono::milliseconds(interval));
        }
        
        for (unsigned int i=1; i<start_time.size(); i++) // 이전과 동일하게 다음 모터 저장값과 현재 값의 차이를 이용하여 모터 속도 계산
        {
            send_motion.left.m1_v = static_cast<int16_t>((abs(record_state[i].left.m1 - record_state[i-1].left.m1))/(end_time[i] - start_time[i]));
            send_motion.left.m2_v = static_cast<int16_t>((abs(record_state[i].left.m2 - record_state[i-1].left.m2))/(end_time[i] - start_time[i]));
            send_motion.left.m3_v = static_cast<int16_t>((abs(record_state[i].left.m3 - record_state[i-1].left.m3))/(end_time[i] - start_time[i]));
            send_motion.left.m4_v = static_cast<int16_t>((abs(record_state[i].left.m4 - record_state[i-1].left.m4))/(end_time[i] - start_time[i]));
            send_motion.right.m1_v = static_cast<int16_t>((abs(record_state[i].right.m1 - record_state[i-1].right.m1))/(end_time[i] - start_time[i]));
            send_motion.right.m2_v = static_cast<int16_t>((abs(record_state[i].right.m2 - record_state[i-1].right.m2))/(end_time[i] - start_time[i]));
            send_motion.right.m3_v = static_cast<int16_t>((abs(record_state[i].right.m3 - record_state[i-1].right.m3))/(end_time[i] - start_time[i]));
            send_motion.right.m4_v = static_cast<int16_t>((abs(record_state[i].right.m4 - record_state[i-1].right.m4))/(end_time[i] - start_time[i]));
            send_motion.chest_v = static_cast<int16_t>((abs(record_state[i].chest - record_state[i-1].chest))/(end_time[i] - start_time[i]));
            send_motion.left.m1 = record_state[i].left.m1;
            send_motion.left.m2 = record_state[i].left.m2;
            send_motion.left.m3 = record_state[i].left.m3;
            send_motion.left.m4 = record_state[i].left.m4;
            send_motion.right.m1 = record_state[i].right.m1;
            send_motion.right.m2 = record_state[i].right.m2;
            send_motion.right.m3 = record_state[i].right.m3;
            send_motion.right.m4 = record_state[i].right.m4;
            send_motion.chest = record_state[i].chest;
            send_motion.num = record_state[i].num;
            this->motion_publisher_->publish(send_motion);
            while(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(static_cast<int>(end_time[i] * 1000)) && rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(interval));
            }
        }
        return true;
    }

    bool readMotionfile(std::string filename)   // motion file을 읽어오는 함수 (LED의 경우 새로운 버전에서는 있으나 현재 버전에서는 없음)
    {
        auto home = std::filesystem::path(std::getenv("HOME"));
        auto filepath = home / filename;
        // std::cout << filepath << std::endl;
        YAML::Node docs = YAML::LoadFile(filepath);
        const YAML::Node& size = docs["size"];;
        // std::cout << "action size : " << size << std::endl;
        const YAML::Node& sequences = docs["sequences"];
        for (const auto& seq : sequences)
        {
            goyoung_msgs::msg::Robot _motion;
            if (seq["left_motor1"]) 
                _motion.left.m1 = seq["left_motor1"].as<float>();
            if (seq["left_motor2"]) 
                _motion.left.m2 = seq["left_motor2"].as<float>();
            if (seq["left_motor3"]) 
                _motion.left.m3 = seq["left_motor3"].as<float>();
            if (seq["left_motor4"]) 
                _motion.left.m4 = seq["left_motor4"].as<float>();
            if (seq["right_motor1"]) 
                _motion.right.m1 = seq["right_motor1"].as<float>();
            if (seq["right_motor2"]) 
                _motion.right.m2 = seq["right_motor2"].as<float>();
            if (seq["right_motor3"]) 
                _motion.right.m3 = seq["right_motor3"].as<float>();
            if (seq["right_motor4"]) 
                _motion.right.m4 = seq["right_motor4"].as<float>();
            if (seq["chest"]) 
                _motion.chest = seq["chest"].as<float>();
            if (seq["mode"])
                _motion.num = seq["mode"].as<int>();
            record_state.push_back(_motion);
        }


        auto time_file = addTimeToFilename(filename);
        auto time_filepath = home / time_file;

        YAML::Node time_docs = YAML::LoadFile(time_filepath);
        const YAML::Node& time_sequences = time_docs["sequences"];
        for (const auto& seq : time_sequences) 
        {
            start_time.push_back(seq["start_time"].as<double>());
            end_time.push_back(seq["end_time"].as<double>());
        }
        return true;    
    }

    std::string addTimeToFilename(const std::string& original) {
        std::string result = original;
        size_t pos = result.rfind(".yaml");
        if (pos != std::string::npos) {
            result.insert(pos, "_time");
        }
        return result;
    }

    std::vector<float> start_time;
    std::vector<float > end_time;
    std::vector<goyoung_msgs::msg::Robot> record_state;
    goyoung_msgs::msg::Robot robot_state_;
    int motor_numbering[9] = {5, 3, 6, 4, 9, 7, 11, 10, 8};

    goyoung_msgs::msg::Robot start_motion;
    bool is_start = false;

    rclcpp::Publisher<goyoung_msgs::msg::Robot>::SharedPtr motion_publisher_;  // motion을 publish하는 publisher
    
    // rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr video_publisher_;
    rclcpp::Service<goyoung_msgs::srv::Motionexecute>::SharedPtr motion_service_;  // motion 실행을 위한 service

    rclcpp::Subscription<goyoung_msgs::msg::Robot>::SharedPtr motor_subscription_; // motor state를 받아오는 subscriber
    
    rclcpp::Subscription<goyoung_msgs::msg::Checkpoint>::SharedPtr end_pose_subscriber_;
    const int TIME_HZ = 10;

    bool end_pose = false;
    // rclcpp::Subscription<goyoung_msgs::msg::Motionpath>::SharedPtr motion_subscriber_;

#ifdef __version1__

    bool readSequencefile(std::string filename)
    {
        auto path = filename;
        auto home = std::filesystem::path(std::getenv("HOME"));
        auto filepath = home / path;

        std::ifstream fin(filepath);
        if (!fin.is_open()) {
            std::cerr << "Error opening file" << std::endl;
            return -1;
        }
        std::vector<YAML::Node> docs = YAML::LoadAll(fin);

        int length = 0;
        if (docs.size() > 0 && docs[0]["length"]) {
            length = docs[0]["length"].as<int>();
            std::cout << "Length: " << length << std::endl;
        }

        for (size_t i = 1; i < docs.size(); ++i) 
        {
            if (docs[i]["motion"]) 
            {
                for (const auto& motion : docs[i]["motion"]) 
                {
                    std::cout << "Motion" << std::endl;
                    std::string _file;
                    float _t = 0.0;
                    if (motion["filename"]) 
                    {
                        _file = motion["filename"].as<std::string>();
                        std::cout << "Filename: " << _file << std::endl;
                    }
                    if (motion["start_time"])
                    {
                        _t = motion["start_time"].as<float>();
                        std::cout << "Start Time: " << _t << std::endl;
                    }
                    this->readMotionfile(_file, _t);    
                }
            }
        }
        
        return true;
    }

    bool readMotionfile(std::string filename, float start_time)
    {
        auto path = "ros2_ws/src/goyoung/goyoung_motion/save_file/" + filename;
        auto home = std::filesystem::path(std::getenv("HOME"));
        auto filepath = home / path;
        std::ifstream fin(filepath);
        std::cout << filepath << std::endl;
        if (!fin.is_open()) 
        {
            std::cerr << "Error opening file" << std::endl;
            return false;
        }
        std::vector<YAML::Node> docs = YAML::LoadAll(fin);
        int length = 0;
        if (docs.size() > 0 && docs[0]["size"]) {
            length = docs[0]["size"].as<int>();
            std::cout << "Motion Length: " << length << std::endl;
            
            float _time = start_time;

            for (size_t i = 1; i < docs.size(); ++i) 
            {
                if (docs[i]["sequence"]) {
                    goyoung_msgs::msg::Robot _motion;
                    for (const auto& sequence : docs[i]["sequence"]) {
                        std::cout << "in " << std::endl;
                        if (sequence["start"]){
                            _time = start_time + sequence["start"].as<float>();
                        }
                        if (sequence["left_motor1"]) {
                            _motion.left.m1 = sequence["left_motor1"].as<float>();
                        }
                        if (sequence["left_motor2"]) {
                            _motion.left.m2 = sequence["left_motor2"].as<float>();
                        }
                        if (sequence["left_motor3"]) {
                            _motion.left.m3 = sequence["left_motor3"].as<float>();
                        }
                        if (sequence["right_motor1"]) {
                            _motion.right.m1 = sequence["right_motor1"].as<float>();
                        }
                        if (sequence["right_motor2"]) {
                            _motion.right.m2 = sequence["right_motor2"].as<float>();
                        }
                        if (sequence["right_motor3"]) {
                            _motion.right.m3 = sequence["right_motor3"].as<float>();
                        }
                        if (sequence["chest"]) {
                            _motion.chest = sequence["chest"].as<float>();
                        }
                    }
                    record_state.push_back(_motion);
                    record_time.push_back(_time);
                }
            }
        }
    }

    bool motion_start()
    {
        std::cout << "motion start" << std::endl;

        if (record_time.empty() || record_state.empty()) 
        {
            std::cerr << "Error: record_time or record_state is empty" << std::endl;
            return false;
        }

        auto start_time = std::chrono::steady_clock::now();
        std::cout << record_time.size() << std::endl;
        for (unsigned int i = 0; i < record_time.size(); ++i) 
        {
            auto next_time = start_time + std::chrono::milliseconds(static_cast<int>(record_time[i] * 1000));
            std::this_thread::sleep_until(next_time);
            this->motion_publisher_->publish(record_state[i]);
            std::cout << "time: " << record_time[i] << std::endl;
        }
        return true;
    }
#endif // __version1__
};
