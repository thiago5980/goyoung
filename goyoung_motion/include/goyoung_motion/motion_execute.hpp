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
        // start_motion.left.m1 = 0.0;
        // start_motion.left.m2 = 0.0;
        // start_motion.left.m3 = 0.0;
        // start_motion.left.m4 = 0.0;
        // start_motion.right.m1 = 0.0;
        // start_motion.right.m2 = 0.0;
        // start_motion.right.m4 = 0.0;
        // start_motion.chest = 0.0;

        motor_subscription_ = this->create_subscription<goyoung_msgs::msg::Robot>(
          "initial_motor_state",
          rclcpp::SensorDataQoS(),
          std::bind(&MotionExecute::motor_cb_message, this, _1));
        motion_publisher_ = this->create_publisher<goyoung_msgs::msg::Robot>("start_motion", rclcpp::SensorDataQoS());
        // video_publisher_ = this->create_publisher<std_msgs::msg::Int8>("change_video", rclcpp::SensorDataQoS());
        motion_service_ = this->create_service<goyoung_msgs::srv::Motionexecute>(
          "motion_path",
          std::bind(&MotionExecute::motion_cb_message, this, _1, _2));
        end_pose_subscriber_ = this->create_subscription<goyoung_msgs::msg::Checkpoint>(
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
    
    void motor_cb_message(const goyoung_msgs::msg::Robot::SharedPtr msg)
    {
        this->robot_state_ = *msg;
        // std::cout << "motor state" << std::endl;
        // std::cout << "left motor1: " << this->robot_state_.left.m1 << std::endl;
        // std::cout << "left motor2: " << this->robot_state_.left.m2 << std::endl;
        // std::cout << "left motor3: " << this->robot_state_.left.m3 << std::endl;
        // std::cout << "left motor4: " << this->robot_state_.left.m4 << std::endl;
        // std::cout << "right motor1: " << this->robot_state_.right.m1 << std::endl;
        // std::cout << "right motor2: " << this->robot_state_.right.m2 << std::endl;
        // std::cout << "right motor3: " << this->robot_state_.right.m3 << std::endl;
        // std::cout << "right motor4: " << this->robot_state_.right.m4 << std::endl;
        // std::cout << "chest: " << this->robot_state_.chest << std::endl;
        is_start = true;
    }
    void motion_cb_message(const goyoung_msgs::srv::Motionexecute::Request::SharedPtr request,
                           goyoung_msgs::srv::Motionexecute::Response::SharedPtr response)
    {
        // RCLCPP_INFO(this->get_logger(), "motion start");
        std::cout << "motion start" << std::endl;
        this->readMotionfile(request->file);
        if (is_start == false)
        {
            std::cerr << "Error: initial_motor_state is empty" << std::endl;
            return;
        }

        else
        {
            // auto future = std::async(std::launch::async, &MotionExecute::motion_start, this, request->file);
            // auto send = future.get();
            auto send = this->motion_start(request->file);
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
        // std::cout << record_state.size() << " " << start_time.size() << " " << end_time.size() << std::endl;
        if (record_state.empty() || start_time.empty() || end_time.empty())
        {
            std::cerr << "Error: record_state or start_time or end_time is empty" << std::endl;
            return false;
        }
        std::chrono::milliseconds interval(1000 / TIME_HZ);

        auto start = std::chrono::steady_clock::now();
        auto send_motion = goyoung_msgs::msg::Robot();
        
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
        ////// bull shit state ///// origin state is 2-state
        // if (filename == "origin.yaml")
        // {
        //     send_motion.left.m1_v = 0;
        //     send_motion.left.m2_v = 0;
        // }
        // ////// bull shit state /////
        // auto v_msg = std_msgs::msg::Int8();
        // v_msg.data = send_motion.num;
        // this->video_publisher_->publish(v_msg);
        this->motion_publisher_->publish(send_motion);
        while ((std::chrono::steady_clock::now() - start < std::chrono::milliseconds(static_cast<int>(end_time[0] * 1000))) && rclcpp::ok())
        {
            // rclcpp::spin_some(this->get_node_base_interface());
            // std::cout << "in wait\n";
            rclcpp::sleep_for(std::chrono::milliseconds(interval));
            // std::this_thread::sleep_for(interval);
        }
        
        for (unsigned int i=1; i<start_time.size(); i++)
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
                // rclcpp::spin_some(this->get_node_base_interface());
                rclcpp::sleep_for(std::chrono::milliseconds(interval));
            }
            // if (end_pose)
            // {
            //     end_pose = false;
            //     break;
            // }
        }
        return true;
    }

    bool readMotionfile(std::string filename)
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
        size_t pos = result.rfind(".yaml"); // 파일 확장자 시작 부분 찾기
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

    rclcpp::Publisher<goyoung_msgs::msg::Robot>::SharedPtr motion_publisher_;
    // rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr video_publisher_;
    rclcpp::Service<goyoung_msgs::srv::Motionexecute>::SharedPtr motion_service_;
    rclcpp::Subscription<goyoung_msgs::msg::Robot>::SharedPtr motor_subscription_;
    
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
