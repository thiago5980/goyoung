#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/robot.hpp"
#include "goyoung_msgs/msg/checkpoint.hpp"
#include "goyoung_msgs/msg/savefile.hpp"
#include "goyoung_msgs/srv/motionexecute.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

class MotionRecord : public rclcpp::Node
{
public:
    MotionRecord()
    : Node("motion_record")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        motor_subscription_ = this->create_subscription<goyoung_msgs::msg::Robot>(
          "motor_record",
          rclcpp::SensorDataQoS(),
          std::bind(&MotionRecord::motor_cb_message, this, _1));
#ifdef __version1__
        check_point_subscription_ = this->create_subscription<goyoung_msgs::msg::Checkpoint>(
          "get_check_point",
          qos_profile,
          std::bind(&MotionRecord::check_point_cb_message, this, _1));
#endif // __version1__

        motion_check_point_sub = this->create_subscription<goyoung_msgs::msg::Checkpoint>(
          "motion_checkpoint",
          qos_profile,
          std::bind(&MotionRecord::motion_cb_checkpoint, this, _1));
        
        save_service_ = this->create_service<goyoung_msgs::srv::Motionexecute>(
          "motion_save",
          std::bind(&MotionRecord::save_file_srv_cb, this, _1, _2));

    }
private:
    void reset()
    {
        this->record_state_.clear();
        this->record_time_.clear();
        this->save_flag_ = false;
        this->seconds_ = 0.0;
    }
    void motor_cb_message(const goyoung_msgs::msg::Robot::SharedPtr msg)
    {
        this->robot_state_ = *msg;
    }
    void motion_cb_checkpoint(const goyoung_msgs::msg::Checkpoint::SharedPtr msg)
    {
        std::cout << "checkpoint" << std::endl;
        if (msg->check == true)
            this->record_state_.push_back(this->robot_state_);
    }
    void save_file_srv_cb(const goyoung_msgs::srv::Motionexecute::Request::SharedPtr request,
                           goyoung_msgs::srv::Motionexecute::Response::SharedPtr response)
    {
        auto file_name = request->file;
        std::cout << file_name << std::endl;
        if (this->save_yml_file(file_name))
            response->success = true;
        else
            response->success = false;
    }
    bool save_yml_file(const std::string filename)
    {
        std::cout << "save file" << std::endl;
        if (filename.empty() || record_state_.empty()) 
        {
            return false;
        }

        auto file_name = filename;
        YAML::Emitter out;

        out << YAML::BeginMap;
        out << YAML::Key << "size" << YAML::Value << std::to_string(record_state_.size());
        out << YAML::Key << "sequences" << YAML::Value << YAML::BeginSeq;
        for (unsigned int i=0; i<record_state_.size(); i++)
        {
            auto left = record_state_[i].left;
            auto right = record_state_[i].right;
            auto chest = record_state_[i].chest;

            out << YAML::BeginMap;
            out << YAML::Key << "action" << YAML::Value << i;
            out << YAML::Key << "left_motor1" << YAML::Value << left.m1;
            out << YAML::Key << "left_motor2" << YAML::Value << left.m2;
            out << YAML::Key << "left_motor3" << YAML::Value << left.m3;
            out << YAML::Key << "left_motor4" << YAML::Value << left.m4;
            out << YAML::Key << "right_motor1" << YAML::Value << right.m1;
            out << YAML::Key << "right_motor2" << YAML::Value << right.m2;
            out << YAML::Key << "right_motor3" << YAML::Value << right.m3;
            out << YAML::Key << "right_motor4" << YAML::Value << right.m4;
            out << YAML::Key << "chest" << YAML::Value << chest;
            out << YAML::Key << "mode" << YAML::Value << 0;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;

        auto path = file_name;
        
        auto home = std::filesystem::path(std::getenv("HOME"));
        auto filepath = home / path;
        std::cout << "file path: " << filepath << std::endl; // "file path: /home/goyoung/ros2_ws/src/goyoung/goyoung_motion/save_file/record.yaml

        std::ofstream fout(filepath);
        fout << out.c_str();
        
        reset();

        return true;
    }
#ifdef __version1__
    rclcpp::Subscription<goyoung_msgs::msg::Checkpoint>::SharedPtr check_point_subscription_;

    void check_point_cb_message(const goyoung_msgs::msg::Checkpoint::SharedPtr msg)
    {
        if (msg->check == true)
        {
            if (save_flag_ == false)
            {
                save_flag_ = true;
                start_time_ = std::chrono::steady_clock::now(); 
                this->record_state_.push_back(this->robot_state_);
                this->record_time_.push_back(seconds_);
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                seconds_ = duration_cast<milliseconds>(now - start_time_).count() / 1000.0f;
                this->record_state_.push_back(this->robot_state_);
                this->record_time_.push_back(seconds_);
            }
        }
    }
    void save_file_srv_cb(const goyoung_msgs::srv::Motionexecute::Request::SharedPtr request,
                           goyoung_msgs::srv::Motionexecute::Response::SharedPtr response)
    {
        auto file_name = request->file;
        std::cout << file_name << std::endl;
        if (this->save_yml_file(file_name))
            response->success = true;
        else
            response->success = false;
    }

    bool save_yml_file(const std::string filename)
    {
        std::cout << "save file" << std::endl;
        if (filename.empty() || record_state_.empty() || record_time_.empty()) 
        {
            return false;
        }

        auto file_name = filename;
        YAML::Emitter out;

        out << YAML::BeginMap;
        out << YAML::Key << "size" << YAML::Value << std::to_string(record_state_.size());
        out << YAML::EndMap;

        for (unsigned int i=0; i<record_state_.size(); i++)
        {
            auto left = record_state_[i].left;
            auto right = record_state_[i].right;
            auto chest = record_state_[i].chest;

            out << YAML::BeginMap;
            out << YAML::Key << "sequence" << YAML::Value << YAML::BeginSeq;
            out << YAML::BeginMap;
            out << YAML::Key << "start" << YAML::Value << record_time_[i];
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "left_motor1" << YAML::Value << left.m1; // motor1 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "left_motor2" << YAML::Value << left.m2; // motor2 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "left_motor3" << YAML::Value << left.m3; // motor2 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "right_motor1" << YAML::Value << right.m1; // motor2 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "right_motor2" << YAML::Value << right.m2; // motor2 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "right_motor3" << YAML::Value << right.m3; // motor2 값
            out << YAML::EndMap;
            out << YAML::BeginMap;
            out << YAML::Key << "chest" << YAML::Value << chest; // motor2 값
            out << YAML::EndMap;
            out << YAML::EndSeq;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;

        auto path = file_name;
        
        auto home = std::filesystem::path(std::getenv("HOME"));
        auto filepath = home / path;
        std::cout << "file path: " << filepath << std::endl; // "file path: /home/goyoung/ros2_ws/src/goyoung/goyoung_motion/save_file/record.yaml

        std::ofstream fout(filepath);
        fout << out.c_str();
        
        reset();

        return true;
    }
#endif // __version1__



    rclcpp::Subscription<goyoung_msgs::msg::Robot>::SharedPtr motor_subscription_;
    rclcpp::Subscription<goyoung_msgs::msg::Savefile>::SharedPtr savefile_subscription_;
    rclcpp::Subscription<goyoung_msgs::msg::Checkpoint>::SharedPtr motion_check_point_sub   ;

    rclcpp::Service<goyoung_msgs::srv::Motionexecute>::SharedPtr save_service_;
    goyoung_msgs::msg::Robot robot_state_;
    std::vector<goyoung_msgs::msg::Robot> record_state_;
    std::vector<float> record_time_;

    std::chrono::steady_clock::time_point start_time_; 
    double seconds_ = 0.0; 
    bool save_flag_ = false;
};
