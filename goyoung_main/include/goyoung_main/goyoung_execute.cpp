#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "goyoung_msgs/msg/mode.hpp"
#include "goyoung_msgs/msg/checkpoint.hpp"
#include "goyoung_msgs/msg/savefile.hpp"

#include "goyoung_msgs/srv/motionexecute.hpp"
#include "goyoung_msgs/srv/motortorque.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

class GoyoungExecute : public rclcpp::Node
{
public:
    GoyoungExecute()
    : Node("goyoung_execute")
    {
        this->declare_parameter("debug", false);
        this->debug = this->get_parameter("debug").as_bool();
        if (this->debug)
            RCLCPP_INFO(this->get_logger(), "Debug Mode");
        else
            RCLCPP_INFO(this->get_logger(), "Release Mode");
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        mode_sub_ = this->create_subscription<goyoung_msgs::msg::Mode>("mode", qos_profile, std::bind(&GoyoungExecute::mode_callback, this, _1)); // GUI 실행에 따른 모드 변경
#ifdef __version1
        checkpoint_pub_ = this->create_publisher<goyoung_msgs::msg::Checkpoint>("get_check_point", qos_profile);
#endif // !__version1
        savefile_subscription_ = this->create_subscription<goyoung_msgs::msg::Savefile>( // 저장된 파일 경로를 읽어오기 위한 subscription
          "save_file",
          qos_profile,
          std::bind(&GoyoungExecute::save_file_cb_message, this, _1));

        video_pub = this->create_publisher<std_msgs::msg::Int8>("change_video", qos_profile); // 현재는 사용하지 않음
        motion_execute_client_ = this->create_client<goyoung_msgs::srv::Motionexecute>("motion_path"); // 로봇의 모션을 실행하기 위한 client
        motion_save_client_ = this->create_client<goyoung_msgs::srv::Motionexecute>("motion_save"); // 로봇의 모션을 저장하기 위한 client
        motor_torque_client_ = this->create_client<goyoung_msgs::srv::Motortorque>("motor_torque"); // 로봇의 모터 토크를 변경하기 위한 client
        motion_mode_sub = this->create_subscription<goyoung_msgs::msg::Mode>( // 로봇의 모션 모드를 변경하기 위한 subscription
          "robot_motion_mode",
          qos_profile,
          std::bind(&GoyoungExecute::motion_mode_callback, this, _1));
        back_origin_sub = this->create_subscription<goyoung_msgs::msg::Mode>( // 로봇 실행 후 원점으로 복귀하기 위한 subscription
            "back_origin",
            qos_profile,
            std::bind(&GoyoungExecute::origin_callback, this, _1));
        end_pose_pub = this->create_publisher<goyoung_msgs::msg::Checkpoint>("end_pose", qos_profile);
        execute();
    }

private:
    void origin_callback(const goyoung_msgs::msg::Mode::SharedPtr msg) // 원점으로 복귀하기 위한 callback
    {
        RCLCPP_INFO(this->get_logger(), "Back Origin Mode: '%d'", msg->mode);
        // motion_mode = msg->mode;
        go_origin = false;
        auto _msg = goyoung_msgs::msg::Checkpoint();
        _msg.check = true;
        end_pose_pub->publish(_msg); // 원점으로 복귀하기 위한 flag publish
    }

    void motion_mode_callback(const goyoung_msgs::msg::Mode::SharedPtr msg) // 로봇의 모션 모드를 변경하기 위한 callback
    {
        RCLCPP_INFO(this->get_logger(), "Motion Mode: '%d'", msg->mode);
        motion_mode = msg->mode;
    }

    void mode_callback(const goyoung_msgs::msg::Mode::SharedPtr msg) // GUI 실행에 따른 모드 변경
    {
        RCLCPP_INFO(this->get_logger(), "Mode: '%d'", msg->mode);
        save_path = msg->file;
        robot_mode = msg->mode;
    }
    
    void save_file_cb_message(const goyoung_msgs::msg::Savefile::SharedPtr msg) // 저장된 파일 경로를 읽어오기 위한 callback
    {
        this->end_flag = msg->save;
    }

    void execute()
    {
        while (rclcpp::ok())
        {
            if ((!this->debug) || (this->robot_mode == 3)) // 로봇의 모션 모드가 3 또는 debug 모드가 아닐 경우(실제 로봇 실행 mode)
            {
                if (!go_origin) // 원점으로 복귀 되어 있지 않다면 로봇 원점으로 이동
                {
                    auto motor_request = std::make_shared<goyoung_msgs::srv::Motortorque::Request>(); // 로봇의 Torque on 동작 실행
                    motor_request->torquechange = true;
                    motor_request->mode = 2;
                    auto motor_future_result = motor_torque_client_->async_send_request(motor_request);

                    while (rclcpp::ok()) // 로봇 Torque on이 완료될 때 동안 대기
                    {
                        if (motor_future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
                            break;
                        rclcpp::spin_some(this->get_node_base_interface());
                    } 
                    auto motor_result = motor_future_result.get();
                    if (motor_result)
                        RCLCPP_INFO(this->get_logger(), "Torque Change");
                    else
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service");

                    auto request = std::make_shared<goyoung_msgs::srv::Motionexecute::Request>(); // 로봇의 원점으로 이동하기 위한 서비스 요청
                    request->file = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_motion/save_file/origin.yaml";
                    RCLCPP_INFO(this->get_logger(), "file name : %s", request->file.c_str());
                    auto future_result = motion_execute_client_->async_send_request(request);
                    while (rclcpp::ok()) 
                    {
                        if (future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) 
                            break;
                        rclcpp::spin_some(this->get_node_base_interface());
                    }

                    auto result = future_result.get();
                    if (result) 
                    {
                        go_origin = true;
                    }
                    else
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "robot motion : %d", motion_mode); // 이전과 동일하게 로봇의 torque on을 실행
                    auto motor_request = std::make_shared<goyoung_msgs::srv::Motortorque::Request>();
                    motor_request->torquechange = true;
                    motor_request->mode = 2;
                    auto motor_future_result = motor_torque_client_->async_send_request(motor_request);

                    while (rclcpp::ok())
                    {
                        if (motor_future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
                            break;
                        rclcpp::spin_some(this->get_node_base_interface());
                    } 
                    auto motor_result = motor_future_result.get();
                    if (motor_result)
                        RCLCPP_INFO(this->get_logger(), "Torque Change");
                    else
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
                    auto request = std::make_shared<goyoung_msgs::srv::Motionexecute::Request>();
                    std::string save_path_file;


                    // 로봇의 모션 모드에 따라 실행할 모션 파일을 변경
                    // 로봇 모션을 추가하거나 변경 시에 해당 부분을 사용하면 됨
                    // 저장된 로봇의 경로를 save_path_file에 저장 후 모션 모드에 따라 확장

                    if (motion_mode == 1) // 이전 goyoung tcp에 mode에 맞춘 실행
                    {
                        save_path_file = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_motion/save_file/test.yaml"; // mode에 맞는 모터 data 저장 파일
                        if (motion_mode == 1)
                            motion_mode = -1;
                    }
                    else
                        save_path_file = "/home/goyoung/ros2_ws/src/Goyoung/goyoung_motion/save_file/stay.yaml"; // shake motion
                    
                    request->file = save_path_file;
                    RCLCPP_INFO(this->get_logger(), "file name : %s", request->file.c_str());

                    auto future_result = motion_execute_client_->async_send_request(request);
                    while (rclcpp::ok()) 
                    {
                        if (future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) 
                            break;
                        rclcpp::spin_some(this->get_node_base_interface());
                    }

                    auto result = future_result.get();
                    if (result) 
                    {
                        go_origin = false;
                    }
                    else
                        RCLCPP_ERROR(this->get_logger(), "Failed to call service");

                
                }
            }
            else if (this->robot_mode == 0)
            {
                // RCLCPP_INFO(this->get_logger(), "Waiting for Mode");
            }
            else if (this->robot_mode == 1) // 로봇 모션 저장을 위한 모드
            {
                RCLCPP_INFO(this->get_logger(), "Read Motor Data and Save Motor Data");
                if (this->save_path.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "No Save Path");
                    return;
                }
                std::string save_path_file = this->save_path; // GUI로 부터 받은 저장 경로
                RCLCPP_INFO(this->get_logger(), "Save Mode Execution");


                auto motor_request = std::make_shared<goyoung_msgs::srv::Motortorque::Request>();
                motor_request->torquechange = true;
                motor_request->mode = 1;
                auto motor_future_result = motor_torque_client_->async_send_request(motor_request);

                while (rclcpp::ok())
                {
                    if (motor_future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
                        break;
                    rclcpp::spin_some(this->get_node_base_interface());
                }
                auto motor_result = motor_future_result.get();
                if (motor_result)
                    std::cout << "Result: " << motor_result->success << std::endl;
                else
                    std::cerr << "Failed to call service" << std::endl;
                RCLCPP_INFO(this->get_logger(), "Motor Torque Change");

#ifdef __version1
                const std::chrono::milliseconds interval(1000 / this->saveHz);
                auto last = std::chrono::steady_clock::now();
                auto msg = goyoung_msgs::msg::Checkpoint();

                while (rclcpp::ok() && !this->end_flag)
                {
                    auto now = std::chrono::steady_clock::now();
                    if (now - last >= interval) 
                    {
                        msg.check = true;
                        checkpoint_pub_->publish(msg);
                        last = now;
                    }
                    rclcpp::spin_some(this->get_node_base_interface());
                }
                this->end_flag = false;

                std::cout << "End Save Mode" << std::endl;
#endif // !__version1

                while (rclcpp::ok() && !this->end_flag)
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                }
                this->end_flag = false;
                
                auto request = std::make_shared<goyoung_msgs::srv::Motionexecute::Request>();
                request->file = save_path_file;
                auto future_result = motion_save_client_->async_send_request(request);

                while (rclcpp::ok())
                {
                    if (future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
                        break;
                    rclcpp::spin_some(this->get_node_base_interface());
                }

                auto result = future_result.get();
                
                if (result)
                    std::cout << "Result: " << result->success << std::endl;
                else
                    std::cerr << "Failed to call service" << std::endl;
                this->robot_mode = 0;
            }
            else if (this->robot_mode == 2) // 생성한 모션을 테스트 하기 위한 모드
            {
                if (this->save_path.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "No Execution Path");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Start Execution");
                std::string save_path_file = this->save_path; // GUI로 입력한 테스트 모드 파일

                auto motor_request = std::make_shared<goyoung_msgs::srv::Motortorque::Request>();
                motor_request->torquechange = true;
                motor_request->mode = 2;
                auto motor_future_result = motor_torque_client_->async_send_request(motor_request); // 모션 실행

                while (rclcpp::ok())
                {
                    if (motor_future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
                        break;
                    rclcpp::spin_some(this->get_node_base_interface());
                } 
                auto motor_result = motor_future_result.get();
                if (motor_result)
                    std::cout << "Result: " << motor_result->success << std::endl;
                else
                    std::cerr << "Failed to call service" << std::endl;
                RCLCPP_INFO(this->get_logger(), "Motor Torque Change");
                
                auto request = std::make_shared<goyoung_msgs::srv::Motionexecute::Request>();
                request->file = save_path_file;

                auto future_result = motion_execute_client_->async_send_request(request);
                while (rclcpp::ok()) 
                {
                    if (future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) 
                        break;
                    rclcpp::spin_some(this->get_node_base_interface());
                }

                auto result = future_result.get();
                if (result) 
                    std::cout << "Result: " << result->success << std::endl;
                else
                    std::cerr << "Failed to call service" << std::endl;
                
                this->robot_mode = 0;
                // std::cout << "Waiting for response" << std::endl;

                // save_path = "";
            }

            rclcpp::Rate(10.0).sleep(); // 10Hz
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    rclcpp::Subscription<goyoung_msgs::msg::Mode>::SharedPtr mode_sub_;
    rclcpp::Subscription<goyoung_msgs::msg::Mode>::SharedPtr motion_mode_sub;
    rclcpp::Subscription<goyoung_msgs::msg::Mode>::SharedPtr back_origin_sub;
#ifdef __version1
    rclcpp::Publisher<goyoung_msgs::msg::Checkpoint>::SharedPtr checkpoint_pub_;
    const int saveHz = 10;
#endif // !__version1
    rclcpp::Subscription<goyoung_msgs::msg::Savefile>::SharedPtr savefile_subscription_;

    // rclcpp::Publisher<goyoung_msgs::msg::Motionpath>::SharedPtr motion_path_pub_;
    rclcpp::Client<goyoung_msgs::srv::Motionexecute>::SharedPtr motion_execute_client_;
    rclcpp::Client<goyoung_msgs::srv::Motionexecute>::SharedPtr motion_save_client_;
    rclcpp::Client<goyoung_msgs::srv::Motortorque>::SharedPtr motor_torque_client_;

    rclcpp::Publisher<goyoung_msgs::msg::Checkpoint>::SharedPtr end_pose_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr video_pub;
    int8_t robot_mode = 0;
    int8_t motion_mode = -1;
    std::string save_path;

    bool end_flag = false;
    bool debug = false;
    bool go_origin = false;
};


// Mode //
// Mode : 1 = Start Execution
// Mode : 2 = Read Motor Data and Save Motor Data
