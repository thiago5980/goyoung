#pragma once
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
#include "goyoung_msgs/msg/robot.hpp"
#include "goyoung_msgs/srv/motortorque.hpp"
#include "goyoung_msgs/msg/mode.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

class CanNode : public rclcpp::Node
{   


public:
    CanNode() : Node("can_node_init")
    {
        this->declare_parameter("debug", false); // debug mode에 따라 모션 생성 시퀀스로 실행할지 아니면 로봇 구동 시퀀스로 실행할지 여부 결정
        this->debug = this->get_parameter("debug").as_bool();
        if (this->debug)
            RCLCPP_INFO(this->get_logger(), "Debug Mode");
        else
            RCLCPP_INFO(this->get_logger(), "Release Mode");

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // CAN 통신을 위한 socket 생성
        if (s < 0) {
            perror("Socket");
            return;
        }

        strcpy(ifr.ifr_name, "can0"); // can0 인터페이스를 사용
        
        ioctl(s, SIOCGIFINDEX, &ifr); // ifr 구조체를 통해 can0 인터페이스의 인덱스를 가져옴


        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Bind");
            return;
        }
        configure_socket_timeout(s, 50); // socket timeout 설정

        
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        
        motor_toque_srv = this->create_service<goyoung_msgs::srv::Motortorque>("motor_torque", // 모터 토크를 on/off하기 위한 서비스
            
            std::bind(&CanNode::motor_toque_callback, this, _1, _2));
        motor_pub = this->create_publisher<goyoung_msgs::msg::Robot>("motor_record", rclcpp::SensorDataQoS()); // 모터값을 publish하기 위한 publisher
        initial_motor_pub = this->create_publisher<goyoung_msgs::msg::Robot>("initial_motor_state", rclcpp::SensorDataQoS()); // 초기 모터값을 publish하기 위한 publisher
        
        motor_sub = this->create_subscription<goyoung_msgs::msg::Robot>("start_motion", rclcpp::SensorDataQoS(),  // 모터값을 subscribe 
                                                                std::bind(&CanNode::motor_callback, this, _1));
        mode_sub_ = this->create_subscription<goyoung_msgs::msg::Mode>("mode", qos_profile, std::bind(&CanNode::mode_callback, this, _1)); // 모션 모드를 subscribe

        // timer_ = this->create_wall_timer(100ms, std::bind(&CanNode::timer_callback, this));
        
        can_execute();
    }
    ~CanNode() {close(s);}

private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    bool torque_on  = false;
    bool torque_off = false;

    int robot_mode = -1;

    int led_mode = -1;

    bool debug = false;

    bool led_flag = false;
    goyoung_msgs::msg::Robot message;
    rclcpp::TimerBase::SharedPtr timer_;

    goyoung_msgs::msg::Robot motor_msg;

    rclcpp::Service<goyoung_msgs::srv::Motortorque>::SharedPtr motor_toque_srv;
    rclcpp::Publisher<goyoung_msgs::msg::Robot>::SharedPtr motor_pub;
    rclcpp::Publisher<goyoung_msgs::msg::Robot>::SharedPtr initial_motor_pub;
    rclcpp::Subscription<goyoung_msgs::msg::Robot>::SharedPtr motor_sub;
    rclcpp::Subscription<goyoung_msgs::msg::Mode>::SharedPtr mode_sub_;

    unsigned int m_id[9] = {0x145, 0x143, 0x146, 0x144, 0x149, 0x147, 0x14B, 0x141, 0x148};
    const int LENGTH = 9;

    void configure_socket_timeout(int socket_fd, int timeout_ms) 
    {
        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    }
    // void timer_callback()
    // {
    //     motor_pub->publish(message);
    // }
    // unsigned int m_id[2] = {0x142, 0x141};
    // unsigned int m_id[5] = {0x149, 0x144, 0x146, 0x143, 0x145};
    // unsigned int m_id[5] = {0x145, 0x143, 0x146, 0x144, 0x149};
    // unsigned int m_id[1] = {0x145};
    bool motor_torque_off = false;
    
    void can_execute()
    {
        while(rclcpp::ok())
        {
            if (torque_on && ((robot_mode == 2) || !this->debug)) // yaml파일에 저장된 모터 값을 goyoung motion node에서 받아와 로봇을 구동
            {
                for (int i=0; i<LENGTH; i++)
                {
                    // std::cout << "motor execute\n";
                    // int16_t speed_data = static_cast<int16_t>(600); // setting speed 300
                    int16_t speed_data = static_cast<int16_t>(600); // setting speed
                    float motorAngle = 0.0; // motor angle
                    

                    if (i==0) // chest
                    {
                        motorAngle = this->motor_msg.chest; // subscribe한 모터 angle 값
                        speed_data = static_cast<int16_t>(this->motor_msg.chest_v); // subscribe한 모터 speed 값
                    
                    }
                    else if (i==1) // left m1
                    {
                        motorAngle = this->motor_msg.left.m1;
                        speed_data = static_cast<int16_t>(this->motor_msg.left.m1_v); 
                    }
                    else if (i==2) // left m2
                    {
                        motorAngle = this->motor_msg.left.m2;
                        speed_data = static_cast<int16_t>(this->motor_msg.left.m2_v); 
                    }
                    else if (i==3) // left m3
                    {
                        motorAngle = this->motor_msg.left.m3;
                        speed_data = static_cast<int16_t>(this->motor_msg.left.m3_v); 
                    }
                    else if (i==4) // left m4
                    {
                        motorAngle = this->motor_msg.left.m4;
                        speed_data = static_cast<int16_t>(this->motor_msg.left.m4_v); 
                    }
                    else if (i==5) // left m1
                    {
                        motorAngle = this->motor_msg.right.m1;
                        speed_data = static_cast<int16_t>(this->motor_msg.right.m1_v);
                    }
                    else if (i==6) // right m2
                    {
                        motorAngle = this->motor_msg.right.m2;
                        speed_data = static_cast<int16_t>(this->motor_msg.right.m2_v);
                    }
                    else if (i==7) // right m3
                    {
                        motorAngle = this->motor_msg.right.m3; 
                        speed_data = static_cast<int16_t>(this->motor_msg.right.m3_v);
                    }
                        
                    else if (i==8) // right m4
                    {
                        motorAngle = this->motor_msg.right.m4;
                        speed_data = static_cast<int16_t>(this->motor_msg.right.m4_v);
                    }
                    if (speed_data < 1)
                        speed_data = 1;
                    struct can_frame s_frame;
                    s_frame.can_dlc = 8;
                    s_frame.can_id = static_cast<canid_t>(m_id[i]);
                    s_frame.data[0] = 0xA4;
                    s_frame.data[1] = 0x00;
                    
                    std::memcpy((s_frame.data + 2), &speed_data, 2);

                    int32_t s_data = static_cast<int32_t>(motorAngle * 100);
                    std::memcpy((s_frame.data + 4), &s_data, 4);

                    if (write(s, &s_frame, sizeof(s_frame)  ) != sizeof(s_frame)) // motor에 명령어 전달
                    {
                        perror("Write Error in get motor data");
                        close(s);
                        rclcpp::sleep_for(std::chrono::milliseconds(500));
                        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                        if (s < 0) {
                            perror("Socket");
                            return;
                        }
                        ioctl(s, SIOCGIFINDEX, &ifr);
                        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                            perror("Bind");
                            return;
                        }
                        configure_socket_timeout(s, 50);
                        continue;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(3));

                    struct can_frame r_frame;
                    r_frame.can_dlc = 8;
                    r_frame.can_id = static_cast<canid_t>(m_id[i]);
                            
                    if (read(s, &r_frame, sizeof(r_frame)) < 0) {
                        perror("Read Error");
                        close(s);
                        // 재연결 로직
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                        if (s < 0) {
                            perror("Socket");
                            return;
                        }
                        ioctl(s, SIOCGIFINDEX, &ifr);
                        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                            perror("Bind");
                            return;
                        }
                        configure_socket_timeout(s, 50);
                        continue;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(3));
                }
                if (led_flag) // LED 제어를 위한 명령어 전달
                {
                    struct can_frame l_frame;
                    l_frame.can_dlc = 8;
                    l_frame.can_id = 0x100;
                    l_frame.data[0] = 0xC1;
                    l_frame.data[1] = led_mode;
                    l_frame.data[2] = 0x00;
                    l_frame.data[3] = 0x00;
                    l_frame.data[4] = 0x00;
                    l_frame.data[5] = 0x00;
                    l_frame.data[6] = 0x00;
                    l_frame.data[7] = 0x00;
                    if (write(s, &l_frame, sizeof(l_frame)  ) != sizeof(l_frame))
                    {
                        perror("Write Error in get motor data");
                        close(s);
                        rclcpp::sleep_for(std::chrono::milliseconds(500));
                        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                        if (s < 0) {
                            perror("Socket");
                            return;
                        }
                        ioctl(s, SIOCGIFINDEX, &ifr);
                        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                            perror("Bind");
                            return;
                        }
                        configure_socket_timeout(s, 50);
                        continue;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(3));

                    led_flag = false;
                }
            }
            else if(torque_off && robot_mode == 1) // 모션 저장을 위한 sequnece
            {
                auto message = goyoung_msgs::msg::Robot();
                for (int i = 0; i<LENGTH; i++)
                {
                    struct can_frame s_frame;
                    s_frame.can_dlc = 8;
                    s_frame.can_id = static_cast<canid_t>(m_id[i]);
                    s_frame.data[0] = 0x92;
                    s_frame.data[1] = 0x00;
                    s_frame.data[2] = 0x00;
                    s_frame.data[3] = 0x00;
                    s_frame.data[4] = 0x00;
                    s_frame.data[5] = 0x00;
                    s_frame.data[6] = 0x00;
                    s_frame.data[7] = 0x00;
                    if (write(s, &s_frame, sizeof(s_frame)  ) != sizeof(s_frame)) // 모터 값을 읽어오는 명령어 전달
                    {
                        perror("Write Error in get motor data");
                        close(s);
                        rclcpp::sleep_for(std::chrono::milliseconds(3));
                        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                        if (s < 0) {
                            perror("Socket");
                            return;
                        }
                        ioctl(s, SIOCGIFINDEX, &ifr);
                        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                            perror("Bind");
                            return;
                        }
                        configure_socket_timeout(s, 50);
                        continue;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(3));

                    float motorAngle = 0.0;
                    
                    struct can_frame r_frame;
                    r_frame.can_dlc = 8;
                    if (read(s, &r_frame, sizeof(r_frame)) < 0) {
                        perror("Read Error");
                        close(s);
                        // 재연결 로직
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                        if (s < 0) {
                            perror("Socket");
                            return;
                        }
                        ioctl(s, SIOCGIFINDEX, &ifr);
                        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                            perror("Bind");
                            return;
                        }
                        configure_socket_timeout(s, 50);
                        continue;
                    }
                    rclcpp::sleep_for(std::chrono::milliseconds(3));
                    
                    if (r_frame.can_id == static_cast<canid_t>(m_id[i])) // 읽어온 데이터를 가공
                    {
                        int64_t m_data = 0;
                        bool isNegative = r_frame.data[1] & 0x80;
                        // std::memcpy(&m_data, (r_frame.data + 2), 6);
                        // std::cout << m_data << std::endl;
                        // float motorAngle = static_cast<float>(m_data) * 0.01f;
                        for (int i=0; i<7; ++i)
                        {
                            m_data |= static_cast<int64_t>(r_frame.data[i+1]) << (8 * i);

                        }
                        if (r_frame.data[7] & 0x80)
                        {
                            if (m_data & (1LL << 55)) 
                            { 
                                m_data |= (~0ULL) << 56;
                            }
                            m_data = ~m_data + 1;
                            m_data = -m_data;
                        }

                        motorAngle = static_cast<float>(m_data) * 0.01f;
                        // std::cout << "Motor Angle : " << motorAngle << std::endl;
                    }
                    if (i==0) // chest                      읽어온 데이터를 message에 저장
                        message.chest = motorAngle;
                    else if (i==1) // left m1
                        message.left.m1 = motorAngle;
                    else if (i==2) // left m2
                        message.left.m2 = motorAngle;
                    else if (i==3) // left m3
                        message.left.m3 = motorAngle;
                    else if (i==4) // left m4
                        message.left.m4 = motorAngle;
                    else if (i==5) // left m1
                        message.right.m1 = motorAngle;
                    else if (i==6) // right m2
                        message.right.m2 = motorAngle;
                    else if (i==7) // right m3
                        message.right.m3 = motorAngle;
                    else if (i==8) // right m4
                        message.right.m4 = motorAngle;
                    rclcpp::sleep_for(std::chrono::milliseconds(3));
                    // RCLCPP_INFO(this->get_logger(), "motor data : %f", motorAngle);
                }
                motor_pub->publish(message);    // 읽어온 데이터를 goyougn motion record 노드에 전달
                rclcpp::sleep_for(std::chrono::milliseconds(3));

                // std::cout << "publish motor data\n";
            }
            else
            {
                if (motor_torque_off == false) // 로봇 구동을 위해 torque값을 on/off하기 위한 명령어 전달
                {
                    std::cout << "torque off/on is not set\n";
                    for(int i=0; i<LENGTH; i++)
                    {
                        struct can_frame frame;
                        frame.can_dlc = 8;
                        frame.can_id = static_cast<canid_t>(m_id[i]);
                        std::cout << "ID : " << m_id[i] << std::endl;
                        frame.data[0] = 0xA1;
                        frame.data[1] = 0x00;
                        frame.data[2] = 0x00;
                        frame.data[3] = 0x00;
                        frame.data[4] = 0x00;
                        frame.data[5] = 0x00;
                        frame.data[6] = 0x00;
                        frame.data[7] = 0x00;
                        std::cout << "torque off\n";
                        if (write(s, &frame, sizeof(frame)) != sizeof(frame))
                        {
                            perror("Write Error in get motor data");
                            close(s);
                            rclcpp::sleep_for(std::chrono::milliseconds(3));
                            s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                            if (s < 0) {
                                perror("Socket");
                                return;
                            }
                            ioctl(s, SIOCGIFINDEX, &ifr);
                            if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                                perror("Bind");
                                return;
                            }
                            configure_socket_timeout(s, 50);
                            continue;
                        }
                        rclcpp::sleep_for(std::chrono::milliseconds(3));

                        if (read(s, &frame, sizeof(frame)) < 0) {
                            perror("Read Error");
                            close(s);
                            // 재연결 로직
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                            if (s < 0) {
                                perror("Socket");
                                return;
                            }
                            ioctl(s, SIOCGIFINDEX, &ifr);
                            if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                                perror("Bind");
                                return;
                            }
                            configure_socket_timeout(s, 50);
                            continue;
                        }
                        std::cout << "Received message with ID : " << std::hex << frame.can_id << std::endl;
                        rclcpp::sleep_for(std::chrono::milliseconds(3));
                    }
                    motor_torque_off = true;
                }
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void motor_callback(const goyoung_msgs::msg::Robot::SharedPtr msg)
    {
        if (msg.get() == nullptr)
        {
            return;
        }
        this->motor_msg = *msg;
        if (this->led_mode != msg->num)
        {
            this->led_mode = msg->num;
            this->led_flag = true;
        }
    }
    void mode_callback(const goyoung_msgs::msg::Mode::SharedPtr msg)
    {
        robot_mode = msg->mode;
        std::cout << "mode : " << robot_mode << std::endl;
    }

    void motor_toque_callback(const goyoung_msgs::srv::Motortorque::Request::SharedPtr request,
                                    goyoung_msgs::srv::Motortorque::Response::SharedPtr response)
    {
        if (request->torquechange && request->mode>0)
        {
            motor_torque_off = false;
            std::cout << "mode : " << int(request->mode) << std::endl;
            torque_on = false;
            torque_off = false;
            // auto message = goyoung_msgs::msg::Robot();
            for(int i=0; i<LENGTH; i++)
            {
                struct can_frame frame;
                frame.can_dlc = 8;
                frame.can_id = static_cast<canid_t>(m_id[i]);
                std::cout << "ID : " << m_id[i] << std::endl;
                if (request->mode == 1)
                {
                    frame.data[0] = 0xA1;
                    frame.data[1] = 0x00;
                    frame.data[2] = 0x00;
                    frame.data[3] = 0x00;
                    frame.data[4] = 0x00;
                    frame.data[5] = 0x00;
                    frame.data[6] = 0x00;
                    frame.data[7] = 0x00;
                    std::cout << "torque off\n";
                }
                else if (request->mode == 2)
                {
                    frame.data[0] = 0x81;
                    frame.data[1] = 0x00;
                    frame.data[2] = 0x00;
                    frame.data[3] = 0x00;
                    frame.data[4] = 0x00;
                    frame.data[5] = 0x00;
                    frame.data[6] = 0x00;
                    frame.data[7] = 0x00;
                    std::cout << "torque on\n";
                }
                if (write(s, &frame, sizeof(frame)) != sizeof(frame))
                {
                    perror("Write Error in get motor data");
                    close(s);
                    rclcpp::sleep_for(std::chrono::milliseconds(3));
                    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                    if (s < 0) {
                        perror("Socket");
                        return;
                    }
                    ioctl(s, SIOCGIFINDEX, &ifr);
                    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                        perror("Bind");
                        return;
                    }
                    configure_socket_timeout(s, 50);
                    continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(3));

                std::cout << "Message sent : " << m_id[i] << std::endl;

                if (read(s, &frame, sizeof(frame)) < 0) {
                    perror("Read Error");
                    close(s);
                    // 재연결 로직
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                    if (s < 0) {
                        perror("Socket");
                        return;
                    }
                    ioctl(s, SIOCGIFINDEX, &ifr);
                    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                        perror("Bind");
                        return;
                    }
                    configure_socket_timeout(s, 50);
                    continue;
                }

                std::cout << "Received message with ID : " << std::hex << frame.can_id << std::endl;
                // std::cout << "Data : ";
                // for (int i = 0; i < frame.can_dlc; ++i)
                //     std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
                // std::cout << std::endl;

                rclcpp::sleep_for(std::chrono::milliseconds(3));

                std::cout << "initial pose read\n";
                struct can_frame s_frame;
                s_frame.can_dlc = 8;
                s_frame.can_id = static_cast<canid_t>(m_id[i]);
                s_frame.data[0] = 0x92;
                s_frame.data[1] = 0x00;
                s_frame.data[2] = 0x00;
                s_frame.data[3] = 0x00;
                s_frame.data[4] = 0x00;
                s_frame.data[5] = 0x00;
                s_frame.data[6] = 0x00;
                s_frame.data[7] = 0x00;
                if (write(s, &s_frame, sizeof(s_frame)) != sizeof(s_frame))
                {
                    perror("Write Error in get motor data");
                    close(s);
                    rclcpp::sleep_for(std::chrono::milliseconds(3));
                    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                    if (s < 0) {
                        perror("Socket");
                        return;
                    }
                    ioctl(s, SIOCGIFINDEX, &ifr);
                    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                        perror("Bind");
                        return;
                    }
                    configure_socket_timeout(s, 50);
                    continue;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(3));

                float motorAngle = 0.0;
                
                struct can_frame r_frame;
                r_frame.can_dlc = 8;
                if (read(s, &r_frame, sizeof(r_frame)) < 0) {
                    perror("Read Error");
                    close(s);
                    // 재연결 로직
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                    if (s < 0) {
                        perror("Socket");
                        return;
                    }
                    ioctl(s, SIOCGIFINDEX, &ifr);
                    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                        perror("Bind");
                        return;
                    }
                    configure_socket_timeout(s, 50);
                    continue;
                }

                if (r_frame.can_id == static_cast<canid_t>(m_id[i]))
                {
                    int64_t m_data = 0;
                    bool isNegative = r_frame.data[1] & 0x80;
                    // std::memcpy(&m_data, (r_frame.data + 2), 6);
                    // std::cout << m_data << std::endl;
                    // float motorAngle = static_cast<float>(m_data) * 0.01f;
                    for (int i=0; i<7; ++i)
                    {
                        m_data |= static_cast<int64_t>(r_frame.data[i+1]) << (8 * i);

                    }
                    if (r_frame.data[7] & 0x80)
                    {
                        if (m_data & (1LL << 55)) 
                        { 
                            m_data |= (~0ULL) << 56;
                        }
                        m_data = ~m_data + 1;
                        m_data = -m_data;
                    }

                    motorAngle = static_cast<float>(m_data) * 0.01f;
                    std::cout << "Motor Angle : " << motorAngle << std::endl;
                }
                if (i==0) // chest
                    message.chest = motorAngle;
                else if (i==1) // left m1
                    message.left.m1 = motorAngle;
                else if (i==2) // left m2
                    message.left.m2 = motorAngle;
                else if (i==3) // left m3
                    message.left.m3 = motorAngle;
                else if (i==4) // left m4
                    message.left.m4 = motorAngle;
                else if (i==5) // left m1
                    message.right.m1 = motorAngle;
                else if (i==6) // right m2
                    message.right.m2 = motorAngle;
                else if (i==7) // right m3
                    message.right.m3 = motorAngle;
                else if (i==8) // right m4
                    message.right.m4 = motorAngle;
                rclcpp::sleep_for(std::chrono::milliseconds(3));
            }

            if (request->mode == 1)
                torque_off = true;
            else if ((request->mode == 2) || (request->mode == 3))
            {
                initial_motor_pub->publish(message);
                torque_on = true;
            }
            response->success = true;
        }
    }
};